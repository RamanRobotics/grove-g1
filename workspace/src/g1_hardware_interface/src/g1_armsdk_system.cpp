/**
 * @file g1_armsdk_system.cpp
 * @brief ros2_control SystemInterface for a G1 under onboard locomotion: rt/arm_sdk plus the loco
 *        service.
 */

#include "g1_hardware_interface/g1_armsdk_system.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace g1_hardware_interface
{

namespace
{
constexpr std::chrono::milliseconds kReleaseTickPeriod{ 5 };
constexpr std::chrono::seconds      kFirstStateTimeout{ 10 };
/// Per call. The service answers in milliseconds; a longer wait means it is not there.
constexpr float kLocoRpcTimeoutS = 0.5F;

struct InWriteGuard
{
    std::atomic<bool>* flag;
    ~InWriteGuard() { flag->store(false, std::memory_order_release); }
};

bool parseDouble(
    const std::unordered_map<std::string, std::string>& params, const std::string& key, double& out)
{
    const auto it = params.find(key);
    if (it == params.end())
    {
        return false;
    }
    try
    {
        out = std::stod(it->second);
    }
    catch (const std::exception&)
    {
        return false;
    }
    return true;
}

bool parseInt(
    const std::unordered_map<std::string, std::string>& params, const std::string& key, int& out)
{
    const auto it = params.find(key);
    if (it == params.end())
    {
        return false;
    }
    try
    {
        out = std::stoi(it->second);
    }
    catch (const std::exception&)
    {
        return false;
    }
    return true;
}

bool parseBool(
    const std::unordered_map<std::string, std::string>& params, const std::string& key, bool& out)
{
    const auto it = params.find(key);
    if (it == params.end())
    {
        return false;
    }
    out = it->second == "true" || it->second == "True" || it->second == "1";
    return true;
}

std::pair<std::string, std::string> splitInterface(const std::string& interface)
{
    const auto pos = interface.rfind('/');
    if (pos == std::string::npos)
    {
        return { "", "" };
    }
    return { interface.substr(0, pos), interface.substr(pos + 1) };
}

int sdkIndexOf(const std::string& name)
{
    const auto* const it = std::find(kG1JointNames.begin(), kG1JointNames.end(), name);
    return it == kG1JointNames.end() ? -1 : static_cast<int>(it - kG1JointNames.begin());
}
}  // namespace

double nextArmSdkWeight(double weight, bool all_powered, double step) noexcept
{
    const double target = all_powered ? 1.0 : 0.0;
    if (!std::isfinite(weight))
    {
        return 0.0;
    }
    if (weight < target)
    {
        return std::min(target, weight + step);
    }
    return std::max(target, weight - step);
}

double sanitizeLocoCommand(double value, double limit) noexcept
{
    if (!std::isfinite(value))
    {
        return 0.0;
    }
    return limit > 0.0 ? std::clamp(value, -limit, limit) : value;
}

G1ArmSdkSystem::~G1ArmSdkSystem()
{
    stopLocoWorker();
    shutdownSdk();
}

hardware_interface::CallbackReturn
G1ArmSdkSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
{
    if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    const auto& info = get_hardware_info();
    const auto& hw   = info.hardware_parameters;

    // Keep empty for the same reason as G1LowCmdSystem: a value makes the SDK drop CYCLONEDDS_URI.
    if (const auto it = hw.find("network_interface"); it != hw.end())
    {
        network_interface_ = it->second;
    }
    double lowstate_timeout_ms = 0.0;
    if (!parseInt(hw, "domain_id", domain_id_) ||
        !parseDouble(hw, "lowstate_timeout_ms", lowstate_timeout_ms) ||
        !parseDouble(hw, "engage_ramp_s", engage_ramp_s_) ||
        !parseDouble(hw, "release_ramp_s", release_ramp_s_) ||
        !parseDouble(hw, "loco_command_rate_hz", loco_command_rate_hz_) ||
        !parseDouble(hw, "loco_command_duration_s", loco_command_duration_s_) ||
        !parseDouble(hw, "max_linear_speed", max_linear_speed_) ||
        !parseDouble(hw, "max_angular_speed", max_angular_speed_))
    {
        RCLCPP_ERROR(logger_, "<hardware> is missing or has an unparseable <param>");
        return hardware_interface::CallbackReturn::ERROR;
    }
    parseBool(hw, "start_locomotion", start_locomotion_);
    lowstate_timeout_s_ = lowstate_timeout_ms / 1000.0;

    if (lowstate_timeout_s_ <= 0.0 || engage_ramp_s_ <= 0.0 || release_ramp_s_ <= 0.0 ||
        loco_command_rate_hz_ <= 0.0 || loco_command_duration_s_ <= 0.0)
    {
        RCLCPP_ERROR(
            logger_,
            "lowstate_timeout_ms, engage_ramp_s, release_ramp_s, loco_command_rate_hz and "
            "loco_command_duration_s must be > 0");
        return hardware_interface::CallbackReturn::ERROR;
    }
    // The onboard controller stops the base once a command's duration runs out, so a duration
    // shorter than two send periods makes the robot stutter between commands.
    if (loco_command_duration_s_ < 2.0 / loco_command_rate_hz_)
    {
        RCLCPP_ERROR(logger_, "loco_command_duration_s must cover two loco_command_rate_hz periods");
        return hardware_interface::CallbackReturn::ERROR;
    }

    joints_.clear();
    joint_index_.clear();
    for (const auto& joint : info.joints)
    {
        Joint j;
        j.name      = joint.name;
        j.sdk_index = sdkIndexOf(joint.name);
        j.commanded = !joint.command_interfaces.empty();
        parseDouble(joint.parameters, "position_only_kp", j.position_only_gains.kp);
        parseDouble(joint.parameters, "position_only_kd", j.position_only_gains.kd);

        if (j.sdk_index < 0)
        {
            RCLCPP_ERROR(logger_, "joint '%s' is not a G1 body motor", j.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        // Without them the position-only branch would be owned and limp.
        if (j.commanded && (j.position_only_gains.kp <= 0.0 || j.position_only_gains.kd <= 0.0))
        {
            RCLCPP_ERROR(
                logger_,
                "joint '%s' needs position_only_kp and position_only_kd params > 0",
                j.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        joint_index_[j.name] = joints_.size();
        joints_.push_back(std::move(j));
    }

    for (const auto& sensor : info.sensors)
    {
        const bool has_orientation = std::any_of(
            sensor.state_interfaces.begin(),
            sensor.state_interfaces.end(),
            [](const auto& iface) { return iface.name.find("orientation") != std::string::npos; });
        if (has_orientation)
        {
            imu_data_.name = sensor.name;
            has_imu_       = true;
            break;
        }
    }

    if (info.gpios.size() != 1)
    {
        RCLCPP_ERROR(logger_, "expected exactly one <gpio>, the loco twist");
        return hardware_interface::CallbackReturn::ERROR;
    }
    loco_gpio_name_ = info.gpios.front().name;
    for (const auto name : kLocoInterfaces)
    {
        const auto& ifaces = info.gpios.front().command_interfaces;
        if (std::none_of(ifaces.begin(), ifaces.end(), [&](const auto& i) {
                return i.name == name;
            }))
        {
            RCLCPP_ERROR(
                logger_,
                "<gpio name=\"%s\"> lacks command interface '%s'",
                loco_gpio_name_.c_str(),
                std::string(name).c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    // As in G1LowCmdSystem: the checksum covers the padding, so zero every byte once.
    arm_cmd_ = std::bit_cast<unitree_hg::msg::dds_::LowCmd_>(
        std::array<std::uint32_t, sizeof(unitree_hg::msg::dds_::LowCmd_) / 4>{});

    RCLCPP_INFO(
        logger_,
        "G1ArmSdkSystem: %zu joints, %zu on rt/arm_sdk, base on the loco service",
        joints_.size(),
        static_cast<std::size_t>(std::count_if(joints_.begin(), joints_.end(), [](const Joint& j) {
            return j.commanded;
        })));
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> G1ArmSdkSystem::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> interfaces;
    for (auto& j : joints_)
    {
        interfaces.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.position_state);
        interfaces.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &j.velocity_state);
        interfaces.emplace_back(j.name, hardware_interface::HW_IF_EFFORT, &j.effort_state);
    }
    if (has_imu_)
    {
        interfaces.emplace_back(imu_data_.name, "orientation.x", &imu_data_.orientation_x);
        interfaces.emplace_back(imu_data_.name, "orientation.y", &imu_data_.orientation_y);
        interfaces.emplace_back(imu_data_.name, "orientation.z", &imu_data_.orientation_z);
        interfaces.emplace_back(imu_data_.name, "orientation.w", &imu_data_.orientation_w);
        interfaces.emplace_back(imu_data_.name, "angular_velocity.x", &imu_data_.angular_velocity_x);
        interfaces.emplace_back(imu_data_.name, "angular_velocity.y", &imu_data_.angular_velocity_y);
        interfaces.emplace_back(imu_data_.name, "angular_velocity.z", &imu_data_.angular_velocity_z);
        interfaces.emplace_back(
            imu_data_.name,
            "linear_acceleration.x",
            &imu_data_.linear_acceleration_x);
        interfaces.emplace_back(
            imu_data_.name,
            "linear_acceleration.y",
            &imu_data_.linear_acceleration_y);
        interfaces.emplace_back(
            imu_data_.name,
            "linear_acceleration.z",
            &imu_data_.linear_acceleration_z);
    }
    // What was last sent to the loco service, so a controller can see its clamp take effect.
    interfaces.emplace_back(loco_gpio_name_, "linear_x", &loco_state_.linear_x);
    interfaces.emplace_back(loco_gpio_name_, "linear_y", &loco_state_.linear_y);
    interfaces.emplace_back(loco_gpio_name_, "angular_z", &loco_state_.angular_z);
    return interfaces;
}

std::vector<hardware_interface::CommandInterface> G1ArmSdkSystem::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (auto& j : joints_)
    {
        if (!j.commanded)
        {
            continue;
        }
        interfaces.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.command.position);
        interfaces.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &j.command.velocity);
        interfaces.emplace_back(j.name, hardware_interface::HW_IF_EFFORT, &j.command.effort);
        interfaces.emplace_back(j.name, std::string(kHwIfKp), &j.command.kp);
        interfaces.emplace_back(j.name, std::string(kHwIfKd), &j.command.kd);
    }
    interfaces.emplace_back(loco_gpio_name_, "linear_x", &loco_command_.linear_x);
    interfaces.emplace_back(loco_gpio_name_, "linear_y", &loco_command_.linear_y);
    interfaces.emplace_back(loco_gpio_name_, "angular_z", &loco_command_.angular_z);
    return interfaces;
}

bool G1ArmSdkSystem::initializeSdk()
{
    try
    {
        unitree::robot::ChannelFactory::Instance()->Init(domain_id_, network_interface_);

        lowstate_subscriber_ =
            std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(
                "rt/lowstate");
        lowstate_subscriber_->InitChannel(
            [this](const void* message) { lowStateCallback(message); },
            1);

        arm_sdk_publisher_ =
            std::make_shared<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>(
                "rt/arm_sdk");
        arm_sdk_publisher_->InitChannel();

        const auto deadline = std::chrono::steady_clock::now() + kFirstStateTimeout;
        while (!first_state_received_.load())
        {
            if (std::chrono::steady_clock::now() > deadline)
            {
                RCLCPP_ERROR(logger_, "no rt/lowstate within 10 s -- is the robot or sim up?");
                shutdownSdk();
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return true;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(logger_, "SDK init failed: %s", e.what());
        shutdownSdk();
        return false;
    }
}

void G1ArmSdkSystem::shutdownSdk()
{
    lowstate_subscriber_.reset();
    arm_sdk_publisher_.reset();
    first_state_received_ = false;
}

void G1ArmSdkSystem::lowStateCallback(const void* message)
{
    StampedLowState sample;
    sample.state   = *static_cast<const unitree_hg::msg::dds_::LowState_*>(message);
    sample.arrival = std::chrono::steady_clock::now();
    lowstate_buffer_.writeFromNonRT(sample);
    first_state_received_ = true;
}

void G1ArmSdkSystem::locoWorker()
{
    unitree::robot::g1::LocoClient client;
    client.SetTimeout(kLocoRpcTimeoutS);
    client.Init();

    if (start_locomotion_)
    {
        // FSM 500, walking. Only from a robot already standing on its feet; see the README.
        const int32_t ret = client.Start();
        if (ret != 0)
        {
            RCLCPP_ERROR(logger_, "loco Start() returned %d; the base will not move", ret);
        }
    }

    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / loco_command_rate_hz_));
    bool moving      = false;
    bool warned_once = false;

    std::unique_lock<std::mutex> lock(loco_mutex_);
    while (loco_running_)
    {
        const Twist twist = loco_pending_;
        lock.unlock();

        // Moving, or the one zero that stops it. Idle zeros are not sent, so the operator's
        // remote still drives the robot while nothing here asks it to move.
        const bool nonzero =
            twist.linear_x != 0.0 || twist.linear_y != 0.0 || twist.angular_z != 0.0;
        if (nonzero || moving)
        {
            const int32_t ret = client.SetVelocity(
                static_cast<float>(twist.linear_x),
                static_cast<float>(twist.linear_y),
                static_cast<float>(twist.angular_z),
                static_cast<float>(loco_command_duration_s_));
            if (ret != 0 && !warned_once)
            {
                RCLCPP_WARN(
                    logger_,
                    "loco SetVelocity returned %d -- is the robot in its walking mode?",
                    ret);
                warned_once = true;
            }
            moving = nonzero;
        }

        lock.lock();
        loco_wake_.wait_for(lock, period, [this] { return !loco_running_; });
    }
    lock.unlock();

    if (moving)
    {
        client.StopMove();
    }
}

void G1ArmSdkSystem::stopLocoWorker()
{
    {
        std::lock_guard<std::mutex> lock(loco_mutex_);
        loco_running_ = false;
        loco_pending_ = Twist{};
    }
    loco_wake_.notify_all();
    if (loco_thread_.joinable())
    {
        loco_thread_.join();
    }
}

hardware_interface::CallbackReturn
G1ArmSdkSystem::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    if (!initializeSdk())
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    const StampedLowState* sample = lowstate_buffer_.readFromRT();
    for (auto& j : joints_)
    {
        const auto& motor = sample->state.motor_state()[static_cast<std::size_t>(j.sdk_index)];
        j.position_state  = motor.q();
        j.velocity_state  = motor.dq();
        j.effort_state    = motor.tau_est();
        j.command         = JointCommand{ motor.q(), 0.0, 0.0, 0.0, 0.0 };
        j.mode            = JointControlMode::kDisabled;
        j.claims          = InterfaceClaims{};
    }
    loco_command_ = Twist{};
    loco_state_   = Twist{};
    weight_       = 0.0;

    {
        std::lock_guard<std::mutex> lock(loco_mutex_);
        loco_running_ = true;
        loco_pending_ = Twist{};
    }
    loco_thread_ = std::thread([this] { locoWorker(); });

    active_ = true;
    RCLCPP_INFO(
        logger_,
        "rt/arm_sdk and the loco service are ours; the onboard controller keeps the legs");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
G1ArmSdkSystem::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    releaseSynchronously();
    shutdownSdk();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
G1ArmSdkSystem::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
{
    releaseSynchronously();
    shutdownSdk();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
G1ArmSdkSystem::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/)
{
    releaseSynchronously();
    shutdownSdk();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type G1ArmSdkSystem::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces)
{
    const auto listed =
        [](const std::vector<std::string>& list, const std::string& joint, std::string_view type) {
            const std::string full = joint + "/" + std::string(type);
            return std::find(list.begin(), list.end(), full) != list.end();
        };

    const auto apply = [&](const std::vector<std::string>& list, bool held) {
        for (const auto& interface : list)
        {
            const auto [joint_name, type] = splitInterface(interface);
            if (joint_name == loco_gpio_name_)
            {
                if (!held)
                {
                    // A controller letting go of the base must not leave its last twist behind.
                    loco_command_ = Twist{};
                }
                continue;
            }
            const auto it = joint_index_.find(joint_name);
            if (it == joint_index_.end())
            {
                continue;
            }
            auto& j = joints_[it->second];

            if (type == hardware_interface::HW_IF_POSITION)
            {
                j.claims.position = held;
                if (held)
                {
                    j.command.position = j.position_state;
                }
            }
            else if (type == hardware_interface::HW_IF_VELOCITY)
            {
                j.claims.velocity = held;
            }
            else if (type == hardware_interface::HW_IF_EFFORT)
            {
                j.claims.effort = held;
            }
            else if (
                (type == kHwIfKp || type == kHwIfKd) && listed(list, joint_name, kHwIfKp) &&
                listed(list, joint_name, kHwIfKd))
            {
                j.claims.impedance = held;
            }
        }
    };

    apply(stop_interfaces, false);
    apply(start_interfaces, true);

    for (auto& j : joints_)
    {
        j.mode = resolveJointMode(j.claims);
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
G1ArmSdkSystem::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    const StampedLowState* sample = lowstate_buffer_.readFromRT();
    for (auto& j : joints_)
    {
        const auto& motor = sample->state.motor_state()[static_cast<std::size_t>(j.sdk_index)];
        j.position_state  = motor.q();
        j.velocity_state  = motor.dq();
        j.effort_state    = motor.tau_est();
    }

    if (has_imu_)
    {
        const auto& imu         = sample->state.imu_state();
        imu_data_.orientation_w = imu.quaternion()[0];
        imu_data_.orientation_x = imu.quaternion()[1];
        imu_data_.orientation_y = imu.quaternion()[2];
        imu_data_.orientation_z = imu.quaternion()[3];

        imu_data_.angular_velocity_x = imu.gyroscope()[0];
        imu_data_.angular_velocity_y = imu.gyroscope()[1];
        imu_data_.angular_velocity_z = imu.gyroscope()[2];

        imu_data_.linear_acceleration_x = imu.accelerometer()[0];
        imu_data_.linear_acceleration_y = imu.accelerometer()[1];
        imu_data_.linear_acceleration_z = imu.accelerometer()[2];
    }

    const auto age = std::chrono::steady_clock::now() - sample->arrival;
    if (active_.load() && age > std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                    std::chrono::duration<double>(lowstate_timeout_s_)))
    {
        RCLCPP_ERROR(logger_, "rt/lowstate went stale while active");
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

bool G1ArmSdkSystem::publishArmSdk(double weight)
{
    arm_cmd_.motor_cmd()[kArmSdkWeightIndex].q() = static_cast<float>(weight);
    computeLowCmdCrc(arm_cmd_);
    return arm_sdk_publisher_->Write(arm_cmd_);
}

hardware_interface::return_type
G1ArmSdkSystem::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
    in_write_.store(true);
    const InWriteGuard guard{ &in_write_ };
    if (!active_.load() || !arm_sdk_publisher_)
    {
        return hardware_interface::return_type::OK;
    }

    bool all_powered = true;
    for (const auto& j : joints_)
    {
        if (!j.commanded)
        {
            continue;
        }
        all_powered = all_powered && j.mode != JointControlMode::kDisabled;
        fillMotorCmd(
            arm_cmd_.motor_cmd()[static_cast<std::size_t>(j.sdk_index)],
            j.mode,
            j.command,
            j.position_only_gains,
            j.position_state);
    }
    const double step = period.seconds() / engage_ramp_s_;
    weight_           = nextArmSdkWeight(weight_, all_powered, step);

    if (!publishArmSdk(weight_))
    {
        RCLCPP_ERROR(logger_, "rt/arm_sdk write refused -- the arms are no longer commanded");
        return hardware_interface::return_type::ERROR;
    }

    loco_state_ = Twist{ sanitizeLocoCommand(loco_command_.linear_x, max_linear_speed_),
                         sanitizeLocoCommand(loco_command_.linear_y, max_linear_speed_),
                         sanitizeLocoCommand(loco_command_.angular_z, max_angular_speed_) };
    // try_lock: the worker holds the mutex only to copy, and a tick that misses it sends the
    // same twist one period later.
    std::unique_lock<std::mutex> lock(loco_mutex_, std::try_to_lock);
    if (lock.owns_lock())
    {
        loco_pending_ = loco_state_;
    }
    return hardware_interface::return_type::OK;
}

void G1ArmSdkSystem::releaseSynchronously()
{
    if (!active_.exchange(false))
    {
        return;
    }
    while (in_write_.load())
    {
        std::this_thread::yield();
    }

    // The base first: a robot walking while its arms hand back is the one thing worse than either.
    stopLocoWorker();

    if (arm_sdk_publisher_)
    {
        // The arms keep their last command while the weight fades, so the onboard pose takes
        // them over smoothly rather than from limp.
        const double step = std::chrono::duration<double>(kReleaseTickPeriod).count() /
                            std::max(release_ramp_s_, 1e-3);
        while (weight_ > 0.0)
        {
            weight_ = nextArmSdkWeight(weight_, false, step);
            publishArmSdk(weight_);
            std::this_thread::sleep_for(kReleaseTickPeriod);
        }
        publishArmSdk(0.0);
    }
}

}  // namespace g1_hardware_interface

PLUGINLIB_EXPORT_CLASS(g1_hardware_interface::G1ArmSdkSystem, hardware_interface::SystemInterface)
