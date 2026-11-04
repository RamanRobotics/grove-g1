/**
 * @file linkerbot_o6_system.cpp
 * @brief ros2_control SystemInterface for one LinkerBot O6 hand over SocketCAN.
 */

#include "g1_hand_interface/linkerbot_o6_system.hpp"

#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <pluginlib/class_list_macros.hpp>
#include <system_error>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace g1_hand_interface
{

namespace
{
constexpr std::chrono::seconds kFirstStateTimeout{ 2 };

/// errno as text. std::strerror shares one buffer between threads, and the I/O thread logs too.
std::string errnoText() { return std::generic_category().message(errno); }

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

/// 0-255, as the speed and torque frames carry them.
bool parseByte(
    const std::unordered_map<std::string, std::string>& params, const std::string& key,
    std::uint8_t& out)
{
    double value = 0.0;
    if (!parseDouble(params, key, value) || value < 0.0 || value > 255.0)
    {
        return false;
    }
    out = static_cast<std::uint8_t>(std::lround(value));
    return true;
}
}  // namespace

LinkerBotO6System::~LinkerBotO6System()
{
    stopIo();
    closeSocket();
}

hardware_interface::CallbackReturn
LinkerBotO6System::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
{
    if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    const auto& info = get_hardware_info();
    const auto& hw   = info.hardware_parameters;

    const auto side_it = hw.find("side");
    if (side_it == hw.end() || (side_it->second != "left" && side_it->second != "right"))
    {
        RCLCPP_ERROR(logger_, "<hardware> needs side: left or right");
        return hardware_interface::CallbackReturn::ERROR;
    }
    const std::string& side   = side_it->second;
    const std::string  prefix = side == "left" ? "lh_" : "rh_";
    logger_                   = rclcpp::get_logger("linkerbot_o6_system_" + side);

    if (const auto it = hw.find("can_interface"); it != hw.end() && !it->second.empty())
    {
        can_interface_ = it->second;
    }
    else
    {
        RCLCPP_ERROR(logger_, "<hardware> needs a can_interface, e.g. can0");
        return hardware_interface::CallbackReturn::ERROR;
    }

    double can_id = side == "left" ? o6::kLeftHandId : o6::kRightHandId;
    parseDouble(hw, "can_id", can_id);
    can_id_ = static_cast<canid_t>(can_id);

    double state_timeout_ms = 0.0;
    if (!parseDouble(hw, "command_rate_hz", command_rate_hz_) ||
        !parseDouble(hw, "state_timeout_ms", state_timeout_ms) ||
        !parseDouble(hw, "max_joint_velocity_rad_s", max_joint_velocity_rad_s_))
    {
        RCLCPP_ERROR(logger_, "<hardware> is missing or has an unparseable <param>");
        return hardware_interface::CallbackReturn::ERROR;
    }
    state_timeout_s_ = state_timeout_ms / 1000.0;
    if (command_rate_hz_ <= 0.0 || state_timeout_s_ <= 0.0 || max_joint_velocity_rad_s_ <= 0.0)
    {
        RCLCPP_ERROR(
            logger_,
            "command_rate_hz, state_timeout_ms and max_joint_velocity_rad_s must be > 0");
        return hardware_interface::CallbackReturn::ERROR;
    }

    std::uint8_t speed  = 0;
    std::uint8_t torque = 0;
    if (!parseByte(hw, "speed", speed) || !parseByte(hw, "torque", torque))
    {
        RCLCPP_ERROR(logger_, "speed and torque must be 0-255");
        return hardware_interface::CallbackReturn::ERROR;
    }
    speed_.fill(speed);
    torque_.fill(torque);

    // The actuated joints in wire order first, then the mimics. Positional, so a reordered xacro
    // would close the wrong fingers.
    if (info.joints.size() < o6::kNumActuated)
    {
        RCLCPP_ERROR(logger_, "expected the %zu actuated joints", o6::kNumActuated);
        return hardware_interface::CallbackReturn::ERROR;
    }
    for (std::size_t i = 0; i < o6::kNumActuated; ++i)
    {
        const auto&       joint    = info.joints[i];
        const std::string expected = prefix + o6::kJointSuffixes[i];
        if (joint.name != expected)
        {
            RCLCPP_ERROR(
                logger_,
                "joint %zu is '%s', the wire order needs '%s'",
                i,
                joint.name.c_str(),
                expected.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        auto& a = actuated_[i];
        a.name  = joint.name;
        if (!parseDouble(joint.parameters, "min", a.min) ||
            !parseDouble(joint.parameters, "max", a.max) || !(a.max > a.min))
        {
            RCLCPP_ERROR(logger_, "joint '%s' needs min < max", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    mimics_.clear();
    for (std::size_t i = o6::kNumActuated; i < info.joints.size(); ++i)
    {
        const auto& joint = info.joints[i];
        Mimic       m;
        m.name                = joint.name;
        const auto  parent_it = joint.parameters.find("mimic_parent");
        auto* const parent = std::find_if(actuated_.begin(), actuated_.end(), [&](const auto& a) {
            return parent_it != joint.parameters.end() && a.name == parent_it->second;
        });
        if (parent == actuated_.end() ||
            !parseDouble(joint.parameters, "mimic_multiplier", m.multiplier))
        {
            RCLCPP_ERROR(
                logger_,
                "mimic joint '%s' needs mimic_parent (an actuated joint) and mimic_multiplier",
                joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        m.parent = static_cast<std::size_t>(parent - actuated_.begin());
        mimics_.push_back(m);
    }

    RCLCPP_INFO(
        logger_,
        "LinkerBot O6 %s hand on %s id 0x%02X, %zu mimic joints",
        side.c_str(),
        can_interface_.c_str(),
        can_id_,
        mimics_.size());
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LinkerBotO6System::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> interfaces;
    interfaces.reserve((actuated_.size() * 2) + mimics_.size());
    for (auto& a : actuated_)
    {
        interfaces.emplace_back(a.name, hardware_interface::HW_IF_POSITION, &a.position_state);
        interfaces.emplace_back(a.name, hardware_interface::HW_IF_VELOCITY, &a.velocity_state);
    }
    for (auto& m : mimics_)
    {
        interfaces.emplace_back(m.name, hardware_interface::HW_IF_POSITION, &m.position);
    }
    return interfaces;
}

std::vector<hardware_interface::CommandInterface> LinkerBotO6System::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.reserve(actuated_.size());
    for (auto& a : actuated_)
    {
        interfaces.emplace_back(a.name, hardware_interface::HW_IF_POSITION, &a.command);
    }
    return interfaces;
}

bool LinkerBotO6System::openSocket()
{
    socket_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (socket_ < 0)
    {
        RCLCPP_ERROR(logger_, "CAN socket: %s", errnoText().c_str());
        return false;
    }

    ifreq request{};
    std::strncpy(request.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    if (::ioctl(socket_, SIOCGIFINDEX, &request) < 0)
    {
        RCLCPP_ERROR(
            logger_,
            "no CAN interface '%s' (%s). On the robot: ip link set %s up type can bitrate "
            "1000000. In simulation: scripts/setup-vcan.sh",
            can_interface_.c_str(),
            errnoText().c_str(),
            can_interface_.c_str());
        closeSocket();
        return false;
    }

    // Only this hand's id: the other hand usually shares the bus.
    can_filter filter{};
    filter.can_id   = can_id_;
    filter.can_mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
    ::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));

    sockaddr_can address{};
    address.can_family  = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (::bind(socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
    {
        RCLCPP_ERROR(logger_, "bind %s: %s", can_interface_.c_str(), errnoText().c_str());
        closeSocket();
        return false;
    }
    return true;
}

void LinkerBotO6System::closeSocket()
{
    if (socket_ >= 0)
    {
        ::close(socket_);
        socket_ = -1;
    }
}

bool LinkerBotO6System::sendFrame(const can_frame& frame) const
{
    return ::write(socket_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}

void LinkerBotO6System::drainReplies()
{
    can_frame frame{};
    while (::read(socket_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame)))
    {
        if (const auto bytes = o6::parsePositionReply(frame, can_id_))
        {
            StampedState sample;
            sample.bytes   = *bytes;
            sample.arrival = std::chrono::steady_clock::now();
            sample.valid   = true;
            state_buffer_.writeFromNonRT(sample);
            first_state_received_ = true;
        }
    }
}

void LinkerBotO6System::ioLoop()
{
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / command_rate_hz_));
    auto next        = std::chrono::steady_clock::now();
    bool warned_once = false;

    while (io_running_.load())
    {
        drainReplies();

        Target target;
        {
            std::lock_guard<std::mutex> lock(target_mutex_);
            target = target_;
        }
        // The command, then a state request every tick: the vendor SDK reads the hand only by
        // the empty 0x01 request (get_current_status), so a reply to a command is not counted on.
        bool sent = true;
        if (target.driven)
        {
            sent = sendFrame(o6::makeFrame(can_id_, o6::kFramePosition, &target.bytes));
        }
        sent = sendFrame(o6::makeFrame(can_id_, o6::kFramePosition, nullptr)) && sent;
        // On a real bus a hand that is off leaves frames unacknowledged until the queue is full
        // (ENOBUFS); the state timeout in read() is what errors the component.
        if (!sent && !warned_once)
        {
            RCLCPP_WARN(
                logger_,
                "CAN write on %s failed: %s",
                can_interface_.c_str(),
                errnoText().c_str());
            warned_once = true;
        }
        else if (sent)
        {
            warned_once = false;
        }

        next += period;
        std::this_thread::sleep_until(next);
    }
}

void LinkerBotO6System::stopIo()
{
    io_running_ = false;
    if (io_thread_.joinable())
    {
        io_thread_.join();
    }
}

hardware_interface::CallbackReturn
LinkerBotO6System::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    if (!openSocket())
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    // Speed and torque persist in the hand, so once per activation; sent twice, as the SDK does,
    // since nothing acknowledges them.
    for (int i = 0; i < 2; ++i)
    {
        sendFrame(o6::makeFrame(can_id_, o6::kFrameSpeed, &speed_));
        sendFrame(o6::makeFrame(can_id_, o6::kFrameTorque, &torque_));
    }

    first_state_received_ = false;
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        target_ = Target{};
    }
    io_running_ = true;
    io_thread_  = std::thread([this] { ioLoop(); });

    const auto deadline = std::chrono::steady_clock::now() + kFirstStateTimeout;
    while (!first_state_received_.load())
    {
        if (std::chrono::steady_clock::now() > deadline)
        {
            RCLCPP_ERROR(
                logger_,
                "no reply from id 0x%02X on %s within 2 s -- is the hand powered, or the "
                "simulator up with this hand?",
                can_id_,
                can_interface_.c_str());
            stopIo();
            closeSocket();
            return hardware_interface::CallbackReturn::ERROR;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Seed at the measurement, so the first write holds the hand where it is.
    const StampedState* sample = state_buffer_.readFromRT();
    for (std::size_t i = 0; i < o6::kNumActuated; ++i)
    {
        auto& a          = actuated_[i];
        a.position_state = o6::wireToRad(sample->bytes[i], a.min, a.max);
        a.velocity_state = 0.0;
        a.command        = a.position_state;
        a.target         = a.position_state;
    }
    active_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
LinkerBotO6System::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    // The O6 holds its last position on its own; stopping the frames freezes it there.
    active_ = false;
    stopIo();
    closeSocket();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
LinkerBotO6System::on_error(const rclcpp_lifecycle::State& previous_state)
{
    return on_deactivate(previous_state);
}

hardware_interface::return_type
LinkerBotO6System::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
    const StampedState* sample = state_buffer_.readFromRT();
    if (!sample->valid)
    {
        return hardware_interface::return_type::OK;
    }

    const double dt = period.seconds();
    for (std::size_t i = 0; i < o6::kNumActuated; ++i)
    {
        auto&        a        = actuated_[i];
        const double position = o6::wireToRad(sample->bytes[i], a.min, a.max);
        // The hand reports no velocity; a difference over one tick is what it has.
        a.velocity_state = dt > 0.0 ? (position - a.position_state) / dt : 0.0;
        a.position_state = position;
    }
    for (auto& m : mimics_)
    {
        m.position = m.multiplier * actuated_[m.parent].position_state;
    }

    const auto age = std::chrono::steady_clock::now() - sample->arrival;
    if (active_.load() && age > std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                    std::chrono::duration<double>(state_timeout_s_)))
    {
        RCLCPP_ERROR(logger_, "no O6 state for %.0f ms", state_timeout_s_ * 1000.0);
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
LinkerBotO6System::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
    if (!active_.load())
    {
        return hardware_interface::return_type::OK;
    }

    const double step = max_joint_velocity_rad_s_ * period.seconds();
    Target       target;
    target.driven = true;
    for (std::size_t i = 0; i < o6::kNumActuated; ++i)
    {
        auto& a = actuated_[i];
        // NaN means no controller has written yet: hold the slewed target where it is.
        const double goal =
            std::isfinite(a.command) ? std::clamp(a.command, a.min, a.max) : a.target;
        a.target        = a.target + std::clamp(goal - a.target, -step, step);
        target.bytes[i] = o6::radToWire(a.target, a.min, a.max);
    }
    // A tick that misses the lock is sent one period later, the same target or a nearer one.
    std::unique_lock<std::mutex> lock(target_mutex_, std::try_to_lock);
    if (lock.owns_lock())
    {
        target_ = target;
    }
    return hardware_interface::return_type::OK;
}

}  // namespace g1_hand_interface

PLUGINLIB_EXPORT_CLASS(g1_hand_interface::LinkerBotO6System, hardware_interface::SystemInterface)
