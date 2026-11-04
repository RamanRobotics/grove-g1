#ifndef G1_HARDWARE_INTERFACE__G1_ARMSDK_SYSTEM_HPP_
#define G1_HARDWARE_INTERFACE__G1_ARMSDK_SYSTEM_HPP_

/**
 * @file g1_armsdk_system.hpp
 * @brief ros2_control SystemInterface for a G1 that keeps Unitree's onboard locomotion: the upper
 *        body over rt/arm_sdk, walking over the loco service.
 */

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unordered_map>
#include <vector>

#include "g1_hardware_interface/g1_lowcmd_system.hpp"
#include "g1_hardware_interface/lowcmd_assembly.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace g1_hardware_interface
{

/// rt/arm_sdk reads its blend weight from the q of this otherwise unused motor slot.
inline constexpr std::size_t kArmSdkWeightIndex = 29;

/// The loco GPIO's interfaces: a body-frame twist, m/s and rad/s.
inline constexpr std::array<std::string_view, 3> kLocoInterfaces = { "linear_x",
                                                                     "linear_y",
                                                                     "angular_z" };

/**
 * @brief Blend weight for the next tick: toward 1 while every upper-body joint is powered, toward
 *        0 otherwise, at most `step` per tick.
 *
 * At weight 1 an unclaimed joint is limp, so the onboard controller gets the arms back whenever a
 * joint loses its controller rather than the arm dropping.
 */
[[nodiscard]] double nextArmSdkWeight(double weight, bool all_powered, double step) noexcept;

/**
 * @brief Clamps a commanded twist component; a non-finite one is zero.
 */
[[nodiscard]] double sanitizeLocoCommand(double value, double limit) noexcept;

/**
 * @brief ros2_control System for a G1 whose legs stay under Unitree's onboard controller.
 *
 * The legs are state only. The waist and arms go out on rt/arm_sdk, which blends them over the
 * onboard pose by a weight this component ramps. The loco GPIO's twist goes to the onboard
 * locomotion through unitree::robot::g1::LocoClient. Nothing here writes rt/lowcmd or releases
 * the onboard motion mode, so the robot keeps balancing itself throughout.
 *
 * In simulation unitree_mujoco's loco emulator answers the same channel and service, so this is
 * not swapped out for sim.
 */
class G1ArmSdkSystem : public hardware_interface::SystemInterface
{
public:
    ~G1ArmSdkSystem() override;

    hardware_interface::CallbackReturn
    on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;

    std::vector<hardware_interface::StateInterface>   export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn
    on_error(const rclcpp_lifecycle::State& previous_state) override;
    /// Runs the release too: controller_manager does not guarantee on_deactivate before exit.
    hardware_interface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    hardware_interface::return_type
    read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type
    write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    struct StampedLowState
    {
        unitree_hg::msg::dds_::LowState_      state{};
        std::chrono::steady_clock::time_point arrival{};
    };

    /// One body joint. Legs carry no command interfaces and never reach the wire.
    struct Joint
    {
        std::string name;
        int         sdk_index = -1;
        bool        commanded = false;

        double position_state = 0.0;
        double velocity_state = 0.0;
        double effort_state   = 0.0;

        JointCommand      command;
        InterfaceClaims   claims;
        PositionOnlyGains position_only_gains;
        JointControlMode  mode = JointControlMode::kDisabled;
    };

    struct Twist
    {
        double linear_x  = 0.0;
        double linear_y  = 0.0;
        double angular_z = 0.0;
    };

    bool initializeSdk();
    void shutdownSdk();
    void lowStateCallback(const void* message);

    /// Packs and writes one rt/arm_sdk frame at `weight`.
    bool publishArmSdk(double weight);

    /// Ramps the weight to zero over release_ramp_s, then stops the base. Blocks; returns at once
    /// if not active.
    void releaseSynchronously();

    /// Sends the latest twist to the loco service. Off the update thread: each call is an RPC.
    void locoWorker();
    void stopLocoWorker();

    rclcpp::Logger logger_{ rclcpp::get_logger("g1_armsdk_system") };

    std::vector<Joint>                           joints_;
    std::unordered_map<std::string, std::size_t> joint_index_;
    ImuData                                      imu_data_;
    bool                                         has_imu_ = false;

    std::string loco_gpio_name_;
    Twist       loco_command_;
    Twist       loco_state_;

    std::string network_interface_;
    int         domain_id_               = 0;
    double      lowstate_timeout_s_      = 0.0;
    double      engage_ramp_s_           = 0.0;
    double      release_ramp_s_          = 0.0;
    double      loco_command_rate_hz_    = 0.0;
    double      loco_command_duration_s_ = 0.0;
    double      max_linear_speed_        = 0.0;
    double      max_angular_speed_       = 0.0;
    bool        start_locomotion_        = false;

    /// The rt/arm_sdk blend: 0 is the onboard pose, 1 is ours.
    double weight_ = 0.0;

    unitree_hg::msg::dds_::LowCmd_ arm_cmd_{};

    realtime_tools::RealtimeBuffer<StampedLowState> lowstate_buffer_;
    std::atomic<bool>                               first_state_received_{ false };
    std::atomic<bool>                               active_{ false };
    std::atomic<bool>                               in_write_{ false };

    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> lowstate_subscriber_;
    unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_>    arm_sdk_publisher_;

    std::thread             loco_thread_;
    std::mutex              loco_mutex_;
    std::condition_variable loco_wake_;
    Twist                   loco_pending_;
    bool                    loco_running_ = false;
};

}  // namespace g1_hardware_interface

#endif  // G1_HARDWARE_INTERFACE__G1_ARMSDK_SYSTEM_HPP_
