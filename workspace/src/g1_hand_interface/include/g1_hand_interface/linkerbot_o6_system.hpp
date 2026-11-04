#ifndef G1_HAND_INTERFACE__LINKERBOT_O6_SYSTEM_HPP_
#define G1_HAND_INTERFACE__LINKERBOT_O6_SYSTEM_HPP_

/**
 * @file linkerbot_o6_system.hpp
 * @brief ros2_control SystemInterface for one LinkerBot O6 hand over SocketCAN.
 */

#include <linux/can.h>

#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "g1_hand_interface/linkerbot_o6_protocol.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace g1_hand_interface
{

/**
 * @brief One LinkerBot O6: six position-commanded joints, plus the five mimic joints the URDF
 *        couples to them, reported from their parents.
 *
 * The CAN socket is served by a thread of its own at command_rate_hz: each tick sends the latest
 * target once there is one, then an empty 0x01 asking for state, and takes in every reply.
 * read() and write() only touch buffers, so a slow bus never stalls controller_manager's loop.
 *
 * In simulation unitree_mujoco answers the same frames on a virtual CAN interface, so this is not
 * swapped out for sim.
 */
class LinkerBotO6System : public hardware_interface::SystemInterface
{
public:
    ~LinkerBotO6System() override;

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

    hardware_interface::return_type
    read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type
    write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    struct Actuated
    {
        std::string name;
        double      min = 0.0;
        double      max = 0.0;

        double position_state = 0.0;
        double velocity_state = 0.0;
        double command        = 0.0;
        /// The slewed target actually sent; the controller's command is where it heads.
        double target = 0.0;
    };

    struct Mimic
    {
        std::string name;
        std::size_t parent     = 0;
        double      multiplier = 1.0;
        double      position   = 0.0;
    };

    struct StampedState
    {
        o6::Bytes                             bytes{};
        std::chrono::steady_clock::time_point arrival{};
        bool                                  valid = false;
    };

    struct Target
    {
        o6::Bytes bytes{};
        bool      driven = false;
    };

    bool openSocket();
    void closeSocket();
    bool sendFrame(const can_frame& frame) const;
    /// Takes every frame waiting on the socket, keeping this hand's position replies.
    void drainReplies();
    void ioLoop();
    void stopIo();

    rclcpp::Logger logger_{ rclcpp::get_logger("linkerbot_o6_system") };

    std::array<Actuated, o6::kNumActuated> actuated_;
    std::vector<Mimic>                     mimics_;

    std::string can_interface_;
    canid_t     can_id_                   = 0;
    double      command_rate_hz_          = 0.0;
    double      state_timeout_s_          = 0.0;
    double      max_joint_velocity_rad_s_ = 0.0;
    o6::Bytes   speed_{};
    o6::Bytes   torque_{};

    int socket_ = -1;

    realtime_tools::RealtimeBuffer<StampedState> state_buffer_;
    /// Written from the update thread with try_lock, so it never waits on the I/O thread.
    std::mutex        target_mutex_;
    Target            target_;
    std::atomic<bool> io_running_{ false };
    std::atomic<bool> active_{ false };
    std::atomic<bool> first_state_received_{ false };
    std::thread       io_thread_;
};

}  // namespace g1_hand_interface

#endif  // G1_HAND_INTERFACE__LINKERBOT_O6_SYSTEM_HPP_
