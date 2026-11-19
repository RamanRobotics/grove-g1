#ifndef G1_CONTROLLERS__G1_LOCO_CONTROLLER_HPP_
#define G1_CONTROLLERS__G1_LOCO_CONTROLLER_HPP_

/**
 * @file g1_loco_controller.hpp
 * @brief Forwards /cmd_vel to a component's loco twist, for a G1 that walks on its onboard
 *        controller.
 */

#include <atomic>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/bool.hpp"

namespace g1_controllers
{

/**
 * @brief The twist a G1LocoController writes for one /cmd_vel sample: clamped, and zero once the
 *        publisher has gone quiet for `timeout_s` or has never published.
 */
[[nodiscard]] geometry_msgs::msg::Twist locoCommand(
    const geometry_msgs::msg::Twist& command, double age_s, bool ever_received, double timeout_s,
    double max_linear_speed, double max_angular_speed);

/**
 * @brief The 23-DOF G1's stand-in for G1AgileController: the same /cmd_vel contract, timeout and
 *        clamps, written to `<loco_interface>/{linear_x,linear_y,angular_z}` instead of joints.
 *
 * The walking itself is the onboard controller's, reached through G1ArmSdkSystem, so this claims
 * no joint and the legs need no freeze to fall back on.
 */
class G1LocoController : public controller_interface::ControllerInterface
{
public:
    controller_interface::CallbackReturn on_init() override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type
    update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    void writeTwist(double linear_x, double linear_y, double angular_z);

    std::string loco_interface_;
    std::string cmd_vel_topic_;
    double      cmd_vel_timeout_s_ = 0.5;
    double      max_linear_speed_  = 0.0;
    double      max_angular_speed_ = 0.0;

    realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist>  cmd_vel_buffer_;
    std::atomic<double>                                        last_cmd_vel_seconds_{ 0.0 };
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;

    /// `~/walking`, transient local: true while active, as G1AgileController's `~/inferring`.
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr walking_publisher_;
};

}  // namespace g1_controllers

#endif  // G1_CONTROLLERS__G1_LOCO_CONTROLLER_HPP_
