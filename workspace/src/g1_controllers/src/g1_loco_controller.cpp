/**
 * @file g1_loco_controller.cpp
 * @brief Forwards /cmd_vel to a component's loco twist.
 */

#include "g1_controllers/g1_loco_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>

namespace g1_controllers
{

namespace
{
/// Command interface order, matched by index in update().
constexpr std::array<const char*, 3> kTwistInterfaces = { "linear_x", "linear_y", "angular_z" };

double clampMagnitude(double value, double limit)
{
    if (!std::isfinite(value))
    {
        return 0.0;
    }
    return limit > 0.0 ? std::clamp(value, -limit, limit) : value;
}
}  // namespace

geometry_msgs::msg::Twist locoCommand(
    const geometry_msgs::msg::Twist& command, double age_s, bool ever_received, double timeout_s,
    double max_linear_speed, double max_angular_speed)
{
    geometry_msgs::msg::Twist out;
    if (!ever_received || (timeout_s > 0.0 && age_s > timeout_s))
    {
        return out;
    }
    out.linear.x  = clampMagnitude(command.linear.x, max_linear_speed);
    out.linear.y  = clampMagnitude(command.linear.y, max_linear_speed);
    out.angular.z = clampMagnitude(command.angular.z, max_angular_speed);
    return out;
}

controller_interface::CallbackReturn G1LocoController::on_init()
{
    auto_declare<std::string>("loco_interface", "loco");
    auto_declare<std::string>("cmd_vel_topic", "/cmd_vel");
    auto_declare<double>("cmd_vel_timeout", 0.5);
    auto_declare<double>("max_linear_speed", 0.0);
    auto_declare<double>("max_angular_speed", 0.0);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
G1LocoController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
    loco_interface_    = get_node()->get_parameter("loco_interface").as_string();
    cmd_vel_topic_     = get_node()->get_parameter("cmd_vel_topic").as_string();
    cmd_vel_timeout_s_ = get_node()->get_parameter("cmd_vel_timeout").as_double();
    max_linear_speed_  = get_node()->get_parameter("max_linear_speed").as_double();
    max_angular_speed_ = get_node()->get_parameter("max_angular_speed").as_double();

    if (loco_interface_.empty())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "loco_interface must name the component's <gpio>");
        return controller_interface::CallbackReturn::ERROR;
    }

    cmd_vel_buffer_.writeFromNonRT(geometry_msgs::msg::Twist{});
    // As G1AgileController: newest only, never latched, reliable like Nav2 and g1_base_approach.
    cmd_vel_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
        [this](const geometry_msgs::msg::Twist::ConstSharedPtr& message) {
            cmd_vel_buffer_.writeFromNonRT(*message);
            last_cmd_vel_seconds_.store(get_node()->now().seconds());
        });

    walking_publisher_ = get_node()->create_publisher<std_msgs::msg::Bool>(
        "~/walking",
        rclcpp::QoS(1).transient_local());
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
G1LocoController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const char* name : kTwistInterfaces)
    {
        config.names.push_back(loco_interface_ + "/" + name);
    }
    return config;
}

controller_interface::InterfaceConfiguration G1LocoController::state_interface_configuration() const
{
    return { controller_interface::interface_configuration_type::NONE, {} };
}

void G1LocoController::writeTwist(double linear_x, double linear_y, double angular_z)
{
    const std::array<double, 3> values = { linear_x, linear_y, angular_z };
    for (std::size_t i = 0; i < command_interfaces_.size() && i < values.size(); ++i)
    {
        (void)command_interfaces_[i].set_value(values[i]);
    }
}

controller_interface::CallbackReturn
G1LocoController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    if (command_interfaces_.size() != kTwistInterfaces.size())
    {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "expected the %zu %s/* command interfaces, got %zu",
            kTwistInterfaces.size(),
            loco_interface_.c_str(),
            command_interfaces_.size());
        return controller_interface::CallbackReturn::ERROR;
    }
    // A velocity from before a deactivation must not replay.
    cmd_vel_buffer_.writeFromNonRT(geometry_msgs::msg::Twist{});
    last_cmd_vel_seconds_.store(0.0);
    writeTwist(0.0, 0.0, 0.0);

    std_msgs::msg::Bool message;
    message.data = true;
    walking_publisher_->publish(message);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
G1LocoController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    writeTwist(0.0, 0.0, 0.0);
    std_msgs::msg::Bool message;
    message.data = false;
    walking_publisher_->publish(message);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
G1LocoController::update(const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
    const double last  = last_cmd_vel_seconds_.load();
    const auto   twist = locoCommand(
        *cmd_vel_buffer_.readFromRT(),
        time.seconds() - last,
        last > 0.0,
        cmd_vel_timeout_s_,
        max_linear_speed_,
        max_angular_speed_);
    writeTwist(twist.linear.x, twist.linear.y, twist.angular.z);
    return controller_interface::return_type::OK;
}

}  // namespace g1_controllers

PLUGINLIB_EXPORT_CLASS(g1_controllers::G1LocoController, controller_interface::ControllerInterface)
