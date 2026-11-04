#ifndef G1_MANIPULATION__G1_ARM_MOTION_SERVER_NODE_HPP_
#define G1_MANIPULATION__G1_ARM_MOTION_SERVER_NODE_HPP_

/**
 * @file g1_arm_motion_server_node.hpp
 * @brief Arm and hand motion as actions over MoveIt, for any G1 and any hand.
 */

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <map>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <vector>

#include "g1_msgs/action/move_hand.hpp"
#include "g1_msgs/action/set_arm_posture.hpp"
#include "g1_msgs/action/set_grip.hpp"
#include "g1_msgs/srv/get_postures.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace g1_manipulation
{

/**
 * @brief Every hand joint `closure` of the way from its `open` value to its `closed` one.
 *
 * @return Nothing when the two postures name different joints or closure is outside [0, 1].
 */
[[nodiscard]] std::optional<std::map<std::string, double>> gripTarget(
    const std::map<std::string, double>& open, const std::map<std::string, double>& closed,
    double closure);

/**
 * @brief Where a pointing hand goes: `reach_m` from the shoulder along the line to the target,
 *        or the target itself when it is nearer.
 */
[[nodiscard]] geometry_msgs::msg::Point pointAlong(
    const geometry_msgs::msg::Point& shoulder, const geometry_msgs::msg::Point& target,
    double reach_m);

/**
 * @brief SetArmPosture, MoveHand and SetGrip over MoveIt, plus GetPostures.
 *
 * Robot-agnostic: it drives whichever of left_arm, right_arm, both_arms, left_hand and right_hand
 * the running SRDF defines, so the 23-DOF G1 with any hand and the 29-DOF G1 are served alike.
 * Each arm reaches with its group's end-effector link. It takes no control authority: the arm and
 * hands must be acquired (activate_arm) before a goal executes. One goal at a time across all
 * three actions.
 *
 * g1_manipulation_server keeps the 29-DOF Dex3 pick and place; this node is the motion only.
 */
class G1ArmMotionServer : public rclcpp::Node
{
public:
    using SetArmPosture = g1_msgs::action::SetArmPosture;
    using MoveHand      = g1_msgs::action::MoveHand;
    using SetGrip       = g1_msgs::action::SetGrip;
    using GetPostures   = g1_msgs::srv::GetPostures;
    using MoveGroup     = moveit::planning_interface::MoveGroupInterface;

    explicit G1ArmMotionServer(const rclcpp::NodeOptions& options);

    /**
     * @brief Builds the move groups and opens the servers. Blocks until the robot model and its
     *        state arrive, so the node must already be spinning.
     */
    void initialize();

private:
    template <typename Action>
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

    template <typename Action>
    typename rclcpp_action::Server<Action>::SharedPtr serve(
        const std::string& name,
        void (G1ArmMotionServer::*execute)(const std::shared_ptr<GoalHandle<Action>>&));

    void executeSetArmPosture(const std::shared_ptr<GoalHandle<SetArmPosture>>& handle);
    void executeMoveHand(const std::shared_ptr<GoalHandle<MoveHand>>& handle);
    void executeSetGrip(const std::shared_ptr<GoalHandle<SetGrip>>& handle);
    void getPostures(
        const std::shared_ptr<GetPostures::Request>&  request,
        const std::shared_ptr<GetPostures::Response>& response);

    /// Plans and executes the group's target at `speed` of velocity_scaling. Empty on success,
    /// otherwise why not.
    std::string planAndExecute(MoveGroup& group, double speed, const std::string& what) const;

    MoveGroup*                               group(const std::string& name);
    std::optional<geometry_msgs::msg::Point> linkPosition(const std::string& link);

    double      velocity_scaling_  = 0.3;
    double      planning_time_s_   = 5.0;
    int         planning_attempts_ = 3;
    double      max_reach_m_       = 0.55;
    double      point_reach_m_     = 0.40;
    double      ik_timeout_s_      = 0.05;
    int         ik_orientations_   = 16;
    std::string left_shoulder_{ "left_shoulder_pitch_link" };
    std::string right_shoulder_{ "right_shoulder_pitch_link" };

    std::map<std::string, std::unique_ptr<MoveGroup>> groups_;
    std::string                                       planning_frame_;
    std::atomic<bool>                                 busy_{ false };

    tf2_ros::Buffer            tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp_action::Server<SetArmPosture>::SharedPtr posture_server_;
    rclcpp_action::Server<MoveHand>::SharedPtr      move_hand_server_;
    rclcpp_action::Server<SetGrip>::SharedPtr       grip_server_;
    rclcpp::Service<GetPostures>::SharedPtr         postures_service_;
};

}  // namespace g1_manipulation

#endif  // G1_MANIPULATION__G1_ARM_MOTION_SERVER_NODE_HPP_
