/**
 * @file g1_arm_motion_server_node.cpp
 * @brief Arm and hand motion as actions over MoveIt, for any G1 and any hand.
 */

#include "g1_manipulation/g1_arm_motion_server_node.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <format>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <numbers>
#include <random>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

namespace g1_manipulation
{

namespace
{
/// The groups this node may drive; whichever the SRDF defines are built.
constexpr std::array<const char*, 5> kGroups = { "left_arm",
                                                 "right_arm",
                                                 "both_arms",
                                                 "left_hand",
                                                 "right_hand" };

double distance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b)
{
    return std::hypot(b.x - a.x, b.y - a.y, b.z - a.z);
}

/// Clears busy_ however a goal ends.
struct BusyGuard
{
    std::atomic<bool>* flag;
    ~BusyGuard() { flag->store(false); }
};
}  // namespace

std::optional<std::map<std::string, double>> gripTarget(
    const std::map<std::string, double>& open, const std::map<std::string, double>& closed,
    double closure)
{
    if (!std::isfinite(closure) || closure < 0.0 || closure > 1.0 || open.size() != closed.size())
    {
        return std::nullopt;
    }
    std::map<std::string, double> out;
    for (const auto& [joint, from] : open)
    {
        const auto to = closed.find(joint);
        if (to == closed.end())
        {
            return std::nullopt;
        }
        out[joint] = from + (closure * (to->second - from));
    }
    return out;
}

geometry_msgs::msg::Point pointAlong(
    const geometry_msgs::msg::Point& shoulder, const geometry_msgs::msg::Point& target,
    double reach_m)
{
    const double              length = distance(shoulder, target);
    geometry_msgs::msg::Point hand   = shoulder;
    if (length < 1e-6)
    {
        return hand;
    }
    const double share = std::min(reach_m, length) / length;
    hand.x += (target.x - shoulder.x) * share;
    hand.y += (target.y - shoulder.y) * share;
    hand.z += (target.z - shoulder.z) * share;
    return hand;
}

G1ArmMotionServer::G1ArmMotionServer(const rclcpp::NodeOptions& options)
  : rclcpp::Node("g1_arm_motion_server", options)
  , tf_buffer_(get_clock())
  , tf_listener_(tf_buffer_)
{
    velocity_scaling_ = declare_parameter<double>("velocity_scaling", velocity_scaling_);
    planning_time_s_  = declare_parameter<double>("planning_time_s", planning_time_s_);
    planning_attempts_ =
        static_cast<int>(declare_parameter<int>("planning_attempts", planning_attempts_));
    max_reach_m_   = declare_parameter<double>("max_reach_m", max_reach_m_);
    point_reach_m_ = declare_parameter<double>("point_reach_m", point_reach_m_);
    ik_timeout_s_  = declare_parameter<double>("ik_timeout_s", ik_timeout_s_);
    ik_orientations_ =
        static_cast<int>(declare_parameter<int>("ik_orientations", ik_orientations_));
    left_shoulder_  = declare_parameter<std::string>("left_shoulder", left_shoulder_);
    right_shoulder_ = declare_parameter<std::string>("right_shoulder", right_shoulder_);
}

void G1ArmMotionServer::initialize()
{
    // The SRDF decides which groups exist: a robot with rubber hands has no hand groups, and a
    // MoveGroupInterface on a missing group throws.
    robot_model_loader::RobotModelLoader loader(shared_from_this(), "robot_description");
    const auto&                          model = loader.getModel();
    if (!model)
    {
        throw std::runtime_error("no robot model: robot_description_semantic is not set");
    }
    for (const char* name : kGroups)
    {
        if (!model->hasJointModelGroup(name))
        {
            continue;
        }
        auto group = std::make_unique<MoveGroup>(shared_from_this(), name);
        group->setPlanningTime(planning_time_s_);
        group->setNumPlanningAttempts(planning_attempts_);
        groups_.emplace(name, std::move(group));
    }
    if (groups_.empty())
    {
        throw std::runtime_error("the SRDF defines none of the arm or hand groups");
    }
    planning_frame_ = groups_.begin()->second->getPlanningFrame();

    posture_server_ =
        serve<SetArmPosture>("~/set_arm_posture", &G1ArmMotionServer::executeSetArmPosture);
    move_hand_server_ = serve<MoveHand>("~/move_hand", &G1ArmMotionServer::executeMoveHand);
    grip_server_      = serve<SetGrip>("~/set_grip", &G1ArmMotionServer::executeSetGrip);
    postures_service_ = create_service<GetPostures>(
        "~/get_postures",
        [this](
            const std::shared_ptr<GetPostures::Request>&  request,
            const std::shared_ptr<GetPostures::Response>& response) {
            getPostures(request, response);
        });

    std::string names;
    for (const auto& [name, _] : groups_)
    {
        names += (names.empty() ? "" : ", ") + name;
    }
    RCLCPP_INFO(get_logger(), "serving %s in %s", names.c_str(), planning_frame_.c_str());
}

template <typename Action>
typename rclcpp_action::Server<Action>::SharedPtr G1ArmMotionServer::serve(
    const std::string& name,
    void (G1ArmMotionServer::*execute)(const std::shared_ptr<GoalHandle<Action>>&))
{
    return rclcpp_action::create_server<Action>(
        this,
        name,
        [this](const rclcpp_action::GoalUUID&, const std::shared_ptr<const typename Action::Goal>&) {
            bool expected = false;
            if (!busy_.compare_exchange_strong(expected, true))
            {
                RCLCPP_WARN(get_logger(), "rejecting a goal: one is already running");
                return rclcpp_action::GoalResponse::REJECT;
            }
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<GoalHandle<Action>>&) {
            // A move under way runs to its end: stopping an arm mid-trajectory leaves it wherever
            // the controller held it, which is no safer.
            return rclcpp_action::CancelResponse::REJECT;
        },
        [this, execute](const std::shared_ptr<GoalHandle<Action>>& handle) {
            // Off the executor: planning and execution block for seconds.
            std::thread([this, execute, handle] {
                const BusyGuard guard{ &busy_ };
                try
                {
                    (this->*execute)(handle);
                }
                catch (const std::exception& e)
                {
                    auto result     = std::make_shared<typename Action::Result>();
                    result->success = false;
                    result->message = std::string("failed: ") + e.what();
                    handle->abort(result);
                }
            }).detach();
        });
}

G1ArmMotionServer::MoveGroup* G1ArmMotionServer::group(const std::string& name)
{
    const auto it = groups_.find(name);
    return it == groups_.end() ? nullptr : it->second.get();
}

std::optional<geometry_msgs::msg::Point> G1ArmMotionServer::linkPosition(const std::string& link)
{
    try
    {
        const auto t =
            tf_buffer_
                .lookupTransform(planning_frame_, link, tf2::TimePointZero, tf2::durationFromSec(0.5))
                .transform.translation;
        geometry_msgs::msg::Point p;
        p.x = t.x;
        p.y = t.y;
        p.z = t.z;
        return p;
    }
    catch (const tf2::TransformException&)
    {
        return std::nullopt;
    }
}

std::string
G1ArmMotionServer::planAndExecute(MoveGroup& group, double speed, const std::string& what) const
{
    const double scaling = velocity_scaling_ * (speed > 0.0 ? std::min(speed, 1.0) : 1.0);
    group.setMaxVelocityScalingFactor(scaling);
    group.setMaxAccelerationScalingFactor(scaling);
    group.setStartStateToCurrentState();

    MoveGroup::Plan plan;
    if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
        return "no plan to " + what;
    }
    const auto code = group.execute(plan);
    if (code != moveit::core::MoveItErrorCode::SUCCESS)
    {
        return std::format(
            "{} did not execute (MoveIt code {}); is the arm acquired (activate_arm)?",
            what,
            code.val);
    }
    return "";
}

void G1ArmMotionServer::executeSetArmPosture(const std::shared_ptr<GoalHandle<SetArmPosture>>& handle)
{
    const auto goal   = handle->get_goal();
    auto       result = std::make_shared<SetArmPosture::Result>();
    MoveGroup* g      = group(goal->group);
    if (g == nullptr)
    {
        result->message = "'" + goal->group + "' is not a group on this robot";
        handle->abort(result);
        return;
    }
    const auto names = g->getNamedTargets();
    if (std::ranges::find(names, goal->named_target) == names.end())
    {
        result->message = "'" + goal->named_target + "' is not a posture of " + goal->group;
        handle->abort(result);
        return;
    }
    g->setNamedTarget(goal->named_target);
    result->message = planAndExecute(*g, 1.0, goal->group + " " + goal->named_target);
    result->success = result->message.empty();
    if (result->success)
    {
        result->message = goal->group + " is at " + goal->named_target;
        handle->succeed(result);
    }
    else
    {
        handle->abort(result);
    }
}

void G1ArmMotionServer::executeMoveHand(const std::shared_ptr<GoalHandle<MoveHand>>& handle)
{
    const auto goal   = handle->get_goal();
    auto       result = std::make_shared<MoveHand::Result>();
    const auto refuse = [&](const std::string& why) {
        result->message = why;
        handle->abort(result);
    };

    if (goal->arm != "left" && goal->arm != "right")
    {
        refuse("arm must be left or right, not '" + goal->arm + "'");
        return;
    }
    if (goal->mode != "reach" && goal->mode != "point")
    {
        refuse("mode must be reach or point, not '" + goal->mode + "'");
        return;
    }
    MoveGroup* g = group(goal->arm + "_arm");
    if (g == nullptr)
    {
        refuse(goal->arm + "_arm is not a group on this robot");
        return;
    }

    geometry_msgs::msg::Point aim;
    try
    {
        aim = tf_buffer_.transform(goal->target, planning_frame_, tf2::durationFromSec(0.5)).point;
    }
    catch (const tf2::TransformException& e)
    {
        refuse(std::string("cannot place the target: ") + e.what());
        return;
    }
    aim.x += goal->offset.x;
    aim.y += goal->offset.y;
    aim.z += goal->offset.z;

    const auto shoulder = linkPosition(goal->arm == "left" ? left_shoulder_ : right_shoulder_);
    if (!shoulder)
    {
        refuse("the shoulder is not in TF");
        return;
    }
    const geometry_msgs::msg::Point hand =
        goal->mode == "point" ? pointAlong(*shoulder, aim, point_reach_m_) : aim;
    if (const double reach = distance(*shoulder, hand); reach > max_reach_m_)
    {
        refuse(std::format(
            "the hand would be {:.2f} m from the shoulder; it reaches {:.2f} m at most",
            reach,
            max_reach_m_));
        return;
    }

    const auto now = g->getCurrentState(1.0);
    if (!now)
    {
        refuse("the arm's state is not known");
        return;
    }
    const moveit::core::JointModelGroup* jmg = now->getJointModelGroup(g->getName());
    // The end effector's parent where the SRDF has one; a hand-less arm reaches with its tip.
    std::string tip = g->getEndEffectorLink();
    if (tip.empty())
    {
        tip = jmg->getLinkModelNames().back();
    }

    // A position goal through IK in a spread of orientations, the nearest collision-free one
    // kept: a five-joint arm's solver ignores orientation, a seven-joint one needs the spread.
    const planning_scene::PlanningScene body(now->getRobotModel());
    const auto                          free = [&body](
                          moveit::core::RobotState*            state,
                          const moveit::core::JointModelGroup* model,
                          const double*                        values) {
        state->setJointGroupPositions(model, values);
        state->update();
        return !body.isStateColliding(*state, model->getName());
    };
    const Eigen::Quaterniond               held(now->getGlobalLinkTransform(tip).linear());
    std::mt19937                           random(7);
    std::uniform_real_distribution<double> unit(0.0, 1.0);

    std::optional<moveit::core::RobotState> best;
    double                                  best_distance = 0.0;
    geometry_msgs::msg::Pose                target;
    target.position = hand;
    for (int i = 0; i <= ik_orientations_; ++i)
    {
        Eigen::Quaterniond q = held;
        if (i > 0)
        {
            // Shoemake's uniform rotations, seeded, so a goal plans the same way each time.
            const double u1 = unit(random);
            const double u2 = 2.0 * std::numbers::pi * unit(random);
            const double u3 = 2.0 * std::numbers::pi * unit(random);
            q               = Eigen::Quaterniond(
                std::sqrt(u1) * std::cos(u3),
                std::sqrt(1.0 - u1) * std::sin(u2),
                std::sqrt(1.0 - u1) * std::cos(u2),
                std::sqrt(u1) * std::sin(u3));
        }
        target.orientation.x = q.x();
        target.orientation.y = q.y();
        target.orientation.z = q.z();
        target.orientation.w = q.w();

        moveit::core::RobotState attempt = *now;
        if (attempt.setFromIK(jmg, target, tip, ik_timeout_s_, free))
        {
            // The IK's own position tolerance is the solver's; check it reached the point.
            const auto   reached = attempt.getGlobalLinkTransform(tip).translation();
            const double miss =
                std::hypot(reached.x() - hand.x, reached.y() - hand.y, reached.z() - hand.z);
            const double moved = attempt.distance(*now, jmg);
            if (miss < 0.01 && (!best || moved < best_distance))
            {
                best          = attempt;
                best_distance = moved;
            }
        }
    }
    if (!best)
    {
        refuse(std::format(
            "no arm pose puts {} at ({:.2f}, {:.2f}, {:.2f})",
            tip,
            hand.x,
            hand.y,
            hand.z));
        return;
    }

    g->setJointValueTarget(*best);
    result->message = planAndExecute(*g, goal->speed, goal->arm + " hand " + goal->mode);
    result->success = result->message.empty();
    if (result->success)
    {
        result->message =
            std::format("{} at ({:.2f}, {:.2f}, {:.2f})", tip, hand.x, hand.y, hand.z);
        handle->succeed(result);
    }
    else
    {
        handle->abort(result);
    }
}

void G1ArmMotionServer::executeSetGrip(const std::shared_ptr<GoalHandle<SetGrip>>& handle)
{
    const auto goal   = handle->get_goal();
    auto       result = std::make_shared<SetGrip::Result>();
    const auto refuse = [&](const std::string& why) {
        result->message = why;
        handle->abort(result);
    };

    if (goal->arm != "left" && goal->arm != "right")
    {
        refuse("arm must be left or right, not '" + goal->arm + "'");
        return;
    }
    MoveGroup* g = group(goal->arm + "_hand");
    if (g == nullptr)
    {
        refuse("this robot has no " + goal->arm + "_hand group: no hand joints to close");
        return;
    }
    const auto names = g->getNamedTargets();
    for (const char* needed : { "open", "closed" })
    {
        if (std::ranges::find(names, needed) == names.end())
        {
            refuse(goal->arm + "_hand has no '" + needed + "' posture in the SRDF");
            return;
        }
    }
    const auto joints = gripTarget(
        g->getNamedTargetValues("open"),
        g->getNamedTargetValues("closed"),
        goal->closure);
    if (!joints)
    {
        refuse(std::format("closure must be between 0 and 1, not {}", goal->closure));
        return;
    }
    g->setJointValueTarget(*joints);
    result->message =
        planAndExecute(*g, goal->speed, std::format("{} grip {:.2f}", goal->arm, goal->closure));
    result->success = result->message.empty();
    if (result->success)
    {
        result->message = std::format("{} hand at closure {:.2f}", goal->arm, goal->closure);
        handle->succeed(result);
    }
    else
    {
        handle->abort(result);
    }
}

void G1ArmMotionServer::getPostures(
    const std::shared_ptr<GetPostures::Request>& /*request*/,
    const std::shared_ptr<GetPostures::Response>& response)
{
    for (const auto& [name, g] : groups_)
    {
        std::string postures;
        for (const auto& posture : g->getNamedTargets())
        {
            postures += (postures.empty() ? "" : ",") + posture;
        }
        response->groups.push_back(name);
        response->postures.push_back(postures);
    }
}

}  // namespace g1_manipulation
