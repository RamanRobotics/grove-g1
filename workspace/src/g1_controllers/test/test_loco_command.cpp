/**
 * @file test_loco_command.cpp
 * @brief What G1LocoController hands the onboard locomotion for a /cmd_vel sample.
 */

#include <gmock/gmock.h>

#include <limits>

#include "g1_controllers/g1_loco_controller.hpp"

using g1_controllers::locoCommand;

namespace
{
geometry_msgs::msg::Twist twist(double x, double y, double z)
{
    geometry_msgs::msg::Twist t;
    t.linear.x  = x;
    t.linear.y  = y;
    t.angular.z = z;
    return t;
}
}  // namespace

TEST(LocoCommand, PassesAFreshCommandThroughClamped)
{
    const auto out = locoCommand(twist(1.0, -0.2, 2.0), 0.1, true, 0.5, 0.6, 0.9);
    EXPECT_DOUBLE_EQ(out.linear.x, 0.6);
    EXPECT_DOUBLE_EQ(out.linear.y, -0.2);
    EXPECT_DOUBLE_EQ(out.angular.z, 0.9);
}

TEST(LocoCommand, ZeroWhenStaleOrNeverReceived)
{
    // A dead planner halts the robot rather than leaving it walking.
    const auto stale = locoCommand(twist(0.3, 0.0, 0.0), 0.6, true, 0.5, 0.6, 0.9);
    EXPECT_DOUBLE_EQ(stale.linear.x, 0.0);
    const auto never = locoCommand(twist(0.3, 0.0, 0.0), 0.0, false, 0.5, 0.6, 0.9);
    EXPECT_DOUBLE_EQ(never.linear.x, 0.0);
}

TEST(LocoCommand, ZeroTimeoutNeverExpires)
{
    const auto out = locoCommand(twist(0.3, 0.0, 0.0), 100.0, true, 0.0, 0.6, 0.9);
    EXPECT_DOUBLE_EQ(out.linear.x, 0.3);
}

TEST(LocoCommand, NonFiniteComponentsAreZero)
{
    const auto out = locoCommand(
        twist(std::numeric_limits<double>::quiet_NaN(), 0.1, std::numeric_limits<double>::infinity()),
        0.0,
        true,
        0.5,
        0.6,
        0.9);
    EXPECT_DOUBLE_EQ(out.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(out.linear.y, 0.1);
    EXPECT_DOUBLE_EQ(out.angular.z, 0.0);
}
