/**
 * @file test_arm_motion.cpp
 * @brief The motion server's grip interpolation and pointing reach.
 */

#include <gmock/gmock.h>

#include <cmath>
#include <limits>

#include "g1_manipulation/g1_arm_motion_server_node.hpp"

using g1_manipulation::gripTarget;
using g1_manipulation::pointAlong;

TEST(GripTarget, InterpolatesEveryJointBetweenOpenAndClosed)
{
    const std::map<std::string, double> open{ { "a", 0.0 }, { "b", -1.0 } };
    const std::map<std::string, double> closed{ { "a", 1.4 }, { "b", 1.0 } };
    const auto                          half = gripTarget(open, closed, 0.5);
    ASSERT_TRUE(half.has_value());
    EXPECT_DOUBLE_EQ(half->at("a"), 0.7);
    EXPECT_DOUBLE_EQ(half->at("b"), 0.0);
    EXPECT_DOUBLE_EQ(gripTarget(open, closed, 0.0)->at("a"), 0.0);
    EXPECT_DOUBLE_EQ(gripTarget(open, closed, 1.0)->at("b"), 1.0);
}

TEST(GripTarget, RefusesOutOfRangeAndMismatchedPostures)
{
    const std::map<std::string, double> open{ { "a", 0.0 } };
    EXPECT_FALSE(gripTarget(open, { { "a", 1.0 } }, 1.5).has_value());
    EXPECT_FALSE(gripTarget(open, { { "a", 1.0 } }, -0.1).has_value());
    EXPECT_FALSE(
        gripTarget(open, { { "a", 1.0 } }, std::numeric_limits<double>::quiet_NaN()).has_value());
    EXPECT_FALSE(gripTarget(open, { { "b", 1.0 } }, 0.5).has_value());
}

TEST(PointAlong, StopsAtTheReachOrAtANearerTarget)
{
    geometry_msgs::msg::Point shoulder;
    geometry_msgs::msg::Point far;
    far.x = 2.0;
    EXPECT_DOUBLE_EQ(pointAlong(shoulder, far, 0.4).x, 0.4);

    geometry_msgs::msg::Point near;
    near.y = 0.2;
    EXPECT_DOUBLE_EQ(pointAlong(shoulder, near, 0.4).y, 0.2);
    EXPECT_DOUBLE_EQ(pointAlong(shoulder, shoulder, 0.4).x, 0.0);
}
