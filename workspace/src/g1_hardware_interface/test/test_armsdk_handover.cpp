/**
 * @file test_armsdk_handover.cpp
 * @brief The rt/arm_sdk weight ramp and the loco twist clamp.
 */

#include <gmock/gmock.h>

#include <cmath>
#include <limits>

#include "g1_hardware_interface/g1_armsdk_system.hpp"

using g1_hardware_interface::nextArmSdkWeight;
using g1_hardware_interface::sanitizeLocoCommand;

TEST(ArmSdkWeight, RampsUpOnlyWhileEveryJointIsPowered)
{
    EXPECT_DOUBLE_EQ(nextArmSdkWeight(0.0, true, 0.25), 0.25);
    EXPECT_DOUBLE_EQ(nextArmSdkWeight(0.9, true, 0.25), 1.0);
    EXPECT_DOUBLE_EQ(nextArmSdkWeight(1.0, true, 0.25), 1.0);
}

TEST(ArmSdkWeight, HandsBackWhenAJointLosesItsController)
{
    // An unclaimed joint at weight 1 would be limp; the onboard pose takes the arms back instead.
    EXPECT_DOUBLE_EQ(nextArmSdkWeight(1.0, false, 0.25), 0.75);
    EXPECT_DOUBLE_EQ(nextArmSdkWeight(0.1, false, 0.25), 0.0);
    EXPECT_DOUBLE_EQ(nextArmSdkWeight(0.0, false, 0.25), 0.0);
}

TEST(ArmSdkWeight, NonFiniteWeightFallsToTheOnboardPose)
{
    EXPECT_DOUBLE_EQ(nextArmSdkWeight(std::numeric_limits<double>::quiet_NaN(), true, 0.25), 0.0);
}

TEST(LocoCommand, ClampsAndZeroesNonFinite)
{
    EXPECT_DOUBLE_EQ(sanitizeLocoCommand(0.3, 0.6), 0.3);
    EXPECT_DOUBLE_EQ(sanitizeLocoCommand(2.0, 0.6), 0.6);
    EXPECT_DOUBLE_EQ(sanitizeLocoCommand(-2.0, 0.6), -0.6);
    EXPECT_DOUBLE_EQ(sanitizeLocoCommand(std::numeric_limits<double>::infinity(), 0.6), 0.0);
    EXPECT_DOUBLE_EQ(sanitizeLocoCommand(std::numeric_limits<double>::quiet_NaN(), 0.6), 0.0);
    // A non-positive limit disables the clamp.
    EXPECT_DOUBLE_EQ(sanitizeLocoCommand(2.0, 0.0), 2.0);
}
