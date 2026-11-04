/**
 * @file test_linkerbot_o6_protocol.cpp
 * @brief The O6's rad/byte mapping, its frames, and plugin discovery.
 */

#include <gmock/gmock.h>

#include <cmath>
#include <hardware_interface/system_interface.hpp>
#include <limits>
#include <pluginlib/class_loader.hpp>

#include "g1_hand_interface/linkerbot_o6_protocol.hpp"

namespace o6 = g1_hand_interface::o6;

TEST(LinkerBotO6Mapping, OpenIs255AndClosedIsZero)
{
    // LinkerBot's SDK maps 255 to the joint's minimum, its open end, on both hands.
    EXPECT_EQ(o6::radToWire(0.0, 0.0, 1.6), 255);
    EXPECT_EQ(o6::radToWire(1.6, 0.0, 1.6), 0);
    EXPECT_EQ(o6::radToWire(0.8, 0.0, 1.6), 128);
}

TEST(LinkerBotO6Mapping, ClampsAndOpensOnGarbage)
{
    EXPECT_EQ(o6::radToWire(-1.0, 0.0, 1.6), 255);
    EXPECT_EQ(o6::radToWire(9.0, 0.0, 1.6), 0);
    EXPECT_EQ(o6::radToWire(std::numeric_limits<double>::quiet_NaN(), 0.0, 1.6), 255);
    EXPECT_EQ(o6::radToWire(0.5, 1.0, 1.0), 255);
}

TEST(LinkerBotO6Mapping, RoundTripsWithinOneStep)
{
    for (int step = 0; step <= 27; ++step)
    {
        const double q = 0.05 * step;
        EXPECT_NEAR(o6::wireToRad(o6::radToWire(q, 0.0, 1.36), 0.0, 1.36), q, 1.36 / 255.0);
    }
    EXPECT_DOUBLE_EQ(o6::wireToRad(255, 0.0, 0.58), 0.0);
    EXPECT_DOUBLE_EQ(o6::wireToRad(0, 0.0, 0.58), 0.58);
}

TEST(LinkerBotO6Frames, PositionCommandCarriesSixBytes)
{
    const o6::Bytes payload{ 1, 2, 3, 4, 5, 6 };
    const auto      frame = o6::makeFrame(o6::kLeftHandId, o6::kFramePosition, &payload);
    EXPECT_EQ(frame.can_id, 0x28U);
    EXPECT_EQ(frame.can_dlc, 7);
    EXPECT_EQ(frame.data[0], 0x01);
    EXPECT_EQ(frame.data[6], 6);
}

TEST(LinkerBotO6Frames, StateRequestIsTheBareType)
{
    const auto frame = o6::makeFrame(o6::kRightHandId, o6::kFramePosition, nullptr);
    EXPECT_EQ(frame.can_id, 0x27U);
    EXPECT_EQ(frame.can_dlc, 1);
}

TEST(LinkerBotO6Frames, OnlyThisHandsPositionReplyIsState)
{
    const o6::Bytes payload{ 10, 20, 30, 40, 50, 60 };
    auto            reply = o6::makeFrame(o6::kLeftHandId, o6::kFramePosition, &payload);

    const auto parsed = o6::parsePositionReply(reply, o6::kLeftHandId);
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ((*parsed)[5], 60);

    EXPECT_FALSE(o6::parsePositionReply(reply, o6::kRightHandId).has_value());
    auto speed = o6::makeFrame(o6::kLeftHandId, o6::kFrameSpeed, &payload);
    EXPECT_FALSE(o6::parsePositionReply(speed, o6::kLeftHandId).has_value());
    auto request = o6::makeFrame(o6::kLeftHandId, o6::kFramePosition, nullptr);
    EXPECT_FALSE(o6::parsePositionReply(request, o6::kLeftHandId).has_value());
}

TEST(LinkerBotO6Pluginlib, DiscoversAndInstantiates)
{
    pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
        "hardware_interface",
        "hardware_interface::SystemInterface");
    ASSERT_TRUE(loader.isClassAvailable("g1_hand_interface/LinkerBotO6System"));
    EXPECT_NE(loader.createUniqueInstance("g1_hand_interface/LinkerBotO6System"), nullptr);
}
