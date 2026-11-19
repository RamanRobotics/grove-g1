#ifndef G1_HAND_INTERFACE__LINKERBOT_O6_PROTOCOL_HPP_
#define G1_HAND_INTERFACE__LINKERBOT_O6_PROTOCOL_HPP_

/**
 * @file linkerbot_o6_protocol.hpp
 * @brief The LinkerBot O6 hand's CAN frames, pure so they unit-test without a bus.
 *
 * From LinkerBot's own SDK (linker-bot/linkerhand-python-sdk, LinkerHandO6Can): classic CAN at
 * 1 Mbit/s, standard 11-bit ids, one id per hand that both directions share. Byte 0 is the frame
 * type, the rest its payload, one byte per actuated joint.
 */

#include <linux/can.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace g1_hand_interface::o6
{

/// Actuated joints per hand. The URDF's five *_ip/*_dip joints are mimics and not on the wire.
inline constexpr std::size_t kNumActuated = 6;

/// The SDK's default ids, which the hands ship with.
inline constexpr canid_t kLeftHandId  = 0x28;
inline constexpr canid_t kRightHandId = 0x27;

/// Frame types. 0x01 carries six positions to the hand; sent empty, it asks for the current six,
/// which the hand answers in a 0x01 of its own.
inline constexpr std::uint8_t kFramePosition = 0x01;
/// Per-joint torque limit, 0-255.
inline constexpr std::uint8_t kFrameTorque = 0x02;
/// Per-joint speed, 0-255.
inline constexpr std::uint8_t kFrameSpeed = 0x05;

/// Wire order, both hands: the SDK's get_finger_order(). The URDF's joint names, less the lh_/rh_
/// prefix.
inline constexpr std::array<const char*, kNumActuated> kJointSuffixes = {
    "thumb_cmc_pitch",  "thumb_cmc_yaw",  "index_mcp_pitch",
    "middle_mcp_pitch", "ring_mcp_pitch", "pinky_mcp_pitch",
};

using Bytes = std::array<std::uint8_t, kNumActuated>;

/**
 * @brief Radians to the wire's 0-255. 255 is the joint's `min` (open), 0 its `max` (closed),
 *        the SDK's mapping for both O6 hands. Out-of-range and non-finite input clamps; NaN opens.
 */
[[nodiscard]] std::uint8_t radToWire(double position, double min, double max) noexcept;

/**
 * @brief The inverse of radToWire, exact at both ends.
 */
[[nodiscard]] double wireToRad(std::uint8_t value, double min, double max) noexcept;

/**
 * @brief A frame of `type` carrying six bytes, or none when `payload` is empty.
 */
[[nodiscard]] can_frame makeFrame(canid_t id, std::uint8_t type, const Bytes* payload) noexcept;

/**
 * @brief The six positions in a hand's 0x01 reply, if `frame` is one from `id`.
 *
 * Anything else on the bus (the other hand, other frame types, a short frame) is not a state.
 */
[[nodiscard]] std::optional<Bytes> parsePositionReply(const can_frame& frame, canid_t id) noexcept;

}  // namespace g1_hand_interface::o6

#endif  // G1_HAND_INTERFACE__LINKERBOT_O6_PROTOCOL_HPP_
