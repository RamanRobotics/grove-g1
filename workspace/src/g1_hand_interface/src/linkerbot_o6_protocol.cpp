/**
 * @file linkerbot_o6_protocol.cpp
 * @brief The LinkerBot O6 hand's CAN frames.
 */

#include "g1_hand_interface/linkerbot_o6_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace g1_hand_interface::o6
{

std::uint8_t radToWire(double position, double min, double max) noexcept
{
    if (!std::isfinite(position) || !(max > min))
    {
        return 255;
    }
    const double closed = std::clamp((position - min) / (max - min), 0.0, 1.0);
    return static_cast<std::uint8_t>(std::lround(255.0 * (1.0 - closed)));
}

double wireToRad(std::uint8_t value, double min, double max) noexcept
{
    return min + (max - min) * (1.0 - static_cast<double>(value) / 255.0);
}

can_frame makeFrame(canid_t id, std::uint8_t type, const Bytes* payload) noexcept
{
    can_frame frame{};
    frame.can_id  = id & CAN_SFF_MASK;
    frame.data[0] = type;
    frame.can_dlc = 1;
    if (payload != nullptr)
    {
        std::memcpy(&frame.data[1], payload->data(), kNumActuated);
        frame.can_dlc = 1 + kNumActuated;
    }
    return frame;
}

std::optional<Bytes> parsePositionReply(const can_frame& frame, canid_t id) noexcept
{
    if ((frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0 ||
        (frame.can_id & CAN_SFF_MASK) != (id & CAN_SFF_MASK) || frame.can_dlc < 1 + kNumActuated ||
        frame.data[0] != kFramePosition)
    {
        return std::nullopt;
    }
    Bytes bytes{};
    std::memcpy(bytes.data(), &frame.data[1], kNumActuated);
    return bytes;
}

}  // namespace g1_hand_interface::o6
