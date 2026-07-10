#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace virtual_bridge {

constexpr std::size_t kControlFrameSize = 11;

struct ControlCommand {
    float speedMps = 0.0f;
    std::uint16_t servoPulseUs = 1500;
};

std::optional<ControlCommand> parseControlFrame(const std::uint8_t* data, std::size_t len);

void appendControlFrameBytes(std::vector<std::uint8_t>& pending,
                             const std::uint8_t* data,
                             std::size_t len,
                             std::vector<ControlCommand>& outCommands);

} // namespace virtual_bridge
