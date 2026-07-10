#include "virtual_bridge/ControlFrame.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace virtual_bridge {
namespace {

constexpr std::uint8_t kFrameHead = 0x42;
constexpr std::uint8_t kAddrCarControl = 1;
constexpr std::uint8_t kFrameLenField = 10;

std::uint8_t checksumFirstNine(const std::uint8_t* data) {
    std::uint8_t checksum = 0;
    for (int i = 0; i < 9; ++i) {
        checksum = static_cast<std::uint8_t>(checksum + data[i]);
    }
    return checksum;
}

} // namespace

std::optional<ControlCommand> parseControlFrame(const std::uint8_t* data, std::size_t len) {
    if (data == nullptr || len < kControlFrameSize) return std::nullopt;
    if (data[0] != kFrameHead || data[1] != kAddrCarControl || data[2] != kFrameLenField) {
        return std::nullopt;
    }
    if (checksumFirstNine(data) != data[9]) return std::nullopt;

    ControlCommand command;
    static_assert(sizeof(command.speedMps) == 4, "speed float must be 32-bit");
    std::memcpy(&command.speedMps, data + 3, sizeof(command.speedMps));
    if (!std::isfinite(command.speedMps)) return std::nullopt;

    command.servoPulseUs = static_cast<std::uint16_t>(data[7]) |
                           static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[8]) << 8u);
    return command;
}

void appendControlFrameBytes(std::vector<std::uint8_t>& pending,
                             const std::uint8_t* data,
                             std::size_t len,
                             std::vector<ControlCommand>& outCommands) {
    if (data == nullptr || len == 0) return;
    pending.insert(pending.end(), data, data + len);

    while (pending.size() >= kControlFrameSize) {
        const auto header = std::find(pending.begin(), pending.end(), kFrameHead);
        if (header == pending.end()) {
            pending.clear();
            return;
        }
        if (header != pending.begin()) {
            pending.erase(pending.begin(), header);
        }
        if (pending.size() < kControlFrameSize) return;

        const std::optional<ControlCommand> command =
            parseControlFrame(pending.data(), kControlFrameSize);
        if (command.has_value()) {
            outCommands.push_back(*command);
            pending.erase(pending.begin(), pending.begin() + static_cast<std::ptrdiff_t>(kControlFrameSize));
        } else {
            pending.erase(pending.begin());
        }
    }

    if (pending.size() > 1024) {
        pending.erase(pending.begin(), pending.end() - 1024);
    }
}

} // namespace virtual_bridge
