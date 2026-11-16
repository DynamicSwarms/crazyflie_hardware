#pragma once

#include <cstddef>
#include <cstdint>
#include <chrono>
#include <optional>

namespace libcrtp {

struct CrtpContainerStats
{
    // Cumulative accepted packets, including removed links; unsigned wrap is defined.
    uint64_t enqueuedPackets{0};
    size_t unicastLinks{0};
    size_t broadcastLinks{0};
    size_t outgoingPackets{0};
    size_t pendingResponses{0};
    size_t linksInBackoff{0};
    std::optional<std::chrono::steady_clock::time_point> oldestOutgoingCreatedAt;
};

} // namespace libcrtp
