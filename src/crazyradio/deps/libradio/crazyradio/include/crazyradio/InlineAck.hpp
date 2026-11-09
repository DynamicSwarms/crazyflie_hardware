#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace libradio::crazyradio {

// Decoded inline-mode-2 reply. RSSI belongs to a received ACK, not a timeout.
struct InlineAck {
    uint8_t total_length = 0;
    std::size_t received_length = 0;
    bool ack = false;
    bool weak_signal = false;
    bool invalid_settings = false;
    uint8_t retry = 0;
    uint8_t raw_rssi = 0;
    bool rssi_valid = false;
    int rssi_dbm = 0;
    bool malformed = false;
    int64_t transfer_us = 0;
    uint8_t data_length = 0;
    uint8_t data[32]{};
};

inline InlineAck decodeInlineAck(const uint8_t* bytes, std::size_t size)
{
    InlineAck reply;
    reply.received_length = size;
    if (size > 0) reply.total_length = bytes[0];
    if (size >= 2) {
        reply.ack = bytes[1] & 0x01;
        reply.weak_signal = bytes[1] & 0x02;
        reply.invalid_settings = bytes[1] & 0x04;
        reply.retry = bytes[1] >> 4;
    }
    // Three header bytes (length, status, RSSI), then at most 32 radio bytes.
    reply.malformed = size < 3 || size > 35 || reply.total_length != size;
    if (size >= 3) reply.raw_rssi = bytes[2];
    reply.rssi_valid = reply.ack && !reply.invalid_settings && !reply.malformed;
    if (reply.rssi_valid) reply.rssi_dbm = -static_cast<int>(reply.raw_rssi);
    if (!reply.malformed) {
        reply.data_length = static_cast<uint8_t>(size - 3);
        std::copy_n(bytes + 3, reply.data_length, reply.data);
    }
    return reply;
}

} // namespace libradio::crazyradio
