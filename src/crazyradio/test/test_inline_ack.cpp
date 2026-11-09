#include "crazyradio/InlineAck.hpp"

#include <array>
#include <stdexcept>

using libradio::crazyradio::decodeInlineAck;

void require(bool condition)
{
    if (!condition) throw std::runtime_error("inline mode 2 decoder regression");
}

int main()
{
    // Header RSSI must not be interpreted as a CRTP payload byte.
    const uint8_t packet[] = {7, 0x23, 67, 0xD1, 0x01, 0x02, 0x03};
    const auto ack = decodeInlineAck(packet, sizeof(packet));
    require(!ack.malformed && ack.ack && ack.weak_signal && ack.retry == 2);
    require(ack.rssi_valid && ack.rssi_dbm == -67);
    require(ack.data_length == 4 && ack.data[0] == 0xD1 && ack.data[3] == 3);

    const uint8_t lost[] = {3, 0x10, 0};
    const auto noAck = decodeInlineAck(lost, sizeof(lost));
    require(!noAck.malformed && !noAck.ack && noAck.retry == 1);
    require(!noAck.rssi_valid && noAck.data_length == 0);

    // A legitimate empty ACK must not wrap its CRTP length to 255.
    const uint8_t empty[] = {3, 1, 45};
    const auto emptyAck = decodeInlineAck(empty, sizeof(empty));
    require(!emptyAck.malformed && emptyAck.ack && emptyAck.data_length == 0);

    const uint8_t invalid[] = {3, 5, 45};
    require(decodeInlineAck(invalid, sizeof(invalid)).invalid_settings);
    require(!decodeInlineAck(invalid, sizeof(invalid)).rssi_valid);

    std::array<uint8_t, 36> wire{};
    // Exhaust truncated headers, length mismatches, and oversized responses.
    for (std::size_t size = 0; size <= wire.size(); ++size) {
        for (int declared = 0; declared <= 255; ++declared) {
            wire[0] = declared;
            const auto result = decodeInlineAck(wire.data(), size);
            const bool valid = size >= 3 && size <= 35 && declared == static_cast<int>(size);
            require(result.malformed == !valid);
            require(result.data_length == (valid ? size - 3 : 0));
        }
    }
    wire[0] = 35;
    wire[1] = 1;
    wire[2] = 70;
    for (int i = 3; i < 35; ++i) wire[i] = i;
    const auto maximum = decodeInlineAck(wire.data(), 35);
    require(maximum.data_length == 32 && maximum.data[31] == 34);
}
