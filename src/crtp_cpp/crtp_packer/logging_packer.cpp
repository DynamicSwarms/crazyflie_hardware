#include "crtp_packer/logging_packer.hpp"
#include <tuple>

#define PORT_LOGGING 5

// Channels used for the logging port
#define TOC_CHANNEL 0
#define CONTROL_CHANNEL 1
#define LOGDATA_CHANNEL 2

// Commands used when accessing the Log configurations
#define CMD_CREATE_BLOCK 0
#define CMD_APPEND_BLOCK 1
#define CMD_DELETE_BLOCK 2
#define CMD_START_LOGGING 3
#define CMD_STOP_LOGGING 4
#define CMD_RESET_LOGGING 5
#define CMD_CREATE_BLOCK_V2 6
#define CMD_APPEND_BLOCK_V2 7

LoggingPacker::LoggingPacker()
    : TocPacker(PORT_LOGGING) {}

std::vector<uint8_t> LoggingPacker::_create_block_content(const std::vector<std::pair<uint8_t, uint16_t>>& content) {
  std::vector<uint8_t> data;
  for (const auto& el : content) {
    uint8_t storage_and_fetch = el.first;
    uint16_t index = el.second;

    // Pack the data using bitwise operations (like struct.pack in Python)
    data.push_back(storage_and_fetch);
    data.push_back(static_cast<uint8_t>(index & 0xFF));       // Lower byte
    data.push_back(static_cast<uint8_t>((index >> 8) & 0xFF)); // Higher byte
  }
  return data;
}

CrtpPacket LoggingPacker::prepare_control_packet(const std::vector<uint8_t>& data)
{
  return CrtpPacker::prepare_packet(CONTROL_CHANNEL, data);
}


std::tuple<CrtpPacket, bool, int> LoggingPacker::create_block(uint8_t index, const std::vector<std::pair<uint8_t, uint16_t>>& content) {
    std::vector<uint8_t> data;
    data.push_back(CMD_CREATE_BLOCK_V2);
    data.push_back(index);

    std::vector<uint8_t> content_data = _create_block_content(content);
    data.insert(data.end(), content_data.begin(), content_data.end());
    return std::make_tuple(prepare_control_packet(data), true, 2);
}

std::tuple<CrtpPacket, bool, int> LoggingPacker::start_block(uint8_t index, uint8_t period) {
  std::vector<uint8_t> data = {CMD_START_LOGGING, index, period};
  return std::make_tuple(prepare_control_packet(data), true, 2);
}

std::tuple<CrtpPacket, bool, int> LoggingPacker::stop_block(uint8_t index) {
  std::vector<uint8_t> data = {CMD_STOP_LOGGING, index};
  return std::make_tuple(prepare_control_packet(data), true, 2);
}