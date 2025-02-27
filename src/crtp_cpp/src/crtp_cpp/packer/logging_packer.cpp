#include "crtp_cpp/packer/logging_packer.hpp"
#include <tuple>
#include <cstring> // Add this line

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

LogTocEntry::LogTocEntry(const std::vector<uint8_t>& data)
{

}

// Constructor from comma-separated string
LogTocEntry::LogTocEntry(const std::string& csv) {
    //  std::istringstream ss(csv);
    //  std::string token;
    //  // Parse ID
    //  std::getline(ss, token, ',');
    //  id = static_cast<uint16_t>(std::stoi(token));
    //  // Parse Group
    //  std::getline(ss, group, ',');
    //  // Parse Name
    //  std::getline(ss, name, ',');
    //  // Parse Type
    //  std::getline(ss, token, ',');
    //  type = (ParamType)static_cast<uint8_t>(std::stoi(token));
    //  // Parse Readonly
    //  std::getline(ss, token, ',');
    //  readonly = token == "1";
}

std::string LogTocEntry::toString() const {
    //std::ostringstream ss;
    //ss << id << "," << group << "," << name << "," << type << "," << (readonly ? "true" : "false");
    //return ss.str();
    return std::string("None");
}


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


CrtpRequest LoggingPacker::create_block(uint8_t index, const std::vector<std::pair<uint8_t, uint16_t>>& content) {
  CrtpRequest request;
  std::vector<uint8_t> data;
  data.push_back(CMD_CREATE_BLOCK_V2);
  data.push_back(index);

  std::vector<uint8_t> content_data = _create_block_content(content);
  data.insert(data.end(), content_data.begin(), content_data.end());
  request.packet = prepare_control_packet(data);
  request.expects_response = true;
  request.matching_bytes = 2;
  return request;
}

CrtpRequest LoggingPacker::start_block(uint8_t index, uint8_t period) {
  CrtpRequest request;
  std::vector<uint8_t> data = {CMD_START_LOGGING, index, period};
  request.packet = prepare_control_packet(data);
  request.expects_response = true;
  request.matching_bytes = 2;
  return request;
}

CrtpRequest LoggingPacker::stop_block(uint8_t index) {
  CrtpRequest request;
  std::vector<uint8_t> data = {CMD_STOP_LOGGING, index};
  request.packet = prepare_control_packet(data);
  request.expects_response = true;
  request.matching_bytes = 2;
  return request;
}

uint8_t LogTocEntry::size() const
{
  switch (type) {
    case LogTypeUint8:
    case LogTypeInt8:
        return 1;
    case LogTypeUint16:
    case LogTypeInt16:
    case LogTypeFP16:  // FP16 (Half-Precision Float) is 2 bytes
        return 2;
    case LogTypeUint32:
    case LogTypeInt32:
    case LogTypeFloat: // Standard 32-bit float
        return 4;
    default:
        return 0; // Unknown type
  }
}

float LogTocEntry::fp16_to_float(uint16_t fp16) const
{
    uint32_t s = (fp16 >> 15) & 0x00000001; // sign
    uint32_t e = (fp16 >> 10) & 0x0000001f; // exponent
    uint32_t f = fp16 & 0x000003ff;         // fraction

    if (e == 0) {
        if (f == 0) {
            uint32_t result = s << 31;
            float float_result;
            std::memcpy(&float_result, &result, sizeof(float));
            return float_result;
        } else {
            while (!(f & 0x00000400)) {
                f <<= 1;
                e -= 1;
            }
            e += 1;
            f &= ~0x00000400;
        }
    } else if (e == 31) {
        if (f == 0) {
            uint32_t result = (s << 31) | 0x7f800000;
            float float_result;
            std::memcpy(&float_result, &result, sizeof(float));
            return float_result;
        } else {
            uint32_t result = (s << 31) | 0x7f800000 | (f << 13);
            float float_result;
            std::memcpy(&float_result, &result, sizeof(float));
            return float_result;
        }
    }

    e += 127 - 15;
    f <<= 13;
    uint32_t result = (s << 31) | (e << 23) | f;
    float float_result;
    std::memcpy(&float_result, &result, sizeof(float));
    return float_result;
}

float LogTocEntry::to_float(const std::vector<uint8_t>& data) const
{ 
  if (data.size() != size()) return 0.0f;

  switch (type) {
      case LogTypeUint8:
          return static_cast<float>(data[0]); // 1-byte unsigned int
      case LogTypeInt8:
          return static_cast<float>(static_cast<int8_t>(data[0])); // 1-byte signed int
      case LogTypeUint16: {
          uint16_t value;
          std::memcpy(&value, data.data(), sizeof(value));
          return static_cast<float>(value);
      }
      case LogTypeInt16: {
          int16_t value;
          std::memcpy(&value, data.data(), sizeof(value));
          return static_cast<float>(value);
      }
      case LogTypeUint32: {
          uint32_t value;
          std::memcpy(&value, data.data(), sizeof(value));
          return static_cast<float>(value);
      }
      case LogTypeInt32: {
          int32_t value;
          std::memcpy(&value, data.data(), sizeof(value));
          return static_cast<float>(value);
      }
      case LogTypeFloat: {
          float value;
          std::memcpy(&value, data.data(), sizeof(value)); // Direct float conversion
          return value;
      }
      case LogTypeFP16: {
          // FP16 to float conversion (requires additional handling)
          // Placeholder: Assuming we receive IEEE 754 half-precision in `data`
          uint16_t fp16_value;
          std::memcpy(&fp16_value, data.data(), sizeof(fp16_value));
          return fp16_to_float(fp16_value); // Convert half-precision float
      }
      default:
          return 0.0f; // Unknown type
  }
}