#pragma once

#include <vector>
#include <cstdint> // For uint8_t, uint16_t, etc.
#include "toc_packer.hpp" // Assuming this is already converted
#include <tuple>



class LoggingPacker : public TocPacker {
public:
 

  LoggingPacker(); // Constructor

  enum LogType {
    LogTypeUint8 = 0x01;
    LogTypeUint16 = 0x02; 
    LogTypeUint32 = 0x03; 
    LogTypeInt8 = 0x04;
    LogTypeInt16 = 0x05;
    LogTypeInt32 = 0x06;
    LogTypeFloat = 0x07;
    LogTypeFP16 = 0x08;
  };

  struct LogTocEntry {
    uint16_t id;
    std::string group;
    std::string name;
    LogType type;
  };

  union LogValue {
      uint8_t valueUint8;
      uint16_t valueUint16;
      uint32_t valueUint32;
      int8_t  valueInt8;
      int16_t valueInt16;
      int32_t valueInt32;
      float valueFloat;
      float valueFP16;

  };

  std::tuple<CrtpPacket, bool, int> create_block(uint8_t index, const std::vector<std::pair<uint8_t, uint16_t>>& content);
  std::tuple<CrtpPacket, bool, int> start_block(uint8_t index, uint8_t period);
  std::tuple<CrtpPacket, bool, int> stop_block(uint8_t index);


private:
  std::vector<uint8_t> _create_block_content(const std::vector<std::pair<uint8_t, uint16_t>>& content);

protected:
    CrtpPacket prepare_control_packet(const std::vector<uint8_t>& data);
};
