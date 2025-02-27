#pragma once

#include <vector>
#include <cstdint> // For uint8_t, uint16_t, etc.
#include "crtp_cpp/packer/toc_packer.hpp" // Assuming this is already converted
#include "crtp_cpp/logic/toc_logic.hpp"

#include <tuple>


enum LogType {
  LogTypeUint8 = 0x01,
  LogTypeUint16 = 0x02, 
  LogTypeUint32 = 0x03, 
  LogTypeInt8 = 0x04,
  LogTypeInt16 = 0x05,
  LogTypeInt32 = 0x06,
  LogTypeFloat = 0x07,
  LogTypeFP16 = 0x08,
};

struct LogTocEntry : public TocEntry {
  uint16_t id;
  std::string group;
  std::string name;
  LogType type;


  float fp16_to_float(uint16_t fp16) const;

  uint8_t size() const;
  float to_float(const std::vector<uint8_t>& data) const;

  LogTocEntry();
  LogTocEntry(const std::vector<uint8_t>& data);
  LogTocEntry(const std::string& line);
  std::string toString() const ;
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

class LoggingPacker : public TocPacker {
public:
 

  LoggingPacker(); // Constructor

  CrtpRequest create_block(uint8_t index, const std::vector<std::pair<uint8_t, uint16_t>>& content);
  CrtpRequest start_block(uint8_t index, uint8_t period);
  CrtpRequest stop_block(uint8_t index);


private:
  std::vector<uint8_t> _create_block_content(const std::vector<std::pair<uint8_t, uint16_t>>& content);

protected:
    CrtpPacket prepare_control_packet(const std::vector<uint8_t>& data);
};
