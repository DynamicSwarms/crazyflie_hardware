#include "crtp_cpp/logic/parameters_logic.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <sstream>
#include <cstring>

#define PORT_PARAMETER 0x02

// Constructor from comma-separated string
ParamTocEntry::ParamTocEntry(const std::string& csv) {
    std::istringstream lineStream(csv);
    std::string token;
    // Parse ID
    std::getline(lineStream, token, ',');
    id = static_cast<uint16_t>(std::stoi(token));
    // Parse Type
    std::getline(lineStream, token, ',');
    type = (ParamType)static_cast<uint8_t>(std::stoi(token));
    // Parse Readonly
    std::getline(lineStream, token, ',');
    readonly = token == "1";
    // Parse Group
    std::getline(lineStream, group, '.');
    // Parse Name
    std::getline(lineStream, name, ',');  
}

ParamTocEntry::ParamTocEntry(const std::vector<uint8_t>& data)
{
    std::memcpy(&id, data.data() + 1, 2);         // Two bytes of ident
    uint8_t type_info = data[3];                  // One byte of typeInfo    
    group  = std::string(reinterpret_cast<const char*>(data.data() + 4)); // std::string will read unil \0 terminated 
    name = std::string(reinterpret_cast<const char*>(data.data() + 4 + group.size() + 1));

    type = (ParamType)(type_info & 0x0F);         // 4 Bits of type (2len, int/float, unsigned/signed)
    readonly = type_info & (0x00 | (1 <<  6));    // 1 Bit if ReadOnly
}

std::string ParamTocEntry::toString() const 
{
    std::ostringstream ss;
    ss << id << "," <<  type << "," << readonly << ","  << group << "." << name;
    return ss.str();
}

bool ParamTocEntry::isInteger() const 
{
    return (type & 0x04) == 0; // If IsNonInt bit (3rd bit) is 0, it's an integer
}

bool ParamTocEntry::isDouble() const 
{
    return (type & 0x04) != 0; // If IsNonInt bit (3rd bit) is 1, it's a float
}


ParametersLogic::ParametersLogic(CrtpLink* crtp_link, const std::string& path)
    : TocLogic<ParamTocEntry>(crtp_link, path, PORT_PARAMETER),
      packer(ParametersPacker()) {}


bool ParametersLogic::send_set_parameter(const std::string& group, const std::string& name, std::variant<int, double> value) {
    for (const auto& entry : ParametersLogic::toc_entries) {
        if (entry.group == group && entry.name == name) {
            CrtpRequest request;
            request.packet = packer.set_parameter(entry.id, entry.type, value);
            link->send_packet_no_response(request);
            return true;
        }
    }
    return false;
}

std::optional<std::variant<int64_t, double>> ParametersLogic::send_get_parameter(
    const std::string& group, const std::string& name) {
    for (const auto& entry : ParametersLogic::toc_entries) {
        if (entry.group != group || entry.name != name) {
            continue;
        }

        auto response = link->send_packet(packer.get_parameter(entry.id));
        // Protocol v2 read response: [ID_L, ID_H, STATUS, VALUE...].
        if (!response || response->data_length < 3 || response->data[2] != 0) {
            return std::nullopt;
        }

        const auto *data = response->data + 3;
        const auto value_size = static_cast<size_t>(response->data_length - 3);

        switch (entry.type) {
            case ParamTypeUint8:
                if (value_size < sizeof(uint8_t)) return std::nullopt;
                return static_cast<int64_t>(*data);
            case ParamTypeInt8: {
                if (value_size < sizeof(int8_t)) return std::nullopt;
                int8_t value;
                std::memcpy(&value, data, sizeof(value));
                return static_cast<int64_t>(value);
            }
            case ParamTypeUint16: {
                if (value_size < sizeof(uint16_t)) return std::nullopt;
                uint16_t value;
                std::memcpy(&value, data, sizeof(value));
                return static_cast<int64_t>(value);
            }
            case ParamTypeInt16: {
                if (value_size < sizeof(int16_t)) return std::nullopt;
                int16_t value;
                std::memcpy(&value, data, sizeof(value));
                return static_cast<int64_t>(value);
            }
            case ParamTypeUint32: {
                if (value_size < sizeof(uint32_t)) return std::nullopt;
                uint32_t value;
                std::memcpy(&value, data, sizeof(value));
                return static_cast<int64_t>(value);
            }
            case ParamTypeInt32: {
                if (value_size < sizeof(int32_t)) return std::nullopt;
                int32_t value;
                std::memcpy(&value, data, sizeof(value));
                return static_cast<int64_t>(value);
            }
            case ParamTypeUint64: {
                if (value_size < sizeof(uint64_t)) return std::nullopt;
                uint64_t value;
                std::memcpy(&value, data, sizeof(value));
                return static_cast<int64_t>(value);
            }
            case ParamTypeInt64: {
                if (value_size < sizeof(int64_t)) return std::nullopt;
                int64_t value;
                std::memcpy(&value, data, sizeof(value));
                return value;
            }
            case ParamTypeFP16: {
                if (value_size < sizeof(uint16_t)) return std::nullopt;
                uint16_t bits;
                std::memcpy(&bits, data, sizeof(bits));
                const int sign = (bits & 0x8000) ? -1 : 1;
                const int exponent = (bits >> 10) & 0x1f;
                const int mantissa = bits & 0x3ff;
                double value;
                if (exponent == 0) {
                    value = std::ldexp(static_cast<double>(mantissa), -24);
                } else if (exponent == 31) {
                    value = mantissa ? std::numeric_limits<double>::quiet_NaN()
                                     : std::numeric_limits<double>::infinity();
                } else {
                    value = std::ldexp(static_cast<double>(mantissa + 1024), exponent - 25);
                }
                return sign * value;
            }
            case ParamTypeFloat: {
                if (value_size < sizeof(float)) return std::nullopt;
                float value;
                std::memcpy(&value, data, sizeof(value));
                return static_cast<double>(value);
            }
            case ParamTypeDouble: {
                if (value_size < sizeof(double)) return std::nullopt;
                double value;
                std::memcpy(&value, data, sizeof(value));
                return value;
            }
        }
    }
    return std::nullopt;
}
