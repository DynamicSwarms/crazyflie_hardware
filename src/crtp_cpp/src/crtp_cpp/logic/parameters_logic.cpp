#include "crtp_cpp/logic/parameters_logic.hpp"
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
    std::memcpy(&id, data.data() + 1, 2);   // Two bytes of ident
    uint8_t type_info = data[3];                  // One byte of typeInfo    
    group  = std::string(reinterpret_cast<const char*>(data.data() + 4)); // std::string will read unil \0 terminated 
    name = std::string(reinterpret_cast<const char*>(data.data() + 4 + group.size() + 1));

    type = (ParamType)(type_info & 0x0F);         // 4 Bits of type (2len, int/float, unsigned/signed)
    readonly = type_info & (0x00 | (1 <<  6));    // 1 Bit if ReadOnly
}

std::string ParamTocEntry::toString() const {
    std::ostringstream ss;
    ss << id << "," <<  type << "," << readonly << ","  << group << "." << name;
    return ss.str();
}


ParametersLogic::ParametersLogic(CrtpLink* crtp_link, const std::string& path)
    : TocLogic<ParamTocEntry>(crtp_link, path, PORT_PARAMETER),
      packer(ParametersPacker()) {}








void ParametersLogic::send_set_parameter(const std::string& group, const std::string& name, double value) {
    /**
    auto toc_element = toc.get_element(group, name);

    if (toc_element == nullptr) {
        throw std::runtime_error("Parameter not found: " + group + "." + name);
    }

    uint16_t id = toc_element->ident;
    int value_nr;
    char pytype = toc_element->pytype;

    if (pytype == 'f' || pytype == 'd') {
        value_nr = static_cast<int>(value); // Assuming value is a double, casting to int
    } else {
        value_nr = static_cast<int>(value);
    }

    auto packet = packer.set_parameter(id, pytype, value_nr);
    link->send_packet_no_response(&packet);
    */
}