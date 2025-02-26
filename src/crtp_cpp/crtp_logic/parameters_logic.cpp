#include "crtp_logic/parameters_logic.hpp"
#include <stdexcept>

ParametersLogic::ParametersLogic(CrtpLink* crtp_link, const std::string& path)
    : TocLogic(ParametersPacker(), crtp_link, ParamTocElement, path),
      packer(ParametersPacker()) {}

ParamTocEntry ParametersLogic::_to_toc_item(const std::vector<uint8_t>& data)
{
    ParamTocEntry entry;
    std::memcpy(&entry.id, data.data() + 1, 2);   // Two bytes of ident
    uint8_t type_info = data[3];                  // One byte of typeInfo    
    entry.group  = std::string(&data.data() + 4); // std::string will read unil \0 terminated 
    entry.name = std::string(&data.data() + 4 + entry.group.size() + 1);

    entry.type = (ParamType)(type_info & 0x0F);         // 4 Bits of type (2len, int/float, unsigned/signed)
    entry.readonly = type_info & (0x00 | (1 <<  6));    // 1 Bit if ReadOnly
}

void add_toc_item_from_data(const std::vector<uint8_t>& data)
{

}

void ParametersLogic::send_set_parameter(const std::string& group, const std::string& name, double value) {
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
}