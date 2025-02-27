#include "crtp_cpp/logic/logging_logic.hpp"
#include <tuple>
#include <iostream> // For debugging

#define PORT_LOGGING 5

LoggingLogic::LoggingLogic(
    CrtpLink * crtp_link,
    const std::string& path
) : TocLogic<LogTocEntry>(crtp_link, path, PORT_LOGGING),
    packer(LoggingPacker()) // Downcast and transfer ownership
{
}


std::vector<float> LoggingLogic::unpack_block(int block_id, const std::vector<uint8_t>& data) {
    if (blocks.find(block_id) == blocks.end()) { // Use find() for map lookup
        return {};
    }
    auto types = blocks.at(block_id); // Use at() for safe access

    std::vector<float> unpacked_data;

    // Unpack data using struct-like functionality
    uint8_t offset = 0;
    for (const auto& type : types) {
        uint8_t entry_size = type.size();
        if (data.size() < offset + entry_size) {
            std::cerr << "Error: Not enough data to unpack block " << block_id << std::endl;
            return {}; // Or throw an exception
        }
        std::vector<uint8_t> entry_data(data.begin() + offset, data.begin() + offset + entry_size);
        unpacked_data.push_back(type.to_float(entry_data));
        offset += entry_size;
    }

    return unpacked_data;
}

void LoggingLogic::start_block(int id, int period_ms_d10) {
    link->send_packet_no_response(packer.start_block(id, period_ms_d10));
}

void LoggingLogic::stop_block(int id) {
    link->send_packet_no_response(packer.stop_block(id));
}

void LoggingLogic::add_block(int id, const std::vector<std::string>& variables) {
    /**
    std::vector<std::pair<uint8_t, uint16_t>> elements;
    std::string unpack_string = "<";
    int total_bytelength = 0;

    for (const auto& variable_name : variables) {
        auto element = toc->get_element_by_complete_name(variable_name);
        if (!element) {
            std::cerr << "Error: Element not found for variable: " << variable_name << std::endl;
            return; // Or throw an exception
        }

        uint8_t type_id = LogTocElement::get_id_from_cstring(element->ctype);
        unpack_string += LogTocElement::get_unpack_string_from_id(type_id).substr(1); // Remove leading '<'
        total_bytelength += LogTocElement::get_size_from_id(type_id);
        elements.push_back({type_id, element->ident});
    }

    blocks[id] = {unpack_string, total_bytelength};

    auto [packet, expects_response, matching] = packer.create_block(id, elements);
    link->send_packet_no_response(packet);
    */
}