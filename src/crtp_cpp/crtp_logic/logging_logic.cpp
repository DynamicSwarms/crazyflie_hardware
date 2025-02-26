#include "crtp_logic/logging_logic.hpp"
#include <tuple>
#include <iostream> // For debugging

LoggingLogic::LoggingLogic(
    std::function<std::unique_ptr<CrtpPacker>(int)> crtp_packer_factory,
    std::unique_ptr<CrtpLink> crtp_link,
    const std::string& path
) : TocLogic(std::make_unique<LoggingPacker>(crtp_packer_factory), std::move(crtp_link), LogTocElement, path),
    __packer(static_cast<std::unique_ptr<LoggingPacker>&>(TocLogic::packer)) // Downcast and transfer ownership
{
}


std::vector<float> LoggingLogic::unpack_block(int block_id, const std::vector<uint8_t>& data) {
    if (blocks.find(block_id) == blocks.end()) { // Use find() for map lookup
        return {};
    }
    std::string unpack_string;
    int length;
    std::tie(unpack_string, length) = blocks.at(block_id); // Use at() for safe access

    if (data.size() < length) {
        std::cerr << "Error: Not enough data to unpack block " << block_id << std::endl;
        return {}; // Or throw an exception
    }
    std::vector<float> unpacked_data;

    // Unpack data using struct-like functionality
    size_t offset = 0;
    for (char format_char : unpack_string) {
        if (format_char == '<') continue; // Skip endianness character
        size_t size;
        switch (format_char) {
            case 'b': size = 1; break; // signed char
            case 'B': size = 1; break; // unsigned char
            case 'h': size = 2; break; // short
            case 'H': size = 2; break; // unsigned short
            case 'i': size = 4; break; // int
            case 'I': size = 4; break; // unsigned int
            case 'f': size = 4; break; // float
            case 'd': size = 8; break; // double
            default:
                std::cerr << "Unsupported format character: " << format_char << std::endl;
                return {};
        }

        if (offset + size > data.size()) {
            std::cerr << "Error: Not enough data to unpack block " << block_id << std::endl;
            return {};
        }
        
        if (format_char == 'f') { // Handle float specifically
            float value;
            std::memcpy(&value, &data[offset], size);
            unpacked_data.push_back(value);
        } else {
            // ... (Handle other types similarly using memcpy and type casting) ...
            // This would require more specific code depending on the types used.
            std::cerr << "Warning: Unpacking of types other than float is not implemented yet." << std::endl;
            return {};
        }

        offset += size;
    }

    return unpacked_data;
}

void LoggingLogic::start_block(int id, int period_ms_d10) {
    auto [packet, expects_response, matching] = __packer->start_block(id, period_ms_d10);
    link->send_packet_no_response(packet);
}

void LoggingLogic::stop_block(int id) {
    auto [packet, expects_response, matching] = __packer->stop_block(id);
    link->send_packet_no_response(packet);
}

void LoggingLogic::add_block(int id, const std::vector<std::string>& variables) {
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

    auto [packet, expects_response, matching] = __packer->create_block(id, elements);
    link->send_packet_no_response(packet);
}