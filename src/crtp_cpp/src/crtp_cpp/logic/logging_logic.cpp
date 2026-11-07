#include "crtp_cpp/logic/logging_logic.hpp"
#include <tuple>
#include <utility>

#define PORT_LOGGING 5

LoggingLogic::LoggingLogic(
    std::shared_ptr<CrtpLink> crtp_link,
    const std::string& path
) : TocLogic<LogTocEntry>(crtp_link, path, PORT_LOGGING),
    packer(LoggingPacker()) 
{
    link->add_callback(PORT_LOGGING, std::bind(&LoggingLogic::crtp_response_callback, this, std::placeholders::_1));
}

bool LoggingLogic::start_block(int id, int period_ms_d10) {
    return link->send_packet(packer.start_block(id, period_ms_d10)).has_value();
}

bool LoggingLogic::stop_block(int id) {
    return link->send_packet(packer.stop_block(id)).has_value();
}

bool LoggingLogic::remove_block(int id) {
    if (!stop_block(id)) return false;
    
    std::lock_guard<std::mutex> lock(m_mutex);
    m_blocks.erase(id);
    return true;
}

bool LoggingLogic::reset() {
    if (!link->send_packet(packer.reset())) return false;

    std::lock_guard<std::mutex> lock(m_mutex);
    m_blocks.clear();
    return true;
}

bool
LoggingLogic::add_block(int id, const std::vector<std::string>& variables, DataCallback callback) {
    std::vector<std::pair<uint8_t, uint16_t>> vars;
    auto block_entries = std::vector<LogTocEntry>();

    for (const auto& variable_name : variables) {
        size_t dot = variable_name.find('.');

        if (dot != std::string::npos) {
            std::string group = variable_name.substr(0, dot);
            std::string name = variable_name.substr(dot + 1);

            bool found = false;
            for (const auto& entry : LoggingLogic::toc_entries) {
                if (entry.group == group && entry.name == name) {
                    vars.push_back({entry.type, entry.id});
                    block_entries.push_back(entry);
                    found = true;
                    break;
                }
            } 
            if (!found) {
                return false; // "Error: Variable not found in TOC"
            }
        }
    }    
    
    if (!link->send_packet(packer.create_block(id, vars))) return false;
    std::lock_guard<std::mutex> lock(m_mutex);
    m_blocks[id] = Block{std::move(block_entries), std::move(callback)};
    return true;
}

std::vector<double> LoggingLogic::unpack_block(int block_id, const std::vector<uint8_t>& data) {
    std::vector<LogTocEntry> variables;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto block = m_blocks.find(block_id);
        if (block == m_blocks.end()) return {};
        variables = block->second.variables;
    }
    return unpack(variables, data);
}

std::vector<double> LoggingLogic::unpack(const std::vector<LogTocEntry> &variables,
    const std::vector<uint8_t> &data)
{
    std::vector<double> unpacked_data;

    // Unpack data using struct-like functionality
    uint8_t offset = 0;
    for (const auto& variable : variables) {
        uint8_t entry_size = variable.size();
        if (data.size() < offset + entry_size) {
            return {}; // "Error: Not enough data to unpack block" 
        }
        std::vector<uint8_t> entry_data(data.begin() + offset, data.begin() + offset + entry_size);
        unpacked_data.push_back(variable.to_double(entry_data));
        offset += entry_size;
    }
    return unpacked_data;
}

void LoggingLogic::crtp_response_callback(const CrtpPacket &packet)
{
    if (packet.channel != LOGDATA_CHANNEL || packet.data_length < 4 ||
        packet.data_length > sizeof(packet.data)) return;
        
    std::lock_guard<std::mutex> lock(m_mutex);

    auto it = m_blocks.find(packet.data[0]);
    if (it == m_blocks.end()) return;
    
    const auto &block = it->second;
    if (!block.callback) return;
    
    const uint32_t timestamp = static_cast<uint32_t>(packet.data[1]) |
        (static_cast<uint32_t>(packet.data[2]) << 8) |
        (static_cast<uint32_t>(packet.data[3]) << 16);
    const std::vector<uint8_t> payload(packet.data + 4, packet.data + packet.data_length);
    auto values = unpack(block.variables, payload);
    if (values.size() == block.variables.size()) block.callback(values, timestamp);
}
