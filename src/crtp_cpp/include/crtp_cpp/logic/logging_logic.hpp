#pragma once

#include <vector>
#include <map>
#include <string>
#include <tuple>
#include <memory> // for std::unique_ptr
#include <cstring>
#include "crtp_cpp/link/crtp_link.hpp"
#include "crtp_cpp/packer/crtp_packer.hpp"
#include "crtp_cpp/packer/logging_packer.hpp"
#include "crtp_cpp/logic/toc_logic.hpp"     


class LoggingLogic : public TocLogic<LogTocEntry> {
public:
    LoggingLogic(CrtpLink * crtp_link, const std::string& path);

    std::vector<float> unpack_block(int block_id, const std::vector<uint8_t>& data);
    void start_block(int id, int period_ms_d10);
    void stop_block(int id);
    void add_block(int id, const std::vector<std::string>& variables);

private:
    LoggingPacker packer;
    std::map<int, std::vector<LogTocEntry>> blocks; // Map for blocks
};
