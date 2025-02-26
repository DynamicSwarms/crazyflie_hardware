#pragma once

#include <vector>
#include <map>
#include <string>
#include <tuple>
#include <memory> // for std::unique_ptr
#include "crtp_link/crtp_link.hpp"
#include "crtp_packer/crtp_packer.hpp"
#include "crtp_packer/logging_packer.hpp"
//#include "log_toc_element.hpp" // Include your LogTocElement header
#include "crtp_logic/toc_logic.hpp"     

class LoggingLogic : public TocLogic {
public:
    LoggingLogic(
        std::function<std::unique_ptr<CrtpPacker>(int)> crtp_packer_factory,
        std::unique_ptr<CrtpLink> crtp_link,
        const std::string& path
    );

    std::vector<float> unpack_block(int block_id, const std::vector<uint8_t>& data);
    void start_block(int id, int period_ms_d10);
    void stop_block(int id);
    void add_block(int id, const std::vector<std::string>& variables);

private:
    std::unique_ptr<LoggingPacker> __packer;
    std::map<int, std::tuple<std::string, int>> blocks; // Map for blocks
};
