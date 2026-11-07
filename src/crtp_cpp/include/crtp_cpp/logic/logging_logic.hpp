#pragma once

#include <vector>
#include <mutex>
#include <map>
#include <string>
#include <tuple>
#include <memory> // for std::unique_ptr
#include <cstring>
#include "crtp_cpp/link/crtp_link.hpp"
#include "crtp_cpp/packer/crtp_packer.hpp"
#include "crtp_cpp/packer/logging_packer.hpp"
#include "crtp_cpp/logic/toc_logic.hpp"     
#include "crtp_cpp/logic/logtocentry.hpp"

// The owner must stop packet dispatch before destroying this object.
class LoggingLogic : public TocLogic<LogTocEntry> {
public:
    using DataCallback = std::function<void(const std::vector<double> &, uint32_t)>;

    LoggingLogic(std::shared_ptr<CrtpLink> crtp_link, const std::string& path);


    bool start_block(int id, int period_ms_d10);
    bool stop_block(int id);
    bool remove_block(int id);
    bool add_block(int id, const std::vector<std::string>& variables, DataCallback callback = {});
    bool reset();

    std::vector<double> unpack_block(int block_id, const std::vector<uint8_t>& data);
private:
    struct Block
    {
        std::vector<LogTocEntry> variables;
        DataCallback callback;
    };
    void crtp_response_callback(const CrtpPacket &packet);
    static std::vector<double> unpack(const std::vector<LogTocEntry> &variables,
        const std::vector<uint8_t> &data);


private:
    LoggingPacker packer;
    std::mutex m_mutex;
    std::map<int, Block> m_blocks;
};
