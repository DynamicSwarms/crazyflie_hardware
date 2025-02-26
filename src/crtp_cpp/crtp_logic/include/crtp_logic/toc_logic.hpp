#pragma once

#include "crtp_link/crtp_link.hpp"
#include "crtp_logic/logic.hpp"
#include "crtp_packer/toc_packer.hpp"
//#include "crtp_logic/toc/toc.hpp"
//#include "crtp_logic/toc/toccache.hpp"
#include <vector>
#include <functional>
#include <tuple>
#include <optional>

struct TocEntry {
    public:
        virtual std::string stringify();
        virtual TocEntry from_string(std::string& str);

    private: 
        std::vector<TocEntry> toc_entries;
}

/**
 * @brief Logic for Table of Contents (TOC) related communication.
 */
class TocLogic : public Logic {
public:
    /**
     * @brief Constructor for TocLogic.
     * @param packer The TocPacker object.
     * @param crtp_link A pointer to the CrtpLink object.
     * @param ElementType A function that creates TocElement objects.
     * @param path Path to the TOC cache file.
     */
    TocLogic(TocPacker packer, CrtpLink* crtp_link, std::function<TocElement*(uint16_t, const std::vector<uint8_t>&)> ElementType, const std::string& path);

    /**
     * @brief Initializes the TOC by fetching or downloading items.
     */
    void initialize_toc();

    /**
     * @brief Sends a request to get TOC information.
     * @return A pair containing the number of items and CRC.
     */
    std::pair<uint16_t, uint32_t> send_get_toc_info();

    /**
     * @brief Sends requests to download all TOC items.
     */
    void send_download_toc_items();


private: 
    virtual TocEntry _to_toc_item(const std::vector<uint8_t>& data);

protected:
    TocPacker packer;
    std::function<TocElement*(uint16_t, const std::vector<uint8_t>&)> ElementType;
    Toc toc;
    TocCache toc_cache;
    std::optional<uint16_t> nbr_of_items;
    std::optional<uint32_t> crc;

private:
    void _to_toc_item(const std::vector<uint8_t>& data);
};