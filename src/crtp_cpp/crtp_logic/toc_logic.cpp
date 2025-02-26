#include "crtp_logic/toc_logic.hpp"
#include <stdexcept>
#include <cstring>

TocLogic::TocLogic(TocPacker packer, CrtpLink* crtp_link, std::function<TocElement*(uint16_t, const std::vector<uint8_t>&)> ElementType, const std::string& path)
    : Logic(crtp_link),
      packer(packer),
      ElementType(ElementType),
      toc(),
      toc_cache(path) {}

void TocLogic::_to_toc_item(const std::vector<uint8_t>& data) {
    if (data.size() < 3) {
        throw std::runtime_error("Invalid TOC item data size");
    }

    uint16_t ident;
    std::memcpy(&ident, data.data() + 1, sizeof(uint16_t));

    std::vector<uint8_t> element_data(data.begin() + 3, data.end());
    TocElement* element = ElementType(ident, element_data);
    toc.add_element(element);
}

void TocLogic::initialize_toc() {
    auto [nbr_of_items, crc] = send_get_toc_info();
    auto cache_data = toc_cache.fetch(crc);
    if (cache_data) {
        toc.toc = cache_data.value();
    } else {
        send_download_toc_items();
    }
}

std::pair<uint16_t, uint32_t> TocLogic::send_get_toc_info() {
    auto [packet, expects_response, matching_bytes] = packer.get_toc_info();
    auto resp = link->send_packet(packet, expects_response, matching_bytes);

    if (!resp.has_value() || resp.value().packet.data.size() < 7) {
        throw std::runtime_error("Invalid TOC info response");
    }

    uint16_t nbr_of_items;
    uint32_t crc;
    std::memcpy(&nbr_of_items, resp.value().packet.data.data() + 1, sizeof(uint16_t));
    std::memcpy(&crc, resp.value().packet.data.data() + 3, sizeof(uint32_t));

    this->nbr_of_items = nbr_of_items;
    this->crc = crc;
    return {nbr_of_items, crc};
}

void TocLogic::send_download_toc_items() {
    if (!nbr_of_items.has_value() || !crc.has_value()) {
        send_get_toc_info();
    }

    std::vector<std::tuple<CrtpPacket, bool, std::vector<uint8_t>>> packets;
    for (uint16_t i = 0; i < nbr_of_items.value(); ++i) {
        packets.push_back(packer.get_toc_item(i));
    }

    auto responses = link->send_batch_request(packets);
    for (const auto& result : responses) {
        _to_toc_item({result.packet.data.begin(), result.packet.data.begin() + result.packet.data_length});
    }

    toc_cache.insert(crc.value(), toc.toc);
}
