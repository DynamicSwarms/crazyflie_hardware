#include "crtp_cpp/logic/toc_logic.hpp"
#include <stdexcept>
#include <cstring>
#include <fstream>

#include "crtp_cpp/logic/parameters_logic.hpp"
#include "crtp_cpp/logic/logging_logic.hpp"


template <class T>
TocLogic<T>::TocLogic(CrtpLink* crtp_link, const std::string& path, uint8_t port)
    : Logic(crtp_link)
    , toc_cache_path(path)
    , packer(TocPacker(port)) {}

template <class T>
bool TocLogic<T>::load_from_file(uint32_t crc) {
    
    return false;
}

template <class T>
void TocLogic<T>::write_to_file(uint32_t crc)
{
    std::string fileName = "winni_log" + std::to_string(crc) + ".csv";
    std::string fileNameTemp = fileName + ".tmp";
    std::ofstream output(fileNameTemp);
    for (const auto& entry : toc_entries) {
        output << entry.toString() << std::endl;
    }
    // change the filename
    rename(fileNameTemp.c_str(), fileName.c_str());

}

template <class T>
void TocLogic<T>::initialize_toc() {
    auto [nbr_of_items, crc] = send_get_toc_info();
    bool cached = load_from_file(crc);
    if (!cached) {
        send_download_toc_items();
        write_to_file(crc);
    }
}

template <class T>
std::pair<uint16_t, uint32_t> TocLogic<T>::send_get_toc_info() {
    CrtpRequest request = packer.get_toc_info();
    auto response = link->send_packet(request);

    if (!response || response.value().data_length < 7) {
        throw std::runtime_error("Invalid TOC info response");
    }

    uint16_t nbr_of_items;
    uint32_t crc;
    std::memcpy(&nbr_of_items, response.value().data + 1, sizeof(uint16_t));
    std::memcpy(&crc, response.value().data + 3, sizeof(uint32_t));

    this->nbr_of_items = nbr_of_items;
    this->crc = crc;
    return {nbr_of_items, crc};
}

template <class T>
void TocLogic<T>::send_download_toc_items() {
    if (!nbr_of_items.has_value() || !crc.has_value()) {
        send_get_toc_info();
    }

    std::vector<CrtpRequest> requests;
    for (uint16_t i = 0; i < nbr_of_items.value(); ++i) {
        requests.push_back(packer.get_toc_item(i));
    }

    toc_entries.clear();
    auto responses = link->send_batch_request(requests);
    for (const auto& packet : responses) {
        std::vector<uint8_t> data(packet.data, packet.data + packet.data_length);
        toc_entries.push_back(T(data));
    }
}


template class TocLogic<ParamTocEntry>;
template class TocLogic<LogTocEntry>;