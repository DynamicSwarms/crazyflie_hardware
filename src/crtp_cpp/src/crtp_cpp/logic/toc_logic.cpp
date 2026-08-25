#include "crtp_cpp/logic/toc_logic.hpp"
#include <stdexcept>
#include <cstring>
#include <filesystem>
#include <fstream>

#include "crtp_cpp/logic/parameters_logic.hpp"
#include "crtp_cpp/logic/logging_logic.hpp"

template <class T>
TocLogic<T>::TocLogic(std::shared_ptr<CrtpLink>crtp_link, const std::string &path, uint8_t port)
    : Logic(crtp_link)
    , packer(TocPacker(port))
    , toc_cache_path(path) {}

template <class T>
bool TocLogic<T>::load_from_file(uint32_t crc)
{
    const std::filesystem::path file_name =
        std::filesystem::path(toc_cache_path) / (std::to_string(crc) + ".csv");
    std::ifstream infile(file_name);
    if (!infile.good())
        return false;

    toc_entries.clear();
    std::string line;
    while (std::getline(infile, line))
    {
        toc_entries.push_back(T(line));
    }
    return true;
}

template <class T>
void TocLogic<T>::write_to_file()
{
    if (!nbr_of_items.has_value() || !crc.has_value())
    {
        if (!send_download_toc_items()) return;
    }

    const std::filesystem::path cache_directory(toc_cache_path);
    std::error_code error;
    std::filesystem::create_directories(cache_directory, error);
    if (error) return;

    const std::filesystem::path file_name =
        cache_directory / (std::to_string(crc.value()) + ".csv");
    const std::filesystem::path temporary_file_name = file_name.string() + ".tmp";
    std::ofstream output(temporary_file_name);
    if (!output.good()) return;
    for (const auto &entry : toc_entries)
    {
        output << entry.toString() << std::endl;
    }
    output.close();
    if (!output.good()) return;

    std::filesystem::rename(temporary_file_name, file_name, error);
}

template <class T>
bool TocLogic<T>::initialize_toc()
{
    auto [nbr_of_items, crc] = send_get_toc_info();
    if (!this->nbr_of_items || !this->crc) return false;
    bool cached = load_from_file(crc);
    if (!cached)
    {
        if (!send_download_toc_items()) return false;
        write_to_file();
    }
    return true;
}

template <class T>
std::pair<uint16_t, uint32_t> TocLogic<T>::send_get_toc_info()
{
    CrtpRequest request = packer.get_toc_info();
    auto response = link->send_packet(request);

    uint16_t nbr_of_items;
    uint32_t crc;
    if (!response || response.value().data_length < 7)
    {
        this->nbr_of_items.reset();
        this->crc.reset();
        return {0, 0};
    }
    else
    {
        std::memcpy(&nbr_of_items, response.value().data + 1, sizeof(uint16_t));
        std::memcpy(&crc, response.value().data + 3, sizeof(uint32_t));
    }

    this->nbr_of_items = nbr_of_items;
    this->crc = crc;
    return {nbr_of_items, crc};
}

template <class T>
bool TocLogic<T>::send_download_toc_items()
{
    if (!nbr_of_items.has_value() || !crc.has_value())
    {
        send_get_toc_info();
        if (!nbr_of_items.has_value() || !crc.has_value()) return false;
    }

    std::vector<CrtpRequest> requests;
    for (uint16_t i = 0; i < nbr_of_items.value(); ++i)
    {
        requests.push_back(packer.get_toc_item(i));
    }

    toc_entries.clear();
    auto responses = link->send_batch_request(requests);
    if (responses.size() != requests.size()) return false;
    for (const auto &packet : responses)
    {
        std::vector<uint8_t> data(packet.data, packet.data + packet.data_length);
        toc_entries.push_back(T(data));
    }
    return true;
}

template class TocLogic<ParamTocEntry>;
template class TocLogic<LogTocEntry>;
