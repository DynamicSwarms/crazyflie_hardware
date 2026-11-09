#include "crazyradio/crtp_logger.hpp"
#include "rclcpp/rclcpp.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <unistd.h>

void require(bool condition)
{
    if (!condition) throw std::runtime_error("radio statistics CSV regression");
}

std::vector<std::string> columns(const std::string& line)
{
    std::vector<std::string> result;
    std::istringstream stream(line);
    std::string value;
    while (std::getline(stream, value, ',')) result.push_back(value);
    if (!line.empty() && line.back() == ',') result.emplace_back();
    return result;
}

int main(int argc, char** argv)
{
    const auto directory = std::filesystem::temp_directory_path() /
        ("crazyradio-logger-test-" + std::to_string(getpid()));
    setenv("ROS_LOG_DIR", directory.c_str(), 1);
    rclcpp::init(argc, argv);
    const uint8_t received[] = {4, 0x23, 67, 0xF3};
    const uint8_t lost[] = {3, 0x10, 0};
    const uint8_t packet[] = {0xD1, 0xAB};
    for (int run = 0; run < 2; ++run) {
        crazyradio::CrtpLogger logger(true, 90);
        // Unicast ID 231 is legitimate: retain full address and ACK-enabled flag.
        logger.logRadioTransaction(90, 0xE7E7E7E7E7ULL, 2, true, packet, 2,
            libradio::crazyradio::decodeInlineAck(received, sizeof(received)),
            std::chrono::nanoseconds(123456789000));
        logger.logRadioTransaction(90, 0xE7E7E7E7E7ULL, 2, true, packet, 2,
            libradio::crazyradio::decodeInlineAck(lost, sizeof(lost)),
            std::chrono::nanoseconds(123456790000));
    }
    int files = 0;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(directory / "crazyradio")) {
        if (entry.path().extension() != ".csv") continue;
        ++files;
        std::ifstream stream(entry.path());
        std::string line;
        std::getline(stream, line);
        require(columns(line).size() == 23);
        std::getline(stream, line);
        const auto success = columns(line);
        require(success.size() == 23 && success[0] == "123456789");
        require(success[3] == "e7e7e7e7e7" && success[4] == "231" && success[6] == "1");
        require(success[10] == "1" && success[11] == "2" && success[13] == "-67");
        require(success[21] == "d1ab" && success[22] == "f3");
        std::getline(stream, line);
        const auto failure = columns(line);
        require(failure.size() == 23 && failure[10] == "0" && failure[13].empty());
    }
    require(files == 2); // Repeated launches must not overwrite each other's logs.
    rclcpp::shutdown();
    std::filesystem::remove_all(directory);
}
