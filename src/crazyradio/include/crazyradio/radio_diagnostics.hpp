#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "crazyflie_interfaces/msg/crazyflie_link_quality_array.hpp"
#include "libcrtp/CrtpContainerStats.hpp"
#include "libcrtp/CrtpPacket.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"

namespace crazyradio {

struct DiagnosticsNodeInterfaces
{
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base;
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock;
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters;
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics;
};

struct LinkStatistics
{
    uint8_t id;
    double crtpQuality;
    double radioQuality;
    std::optional<double> rssiDbm;
};

class RadioDiagnostics
{
public:
    RadioDiagnostics(DiagnosticsNodeInterfaces interfaces, uint8_t channel, bool useUDPRadio);

    void recordCommunication(const libcrtp::CrtpPacket& packet, bool success, bool isPortPacket, bool isBroadcast);
    void publish(const libcrtp::CrtpContainerStats& container,
                 const std::vector<LinkStatistics>& links);
    void reportError(const std::string& message);

private:
    void publishLinkQualities(const std::vector<LinkStatistics>& links);
    void publishRadioStatus(uint8_t level, const std::string& message,
                            std::vector<diagnostic_msgs::msg::KeyValue> values);

    struct CommunicationStats
    {
        uint64_t attempts{0};
        uint64_t failedAttempts{0};
        uint64_t payloadAttempts{0};
        uint64_t pollAttempts{0};
        uint64_t broadcastAttempts{0};
    };

    DiagnosticsNodeInterfaces m_interfaces;
    uint8_t m_channel;
    bool m_useUDPRadio;
    double m_deliveryWarnMs;
    std::mutex m_statisticsMutex;
    CommunicationStats m_statistics;
    CommunicationStats m_previousStatistics;
    double m_maxCompletedDeliveryDelayMs{0.0};
    uint64_t m_previousEnqueuedPackets{0};
    std::chrono::steady_clock::time_point m_lastDiagnosticsTime{std::chrono::steady_clock::now()};
    // Serialize periodic publication and immediate errors; retain a fatal error until shutdown.
    std::mutex m_publishMutex;
    std::string m_error;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr m_diagnosticsPub;
    rclcpp::Publisher<crazyflie_interfaces::msg::CrazyflieLinkQualityArray>::SharedPtr m_linkQualityPub;
};

} // namespace crazyradio
