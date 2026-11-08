#include "crazyradio/radio_diagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace crazyradio {

RadioDiagnostics::RadioDiagnostics(DiagnosticsNodeInterfaces interfaces, uint8_t channel, bool useUDPRadio)
    : m_interfaces(std::move(interfaces))
    , m_channel(channel)
    , m_useUDPRadio(useUDPRadio)
    , m_deliveryWarnMs(100.0)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Outgoing delivery delay warning threshold in milliseconds; 0 disables";
    descriptor.read_only = true;
    m_deliveryWarnMs = m_interfaces.parameters->declare_parameter(
        "diagnostics_delivery_warn_ms", rclcpp::ParameterValue(100.0), descriptor).get<double>();
    if (!std::isfinite(m_deliveryWarnMs) || m_deliveryWarnMs < 0.0) {
        throw std::invalid_argument("diagnostics_delivery_warn_ms must be finite and nonnegative");
    }
    m_diagnosticsPub = rclcpp::create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        m_interfaces.parameters, m_interfaces.topics, "/diagnostics", rclcpp::QoS(10));
    m_linkQualityPub = rclcpp::create_publisher<crazyflie_interfaces::msg::CrazyflieLinkQualityArray>(
        m_interfaces.parameters, m_interfaces.topics, "crazyradio/link_qualities", rclcpp::QoS(10));
}

void RadioDiagnostics::recordCommunication(
    const libcrtp::CrtpPacket& packet, bool success, bool isPortPacket, bool isBroadcast)
{
    std::lock_guard<std::mutex> lock(m_statisticsMutex);
    ++m_statistics.attempts;
    m_statistics.failedAttempts += !success;
    m_statistics.payloadAttempts += isPortPacket;
    m_statistics.pollAttempts += !isPortPacket;
    m_statistics.broadcastAttempts += isBroadcast;
    if (success && isPortPacket) {
        const double delayMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - packet.createdAt).count();
        m_maxCompletedDeliveryDelayMs = std::max(m_maxCompletedDeliveryDelayMs, delayMs);
    }
}

void RadioDiagnostics::publish(
    const libcrtp::CrtpContainerStats& container,
    const crazyflie_interfaces::msg::CrazyflieLinkQualityArray& linkQualities)
{
    std::lock_guard<std::mutex> publishLock(m_publishMutex);
    m_linkQualityPub->publish(linkQualities);

    CommunicationStats current;
    double maxCompletedDeliveryDelayMs;
    {
        std::lock_guard<std::mutex> lock(m_statisticsMutex);
        current = m_statistics;
        maxCompletedDeliveryDelayMs = m_maxCompletedDeliveryDelayMs;
        m_maxCompletedDeliveryDelayMs = 0.0;
    }
    const auto now = std::chrono::steady_clock::now();
    const double oldestOutgoingAgeMs = container.oldestOutgoingCreatedAt ?
        std::chrono::duration<double, std::milli>(now - *container.oldestOutgoingCreatedAt).count() : 0.0;
    const double seconds = std::chrono::duration<double>(now - m_lastDiagnosticsTime).count();
    const auto attempts = current.attempts - m_previousStatistics.attempts;
    const auto failures = current.failedAttempts - m_previousStatistics.failedAttempts;
    std::vector<diagnostic_msgs::msg::KeyValue> values;
    const auto add = [&values](const std::string& key, const auto& value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(2) << value;
        item.value = stream.str();
        values.push_back(std::move(item));
    };
    // Identity and connected workload.
    add("Channel", static_cast<unsigned>(m_channel));
    add("Transport", m_useUDPRadio ? "UDP / SITL" : "USB / Crazyradio");
    add("Unicast links", container.unicastLinks);
    add("Broadcast links", container.broadcastLinks);

    double worstLinkQuality = 1.0;
    if (linkQualities.link_qualities.empty()) {
        add("Average link quality (%)", "N/A");
        add("Worst link quality (%)", "N/A");
        add("Best link quality (%)", "N/A");
    } else {
        double total = 0.0;
        double worst = linkQualities.link_qualities.front().link_quality;
        double best = worst;
        for (const auto& quality : linkQualities.link_qualities) {
            total += quality.link_quality;
            worst = std::min(worst, static_cast<double>(quality.link_quality));
            best = std::max(best, static_cast<double>(quality.link_quality));
        }
        worstLinkQuality = worst;
        add("Average link quality (%)", 100.0 * total / linkQualities.link_qualities.size());
        add("Worst link quality (%)", 100.0 * worst);
        add("Best link quality (%)", 100.0 * best);
    }

    // Delay and backlog: the most useful indicators when diagnosing congestion.
    add("Oldest outgoing packet age (ms)", oldestOutgoingAgeMs);
    add("Maximum completed delivery delay in interval (ms)", maxCompletedDeliveryDelayMs);
    add("Outgoing packets", container.outgoingPackets);
    add("Pending responses (may overlap outgoing)", container.pendingResponses);
    add("Links in backoff", container.linksInBackoff);

    // Rates count software transport calls, not individual hardware retries.
    // uint64_t subtraction remains correct across counter wrap between samples.
    add("Send attempts / s", attempts / seconds);
    add("Enqueued packets / s", (container.enqueuedPackets - m_previousEnqueuedPackets) / seconds);
    add("Payload send attempts / s",
        (current.payloadAttempts - m_previousStatistics.payloadAttempts) / seconds);
    add("Polling send attempts / s",
        (current.pollAttempts - m_previousStatistics.pollAttempts) / seconds);
    add("Broadcast send attempts / s",
        (current.broadcastAttempts - m_previousStatistics.broadcastAttempts) / seconds);
    add("Failed send attempts / s", failures / seconds);

    // Lifetime counters and reporting configuration.
    add("Enqueued packets total", container.enqueuedPackets);
    add("Send attempts total", current.attempts);
    add("Failed send attempts total", current.failedAttempts);
    add("Delivery delay warning threshold (ms; 0 disables)", m_deliveryWarnMs);
    add("Measurement interval (s)", seconds);

    const bool idle = container.unicastLinks == 0 && container.broadcastLinks == 0;
    const bool delayed = m_deliveryWarnMs > 0.0 && std::max(oldestOutgoingAgeMs, maxCompletedDeliveryDelayMs) > m_deliveryWarnMs;
    const bool lowLinkQuality = worstLinkQuality < 0.5;

    std::string message = "";
    uint8_t level;
    if (!m_error.empty()) {
        level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        message = m_error;
    } else if (delayed || lowLinkQuality) {
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        if (delayed) message += "Outgoing delivery delayed; ";
        if (lowLinkQuality) message += "Low link quality; ";
    } else if (idle) {
        level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        message = "Running; no links";
    } else {
        level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        message = "Running";
    }
 
    
    publishRadioStatus(
        level, message,
        std::move(values));
    m_previousEnqueuedPackets = container.enqueuedPackets;
    m_previousStatistics = current;
    m_lastDiagnosticsTime = now;
}

void RadioDiagnostics::publishRadioStatus(
    uint8_t level, const std::string& message,
    std::vector<diagnostic_msgs::msg::KeyValue> values)
{
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = level;
    status.name = std::string(m_interfaces.base->get_fully_qualified_name()) + "/radio";
    // Channel identifies the configured radio endpoint, not a USB device serial number.
    status.hardware_id = std::string(m_useUDPRadio ? "udp/channel/" : "crazyradio/channel/") +
        std::to_string(m_channel);
    status.message = message;
    status.values = std::move(values);
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = m_interfaces.clock->get_clock()->now();
    diagnostics.status.push_back(std::move(status));
    m_diagnosticsPub->publish(diagnostics);
}

void RadioDiagnostics::reportError(const std::string& message)
{
    std::lock_guard<std::mutex> lock(m_publishMutex);
    m_error = message;
    publishRadioStatus(diagnostic_msgs::msg::DiagnosticStatus::ERROR, m_error, {});
}

} // namespace crazyradio
