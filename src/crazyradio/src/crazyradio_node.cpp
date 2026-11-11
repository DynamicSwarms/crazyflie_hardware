#define BOOST_BIND_NO_PLACEHOLDERS
#include <chrono>
#include <atomic>
#include <string.h>
#include <sstream>
#include <map>
#include <thread>
#include <pthread.h>
#include <csignal>
#include <functional>
// Ros2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "crazyradio/Crazyradio.hpp"
#include "udpradio/Udpradio.hpp"
#include "interface/IRadio.hpp"
#include "libcrtp/CrtpLinkContainer.hpp"

#include "crtp_interfaces/srv/crtp_packet_send.hpp"
#include "crtp_interfaces/msg/crtp_packet_send_no_response.hpp"
#include "crtp_interfaces/msg/crtp_response.hpp"
#include "crazyflie_interfaces/msg/crazyflie_link_quality_array.hpp"
#include "crazyradio/radio_diagnostics.hpp"


#include <condition_variable>
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace {
static_assert(std::atomic<bool>::is_always_lock_free);
std::atomic<bool> shutdown_requested(false);

void signalHandler(int)
{
    shutdown_requested.store(true);
}
} // namespace

class CrazyradioNode : public rclcpp::Node
{
public:
    CrazyradioNode(const rclcpp::NodeOptions &options)
        : Node("crazyradio_cpp", options)
        , m_links()
        , m_radioPeriodUs(500) // 2000 Hz polling, realistic rate is ~ 1100Hz.
    {
        uint8_t channel = this->declare_parameter("channel", rclcpp::ParameterValue(80)).get<uint8_t>();

        bool useUDPRadio = this->declare_parameter("use_udpradio", rclcpp::ParameterValue(false)).get<bool>();
        bool log_enabled = this->declare_parameter("log_enabled", rclcpp::ParameterValue(true)).get<bool>();
        bool log_to_socket = this->declare_parameter("log_to_socket", false);

        if (!useUDPRadio) m_radio = std::make_unique<libradio::crazyradio::Crazyradio>(
            log_enabled, log_to_socket, channel, rclcpp::get_log_directory());
        else {
            m_radio = std::make_unique<libradio::udpradio::UDPRadio>(
                log_enabled, log_to_socket, channel, rclcpp::get_log_directory());
            m_radioPeriodUs = 100; // 10_000 Hz
        }

        
        m_radioPriority = this->declare_parameter("radio_priority", rclcpp::ParameterValue(20)).get<int>();

        auto qos = rclcpp::ServicesQoS().keep_all().reliable().durability_volatile();

        crtp_send_callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        send_crtp_packet_service = this->create_service<crtp_interfaces::srv::CrtpPacketSend>(
            "crazyradio/send_crtp_packet" + std::to_string(channel),
            std::bind(&CrazyradioNode::sendCrtpPacketCallback, this, _1, _2, _3),
            qos,
            crtp_send_callback_group);

        auto no_response_qos = rclcpp::QoS(rclcpp::KeepLast(1000))
            .reliable()
            .durability_volatile();
        auto no_response_options = rclcpp::SubscriptionOptions();
        no_response_options.callback_group = crtp_send_callback_group;
        send_crtp_packet_no_response_sub =
            this->create_subscription<crtp_interfaces::msg::CrtpPacketSendNoResponse>(
                "crazyradio/send_crtp_packet_no_response" + std::to_string(channel),
                no_response_qos,
                std::bind(&CrazyradioNode::sendCrtpPacketNoResponseCallback, this, _1),
                no_response_options);

        send_response_pub = this->create_publisher<crtp_interfaces::msg::CrtpResponse>(
            "crazyradio/crtp_response", 10);

        link_end_pub = this->create_publisher<crtp_interfaces::msg::CrtpLink>(
            "crazyradio/crtp_link_end", 10);

        link_close_sub = this->create_subscription<crtp_interfaces::msg::CrtpLink>(
            "crazyradio/close_crtp_link", 10,
            std::bind(&CrazyradioNode::closeLinkCallback, this, _1));
        
        m_diagnostics = std::make_unique<crazyradio::RadioDiagnostics>(
            crazyradio::DiagnosticsNodeInterfaces{
                get_node_base_interface(), get_node_clock_interface(),
                get_node_parameters_interface(), get_node_topics_interface()},
            channel, useUDPRadio);


        radio_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        if (useUDPRadio) {
            radio_timer = this->create_wall_timer(
                std::chrono::microseconds(m_radioPeriodUs),
                [this] {
                    radioCallback();
                    if (m_shutdownRequested.load()) radio_timer->cancel();
                },
                radio_callback_group);
        } else {
            m_radioThread = std::thread(
                &CrazyradioNode::radioLoop,
                this);
        }

        /**
         * Tick all links periodically.
         * This updates internal link states such as null packet frequency.
         */
        m_tick_timer = this->create_wall_timer(
            std::chrono::milliseconds(1),
            [this] {m_links.tickLinksMs(1); });

        m_diagnostics_timer = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&CrazyradioNode::diagnosticsCallback, this));

        if (useUDPRadio) {
            RCLCPP_WARN(this->get_logger(), "Started Crazyradio Node in SITL mode on Channel %d", channel);
        } else {
            RCLCPP_WARN(this->get_logger(), "Started Crazyradio Node on Channel %d", channel);
        }
    }

    ~CrazyradioNode() override
    {
        m_shutdownRequested = true;
        if (radio_timer) radio_timer->cancel();
        if (m_radioThread.joinable()) m_radioThread.join();
    }

    bool shutdownRequested() const
    {
        return m_shutdownRequested.load();
    }

    void shutdown()
    {
        if (m_stopping.exchange(true)) return;
        m_shutdownRequested = true;
        
        m_links.stopLinkCreation();
        RCLCPP_INFO(get_logger(), "Shutting down radio and failing pending requests.");

        if (radio_timer) radio_timer->cancel();
        m_tick_timer->cancel();
        m_diagnostics_timer->cancel();
        if (m_radioThread.joinable()) m_radioThread.join();

        std::vector<libcrtp::CrtpLinkIdentifier> links;
        std::vector<libcrtp::CrtpResponseCallback> callbacks;
        m_links.removeAllLinks(links, callbacks);

        auto packet = libcrtp::nullPacket;
        for (auto &callback : callbacks) callback(&packet, false);
        for (auto &link : links) m_radio->resetLink(&link);
        for (auto &link : links) publishLinkEnd(&link);

        // May throw if the RMW/DDS implementation does not support waiting for acknowledgements.
        if (!link_end_pub->wait_for_all_acked(500ms)) {
            RCLCPP_WARN(get_logger(), "Timed out waiting for link-end acknowledgements.");
        }
    }

private:
    bool radioCallback()
    {
        try {
            libcrtp::CrtpLinkIdentifier link;
            if (!chooseLink(&link)) return false;
            communicateLink(&link);
            return true;
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(get_logger(), "Radio communication failed: %s", ex.what());
            m_diagnostics->reportError(std::string("Radio communication failed: ") + ex.what());
            m_shutdownRequested = true;
            return false;
        }
    }

    void radioLoop()
    {
        if (m_radioPriority > 0) {
            sched_param schedulingParameters{};
            schedulingParameters.sched_priority = m_radioPriority;
            const int error = pthread_setschedparam(
                pthread_self(), SCHED_RR, &schedulingParameters);
            if (error != 0) {
                RCLCPP_WARN(this->get_logger(),
                    "Failed to set radio thread priority to %d: %s",
                    m_radioPriority, strerror(error));
                m_diagnostics->reportError(std::string("Failed to set radio thread priority to ") + std::to_string(m_radioPriority) + std::string(": ") + strerror(error));
            } else {
                RCLCPP_INFO(this->get_logger(), "Radio thread using SCHED_RR priority %d", m_radioPriority);
            }
        }
        while (rclcpp::ok() && !m_shutdownRequested.load()) {
            if (!radioCallback()) {
                std::this_thread::yield();
            }
        }
    }

    bool chooseLink(libcrtp::CrtpLinkIdentifier *link)
    {
        libcrtp::CrtpPort port;

        bool packetsAvailable = m_links.getHighestPriorityLink(link, &port);
        if (packetsAvailable) return true;
        else return m_links.getRandomRelaxedNonBroadcastLink(link);
    }

    void communicateLink(libcrtp::CrtpLinkIdentifier *link)
    {
        libcrtp::CrtpPacket packet;
        libcrtp::CrtpPacket responsePacket;
        bool isPortPacket;

        if (!m_links.linkGetOutboundPacket(link, &packet, &isPortPacket)) return;
        if (link->isBroadcast && !isPortPacket) return; // No broadcast packet available, broadcast links do not send null packets.
                
        bool sendSuccess = m_radio->sendCrtpPacket(link, &packet, &responsePacket);
        if (sendSuccess)
        {
            const bool responseIsNullpacket =
                link->isBroadcast || libcrtp::isNullPacket(&responsePacket);
            if (isPortPacket) m_links.linkNotifySuccessfullPortMessage(link, packet.port, responseIsNullpacket);
            else m_links.linkNotifySuccessfullNullpacket(link, responseIsNullpacket);
            handleReponsePacket(link, &responsePacket);
        } else {
            bool shallDie;
            if (isPortPacket) shallDie = m_links.linkNotifyFailedPortMessage(link);
            else shallDie = m_links.linkNotifyFailedNullpacket(link);
            
            if (shallDie) {
                destroyLink(link);
            }
        }

        m_diagnostics->recordCommunication(packet, sendSuccess, isPortPacket, link->isBroadcast);
    }

    void handleReponsePacket(libcrtp::CrtpLinkIdentifier *link, libcrtp::CrtpPacket *responsePacket)
    {
        if (link->isBroadcast)
            return;
        libcrtp::CrtpResponseCallback callback;
        if (m_links.linkReleasePacket(link, responsePacket, callback))
        {
            callback(responsePacket, true);
        }
        else
        {
            sendCrtpUnresponded(responsePacket, link);
        }
    }

    // Publish a response packet if no callback was registered for it.
    void sendCrtpUnresponded(libcrtp::CrtpPacket *packet, libcrtp::CrtpLinkIdentifier *link)
    {
        if (packet->port == libcrtp::CrtpPort::LINK_LAYER && !packet->dataLength)
            return;
        auto resp = crtp_interfaces::msg::CrtpResponse();

        resp.channel = link->channel;
        for (int i = 0; i < 5; i++)
            resp.address[4 - i] = (link->address & ((uint64_t)0xFF << i * 8)) >> i * 8;
        resp.packet.port = (uint8_t)packet->port;
        resp.packet.channel = packet->channel;
        for (int i = 0; i < packet->dataLength; i++)
            resp.packet.data[i] = packet->data[i];
        resp.packet.data_length = packet->dataLength;
        send_response_pub->publish(resp);
    }

     // Destroys a link and calls all callbacks that were waiting for a response.
    void destroyLink(libcrtp::CrtpLinkIdentifier * link)
    {
        std::vector<libcrtp::CrtpResponseCallback> callbacks;
        libcrtp::CrtpPacket packet = libcrtp::nullPacket;
        if (!m_links.removeLink(link, callbacks)) return;
        m_radio->resetLink(link);
        for(auto& callback : callbacks) {
            callback(&packet, false); // Call each callback with nullptr and false (failure)
        }
        publishLinkEnd(link);

        RCLCPP_DEBUG(this->get_logger(),"Destroyed link: ID: %lX, CH: %d, CC: %ld", link->address, link->channel, callbacks.size());
    }

    // Sends a message that a link has ended.
    void publishLinkEnd(libcrtp::CrtpLinkIdentifier *link)
    {
        auto msg = crtp_interfaces::msg::CrtpLink();
        msg.channel = link->channel;
        for (int i = 0; i < 5; i++)
            msg.address[4 - i] = (link->address & ((uint64_t)0xFF << i * 8)) >> i * 8;
        msg.datarate = link->datarate;
        link_end_pub->publish(msg);
    }

    // Callback for the subscription that closes links
    void closeLinkCallback(const crtp_interfaces::msg::CrtpLink::SharedPtr msg)
    {
        uint64_t address = 0;
        for (int i = 0; i < 5; i++)
            address |= (uint64_t)msg->address[i] << (8 * (4 - i));
        libcrtp::CrtpLinkIdentifier link;
        link.channel = msg->channel;
        link.address = address;
        link.datarate = msg->datarate;
        destroyLink(&link);        
    }

    // Ensure the link exists and resolve its identifier for either send path.
    bool ensureLink(
        const crtp_interfaces::msg::CrtpLink &requested_link,
        libcrtp::CrtpLinkIdentifier &link)
    {
        uint64_t address = 0;
        for (size_t i = 0; i < requested_link.address.size(); ++i) {
            address |= static_cast<uint64_t>(requested_link.address[i])
                << (8 * (requested_link.address.size() - 1 - i));
        }
        m_links.addLink(requested_link.channel, address, requested_link.datarate);
        return m_links.getLinkIdentifier(&link, requested_link.channel, address);
    }

    // Callback for the service that sends CRTP packets
    void sendCrtpPacketCallback(
        const std::shared_ptr<rclcpp::Service<crtp_interfaces::srv::CrtpPacketSend>> service_handle,
        const std::shared_ptr<rmw_request_id_t> header,
        const std::shared_ptr<crtp_interfaces::srv::CrtpPacketSend::Request> request)
    {
        (void)service_handle;

        if (!validateCrtpRequest(request->packet, request->link)) {
            auto failed_packet = libcrtp::nullPacket;
            responseCallback(&failed_packet, false, header);
            return;
        }

        libcrtp::CrtpPacket packet = makeCrtpPacket(
            request->packet,
            request->expects_response,
            request->matching_bytes,
            request->obeys_ordering
        );

        auto response_callback = std::bind(
                &CrazyradioNode::responseCallback,
                this,
                std::placeholders::_1,      // libcrtp::CrtpPacket* pkt
                std::placeholders::_2,      // bool success
                std::make_shared<rmw_request_id_t>(*header)); // shared_ptr<rmw_request_id_t> header
        
        libcrtp::CrtpLinkIdentifier link;
        if (!ensureLink(request->link, link) || !m_links.linkAddPacket(&link, &packet, response_callback)) {
            auto failed_packet = libcrtp::nullPacket;
            response_callback(&failed_packet, false);
            return;
        }
        if (!packet.expectsResponse) response_callback(&packet, false); // echo as response
    }

    // Callback for fire-and-forget CRTP packets.
    void sendCrtpPacketNoResponseCallback(
        const crtp_interfaces::msg::CrtpPacketSendNoResponse::SharedPtr request)
    {
        if (!validateCrtpRequest(request->packet, request->link)) {
            RCLCPP_ERROR(this->get_logger(), "Dropping malformed no-response CRTP packet.");
            return;
        }

        libcrtp::CrtpPacket packet = makeCrtpPacket(request->packet, false, 0, false);

        libcrtp::CrtpLinkIdentifier link;
        if (!ensureLink(request->link, link) || !m_links.linkAddPacket(&link, &packet, nullptr)) {
            RCLCPP_DEBUG(this->get_logger(), "No-response CRTP packet rejected: link unavailable or radio stopping.");
        }
    }
    
    bool validateCrtpRequest(
        const crtp_interfaces::msg::CrtpPacket &packet,
        const crtp_interfaces::msg::CrtpLink &link)
    {
        if (packet.data_length > packet.data.size()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Malformed CRTP packet: data_length %u exceeds %zu",
                static_cast<unsigned>(packet.data_length),
                packet.data.size());
            return false;
        }
        if (packet.port > 15 || packet.channel > 15 ||
            link.channel > 125 || link.datarate > 2) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Malformed CRTP packet (port=%u, channel=%u, "
                "radio_channel=%u, datarate=%u)",
                static_cast<unsigned>(packet.port),
                static_cast<unsigned>(packet.channel),
                static_cast<unsigned>(link.channel),
                static_cast<unsigned>(link.datarate));
            return false;
        }
        return true;
    }

    libcrtp::CrtpPacket makeCrtpPacket(
        const crtp_interfaces::msg::CrtpPacket &msg,
        bool expectsResponse,
        uint8_t matchingBytes,
        bool obeysOrdering)
    {
        libcrtp::CrtpPacket packet = {
            static_cast<libcrtp::CrtpPort>(msg.port),
            msg.channel,
            {/*data*/},
            msg.data_length,
            expectsResponse,
            matchingBytes,
            obeysOrdering,
            std::chrono::steady_clock::now()
        };

        std::copy_n(msg.data.begin(), msg.data_length, packet.data);
        return packet;
    }


    // Callback for sending out link quality information regularly
    void diagnosticsCallback()
    {
        std::vector<libcrtp::CrtpLinkIdentifier> links;
        std::vector<double> crtp_quality;
        m_links.getConnectionStats(links, crtp_quality);

        crazyflie_interfaces::msg::CrazyflieLinkQualityArray crtpQualities;
        crazyflie_interfaces::msg::CrazyflieLinkQualityArray radioQualities;
        std::vector<std::optional<double>> radioRssi;
        for (size_t i = 0; i < links.size(); ++i) {
            if (links[i].isBroadcast) continue;

            const double radio_quality = m_radio->getLinkQuality(&links[i]);
            if (radio_quality < 0.5) {
                RCLCPP_DEBUG(this->get_logger(), "Radio quality for CF 0x%X low! (%d %%)",
                    static_cast<unsigned>(links[i].address & 0xFF),
                    static_cast<int>(radio_quality * 100));
            }

            crazyflie_interfaces::msg::CrazyflieLinkQuality quality;
            quality.id = static_cast<uint8_t>(links[i].address & 0xFF);
            quality.link_quality = crtp_quality[i];
            crtpQualities.link_qualities.push_back(quality);
            quality.link_quality = radio_quality;
            radioQualities.link_qualities.push_back(quality);
            radioRssi.push_back(m_radio->getLinkRssi(&links[i]));
        }

        m_diagnostics->publish(m_links.getStats(), crtpQualities, radioQualities, radioRssi);
    }
private: 
    void responseCallback(
        libcrtp::CrtpPacket * pkt, 
        bool success, 
        std::shared_ptr<rmw_request_id_t> header)
    {
        crtp_interfaces::srv::CrtpPacketSend::Response response;
        response.packet.port = (uint8_t)pkt->port;
        response.packet.channel = pkt->channel;
        response.packet.data_length = pkt->dataLength;
        for (int i = 0; i < pkt->dataLength; i++)
            response.packet.data[i] = pkt->data[i];
        response.success = success;
        try {
            send_crtp_packet_service->send_response(*header, response);
        } catch (const std::exception& e) {
            // https://github.com/ros2/ros2/issues/1253#issuecomment-1702937597
            RCLCPP_WARN(this->get_logger(),"Caught exception:  %s; Port: %d, Ch: %d, Success: %d", e.what(), response.packet.port, response.packet.channel, success);
        }  
    }

private:
    std::unique_ptr<libradio::IRadio> m_radio;
    libcrtp::CrtpLinkContainer m_links;
    std::thread m_radioThread;

    rclcpp::CallbackGroup::SharedPtr crtp_send_callback_group;
    rclcpp::CallbackGroup::SharedPtr radio_callback_group;

    rclcpp::TimerBase::SharedPtr m_diagnostics_timer;

    std::unique_ptr<crazyradio::RadioDiagnostics> m_diagnostics;

    int m_radioPriority;
    rclcpp::TimerBase::SharedPtr radio_timer;
    rclcpp::TimerBase::SharedPtr m_tick_timer;
    uint16_t m_radioPeriodUs;

    rclcpp::Service<crtp_interfaces::srv::CrtpPacketSend>::SharedPtr send_crtp_packet_service;
    rclcpp::Subscription<crtp_interfaces::msg::CrtpPacketSendNoResponse>::SharedPtr
        send_crtp_packet_no_response_sub;
    rclcpp::Publisher<crtp_interfaces::msg::CrtpResponse>::SharedPtr send_response_pub;
    rclcpp::Publisher<crtp_interfaces::msg::CrtpLink>::SharedPtr link_end_pub;
    rclcpp::Subscription<crtp_interfaces::msg::CrtpLink>::SharedPtr link_close_sub;

    std::atomic<bool> m_stopping{false};
    std::atomic<bool> m_shutdownRequested{false};

};

int main(int argc, char **argv)
{
    // Keep ROS alive until the node has failed requests and announced link ends.
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);

    rclcpp::NodeOptions options;
    auto node = std::make_shared<CrazyradioNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    auto shutdown_timer = node->create_wall_timer(50ms, [&node, &executor] {
        if (!shutdown_requested.load() && !node->shutdownRequested()) return;
        node->shutdown();
        executor.cancel();
    });

    executor.add_node(node);
    executor.spin();

    // Also release resources if the context/executor stopped without a signal.
    node->shutdown();
    shutdown_timer.reset();
    executor.remove_node(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
}
