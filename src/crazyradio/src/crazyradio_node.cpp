#define BOOST_BIND_NO_PLACEHOLDERS
#include <chrono>
#include <string.h>
#include <sstream>
#include <map>
// Ros2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "libcrazyradio/Crazyradio.hpp"
#include "libcrtp/CrtpLinkContainer.hpp"

#include "crtp_interfaces/srv/crtp_packet_send.hpp"
#include "crtp_interfaces/msg/crtp_response.hpp"
#include "crtp_interfaces/msg/crtp_link_quality.hpp"
#include "crtp_interfaces/msg/crtp_link_qualities.hpp"


#include <condition_variable>
#include <mutex>
#include <fstream>
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class CrazyradioNode : public rclcpp::Node
{
public:
    CrazyradioNode(const rclcpp::NodeOptions &options)
        : Node("crazyradio_cpp", options)
        , m_radio()
        , m_links()
        , m_radioPeriodMs(1) // 1000 Hz
        , m_logEnabled(true)
    {
        this->declare_parameter("channel", 80);
        uint8_t channel = this->get_parameter("channel").as_int();

        if (m_logEnabled)
        {
            m_logStartTime = this->get_clock()->now();
            m_logStream.open("crazyradio_log_" + std::to_string(channel) + ".log");
        }

        auto qos = rclcpp::QoS(500);
        qos.reliable();
        qos.keep_all();
        qos.durability_volatile();

        crtp_send_callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        send_crtp_packet_service = this->create_service<crtp_interfaces::srv::CrtpPacketSend>(
            "crazyradio/send_crtp_packet" + std::to_string(channel),
            std::bind(&CrazyradioNode::sendCrtpPacketCallback, this, _1, _2, _3),
            qos.get_rmw_qos_profile(),
            crtp_send_callback_group);

        send_response_pub = this->create_publisher<crtp_interfaces::msg::CrtpResponse>(
            "crazyradio/crtp_response", 10);

        link_end_pub = this->create_publisher<crtp_interfaces::msg::CrtpLink>(
            "crazyradio/crtp_link_end", 10);

        link_close_sub = this->create_subscription<crtp_interfaces::msg::CrtpLink>(
            "crazyradio/close_crtp_link", 10,
            std::bind(&CrazyradioNode::closeLinkCallback, this, _1));
        
        link_quality_pub = this->create_publisher<crtp_interfaces::msg::CrtpLinkQualities>(
            "crazyradio/crtp_link_qualities", 10);


        radio_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        radio_timer = this->create_wall_timer(
            std::chrono::milliseconds(m_radioPeriodMs),
            std::bind(&CrazyradioNode::radioCallback, this),
            radio_callback_group);
        
        /**
         * Tick all links periodically.
         * This updates internal link states such as null packet frequency.
         */
        m_relax_timer = this->create_wall_timer(
            std::chrono::milliseconds(1),
            [this] {m_links.tickLinksMs(1); });

        m_link_quality_timer = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&CrazyradioNode::logLinkQuality, this));

        RCLCPP_WARN(this->get_logger(), "Started Crazyradio Node on Channel %d", channel);
    }

private:
    std::stringstream formatCrtpPacket(
        libcrtp::CrtpPacket *packet)
    {
        std::stringstream ss;
        ss << std::dec << " [" << (int)packet->port << ":" << (int)packet->channel << "] ";
        for (int i = 0; i < packet->dataLength; i++) ss << std::hex << (int)packet->data[i] << " ";
        return ss;
    } 

    void logCommunication(
        libcrtp::CrtpLinkIdentifier *link,
        libcrtp::CrtpPacket *packet,
        libcrtp::CrtpPacket *responsePacket,
        bool responseValid)
    {
        if (m_logEnabled)
        {
            std::stringstream ss;
            auto time = this->get_clock()->now() - m_logStartTime;
            ss << "[" << (long int)(time.nanoseconds() / 1000 ) << "] "; //  in microseconds
            ss << std::hex << (int)(uint8_t)(link->address & 0xFF);

            m_logStream << ss.str() << formatCrtpPacket(packet).str() << std::endl;
            if (responseValid) m_logStream << '\t' << formatCrtpPacket(responsePacket).str();
            m_logStream << std::endl;
        }
    }

    void radioCallback()
    {
        libcrtp::CrtpLinkIdentifier link;
        if (chooseLink(&link))
            communicateLink(&link); // This recursively communicates up to 4 packets        
    }    

    bool chooseLink(libcrtp::CrtpLinkIdentifier *link)
    {
        libcrtp::CrtpPort port;

        bool packetsAvailable = m_links.getHighestPriorityLink(link, &port);
        if (packetsAvailable) return true;
        else return m_links.getRandomRelaxedNonBroadcastLink(link);
    }

    void communicateLink(libcrtp::CrtpLinkIdentifier *link, int maxPackets = 4)
    {
        libcrtp::CrtpPacket packet = libcrtp::nullPacket;
        libcrtp::CrtpPacket responsePacket;

        bool isPortPacket = m_links.linkGetHighestPriorityPacket(link, &packet);
        if (link->isBroadcast && !isPortPacket) return; // No broadcast packet available, broadcast links do not send null packets.
                
        bool sendSuccess = sendCrtpPacket(link, &packet, &responsePacket);
        if (sendSuccess)
        {
            if (isPortPacket) m_links.linkNotifySuccessfullPortMessage(link, packet.port);
            else m_links.linkNotifySuccessfullNullpacket(link);
            handleReponsePacket(link, &responsePacket);
        } else {
            bool shallDie;
            if (isPortPacket) shallDie = m_links.linkNotifyFailedPortMessage(link);
            else shallDie = m_links.linkNotifyFailedNullpacket(link);
            
            if (shallDie) {
                destroyLink(link);
            }
        }

        logCommunication(link, &packet, &responsePacket, sendSuccess);

        if (maxPackets > 1
            && sendSuccess 
            && (link->isBroadcast || !libcrtp::isNullPacket(&responsePacket)))
        {
            communicateLink(link, maxPackets - 1);
        }
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

    // Callback for sending out link quality information regularly
    void logLinkQuality()
    {
        std::vector<libcrtp::CrtpLinkIdentifier> links;
        std::vector<double> qualities;
        m_links.getConnectionStats(links, qualities);
        for (size_t i = 0; i < links.size(); i++)
        {
            if (qualities[i] < 0.5)
            {
                RCLCPP_INFO(this->get_logger(), "Link Quality for CF 0x%X low! (%d %%)", (int)(uint8_t)(links[i].address & 0xFF), (int)(qualities[i] * 100));
            }
        }

        crtp_interfaces::msg::CrtpLinkQualities msg;
        for (size_t i = 0; i < links.size(); i++) {
            crtp_interfaces::msg::CrtpLinkQuality quality;
            crtp_interfaces::msg::CrtpLink link_msg;
            link_msg.channel = links[i].channel;
            for (int j = 0; j < 5; j++)
                link_msg.address[4 - j] = (links[i].address & ((uint64_t)0xFF << j * 8)) >> j * 8;
            link_msg.datarate = links[i].datarate;
            quality.link = link_msg;
            quality.link_quality = qualities[i];
            msg.link_qualities.push_back(quality);
        }   
        if (links.size()) link_quality_pub->publish(msg);
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

    // Destroys a link and calls all callbacks that were waiting for a response.
    void destroyLink(libcrtp::CrtpLinkIdentifier * link)
    {
        std::vector<libcrtp::CrtpResponseCallback> callbacks;
        libcrtp::CrtpPacket packet = libcrtp::nullPacket;
        // During this time a packet might get added to the link. There is no way to check for this.
     
        m_links.removeLink(link, callbacks);
        RCLCPP_WARN(this->get_logger(),"Destroying link: ID: %lX, CH: %d, CC: %ld", link->address, link->channel, callbacks.size());
        for(auto& callback : callbacks) {
            callback(&packet, false); // Call each callback with nullptr and false (failure)
        }
        publishLinkEnd(link);
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

    // Callback for the service that sends CRTP packets
    void sendCrtpPacketCallback(
        const std::shared_ptr<rclcpp::Service<crtp_interfaces::srv::CrtpPacketSend>> service_handle,
        const std::shared_ptr<rmw_request_id_t> header,
        const std::shared_ptr<crtp_interfaces::srv::CrtpPacketSend::Request> request)
    {
        // Insert Link
        uint64_t address = 0;
        for (int i = 0; i < 5; i++)
            address |= (uint64_t)request->link.address[i] << (8 * (4 - i));
        m_links.addLink(request->link.channel, address, request->link.datarate);

        // Create Packet
        libcrtp::CrtpPacket packet = {
            (libcrtp::CrtpPort)request->packet.port,
            request->packet.channel,
            {/*data*/},
            request->packet.data_length,
            request->expects_response,
            request->matching_bytes,
            request->obeys_ordering};
        for (int i = 0; i < request->packet.data_length; i++)
            packet.data[i] = request->packet.data[i];

        libcrtp::CrtpLinkIdentifier link;
        if (this->m_links.getLinkIdentifier(&link, request->link.channel, address))
        { // release if not
            // RCLCPP_WARN(this->get_logger(),"Add Packet %X; [%d; %d]!", (uint8_t)(link.address & 0xFF), packet.port, packet.channel );
            if (packet.expectsResponse)
            {
                // Start thread which puts packet onto the queue and waits for response, if this is necessary
                std::thread t([this, packet, channel = request->link.channel, address, service_handle, header, link]() mutable
                {
                    using namespace std::chrono_literals;
                    volatile bool callback_called = false;

                    auto response_callback = [this, service_handle, header, &callback_called](libcrtp::CrtpPacket *pkt, bool success)
                        {   
                            auto response = crtp_interfaces::srv::CrtpPacketSend::Response();
                            response.packet.port = (uint8_t)pkt->port;
                            response.packet.channel = pkt->channel;
                            response.packet.data_length = pkt->dataLength;
                            for (int i = 0; i < pkt->dataLength; i++)
                                response.packet.data[i] = pkt->data[i];
                            response.success = success;
                            try {
                                service_handle->send_response(*header, response);
                            } catch (const std::exception& e) {
                                // https://github.com/ros2/ros2/issues/1253#issuecomment-1702937597
                                RCLCPP_WARN(this->get_logger(),"Caught exception:  %s; Port: %d, Ch: %d, Success: %d", e.what(), response.packet.port, response.packet.channel, success);
                            }  
                            callback_called = true;
                        };
                    this->m_links.linkAddPacket(&link, &packet, response_callback);                         

                    while (!callback_called)
                        std::this_thread::sleep_for(10ms);
                          // std::cerr << "Thread going down\n";
                });
                t.detach();        
            }
            else
            {
                this->m_links.linkAddPacket(&link, &packet, NULL);
                auto response = crtp_interfaces::srv::CrtpPacketSend::Response(); // send an empty response
                try {
                    service_handle->send_response(*header, response);
                } catch (const std::exception& e) {
                    // https://github.com/ros2/ros2/issues/1253#issuecomment-1702937597
                    RCLCPP_WARN(this->get_logger(),"Caught exception:  %s; Port: %d, Ch: %d, unresp", e.what(), response.packet.port, response.packet.channel);
                }  
            }
        }
    }


private:
    
    /**
     * Transmits a CrtpPacket over the Crazyradio.
     * Returns true if the packet was sent successfully.
     * If the link is non broadcast and the packet was sent successfully, the responsePacket will contain a response from the Crazyflie.
    */
    bool sendCrtpPacket(
        libcrtp::CrtpLinkIdentifier *link,
        libcrtp::CrtpPacket *packet,
        libcrtp::CrtpPacket *responsePacket)
    {
        auto start = std::chrono::high_resolution_clock::now();

        libcrazyradio::Crazyradio::Ack ack;
        m_radio.sendCrtpPacket(link, packet, ack);


        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::micro> elapsed = end - start;
        // std::cerr << "Packet transfer took " << elapsed.count() << " us" << std::endl;
        
        if (link->isBroadcast) return true;
        
        if (!ack.ack) {
            return false;
        } else if (!ack.size)
        {
            /* The Bug in https://github.com/bitcraze/crazyflie-firmware/issues/703 prevents a response from beeing sent back from the crazyflie.
             *  The message however gets succesfully received by the crazyflie.
             *  For now we just assume that a nullpacket would have been sent from crazyflie, in order not to break any other code.
             */
            RCLCPP_WARN(this->get_logger(), "Empty response #703");
            memcpy(responsePacket, &libcrtp::nullPacket, sizeof(libcrtp::CrtpPacket));
            return true;
        } 
        
        libcrazyradio::Crazyradio::ackToCrtpPacket(&ack, responsePacket);
        return true;
    }

private:
    libcrazyradio::Crazyradio m_radio;
    libcrtp::CrtpLinkContainer m_links;

    rclcpp::CallbackGroup::SharedPtr crtp_send_callback_group;
    rclcpp::CallbackGroup::SharedPtr radio_callback_group;

    rclcpp::TimerBase::SharedPtr m_link_quality_timer;

    rclcpp::TimerBase::SharedPtr radio_timer;
    rclcpp::TimerBase::SharedPtr m_relax_timer;
    uint8_t m_radioPeriodMs;

    rclcpp::Service<crtp_interfaces::srv::CrtpPacketSend>::SharedPtr send_crtp_packet_service;
    rclcpp::Publisher<crtp_interfaces::msg::CrtpResponse>::SharedPtr send_response_pub;
    rclcpp::Publisher<crtp_interfaces::msg::CrtpLink>::SharedPtr link_end_pub;
    rclcpp::Publisher<crtp_interfaces::msg::CrtpLinkQualities>::SharedPtr link_quality_pub;
    rclcpp::Subscription<crtp_interfaces::msg::CrtpLink>::SharedPtr link_close_sub;

    std::mutex m_radioMutex;

    std::ofstream m_logStream;
    rclcpp::Time m_logStartTime;
    bool m_logEnabled = false;

};

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<CrazyradioNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
