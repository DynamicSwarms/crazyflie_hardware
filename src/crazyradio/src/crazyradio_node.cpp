#define BOOST_BIND_NO_PLACEHOLDERS
#include <chrono>
#include <string.h>
#include <sstream>
#include <map>
// Ros2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "libcrazyradio/Crazyradio.hpp"
#include "libcrtp/CrtpLink.hpp"

#include "crtp_interfaces/srv/crtp_packet_send.hpp"
#include "crtp_interfaces/msg/crtp_response.hpp"

#include <condition_variable>
#include <mutex>
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class CrazyradioNode : public rclcpp::Node
{
public:
    CrazyradioNode(const rclcpp::NodeOptions &options)
        : Node("crazyradio_cpp", options),  m_radio(), m_links(), m_radioPeriodMs(2) // 500 Hz
    {
        this->declare_parameter("channel", 80);
        uint8_t channel = this->get_parameter("channel").as_int();

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


        radio_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        radio_timer = this->create_wall_timer(
            std::chrono::milliseconds(m_radioPeriodMs),
            std::bind(&CrazyradioNode::radioCallback, this),
            radio_callback_group);

        RCLCPP_WARN(this->get_logger(), "Started Crazyradio Node on Channel %d", channel);
    }

private:
    void radioCallback()
    {
        m_links.relaxLinks(m_radioPeriodMs);
        /**
         * Relax all links because time has passed. The link relaxations allows
         * for periodic sending of nullpackets, with a maxmim rate for each link.
         * Nullpackets only get sent if no link want's to send anything.
         * You can achieve faster polling rate by manually sending nullpackets.
         * They will not "tense" the link.
         */

        libcrtp::CrtpLinkIdentifier link;
        libcrtp::CrtpPort port;

        if (m_links.getHighestPriorityLink(&link, &port))
        {
            sendPacketFromPort(&link, port);
        }
        else if (m_links.getRandomRelaxedNonBroadcastLink(&link))
        {
            if (sendNullpacket(&link))
            {
                m_links.linkTense(&link);
            }
        }
    }


    /**
     * Send out a Nullpacket.
     * We cannot add the nullpacket into the message queue because this would prioritize 
     * the link until the nullpacket is succesfully received. 
    */
    bool sendNullpacket(libcrtp::CrtpLinkIdentifier * link)
    {
        libcrtp::CrtpPacket packet = libcrtp::nullPacket;
        libcrtp::CrtpPacket responsePacket;
        if (this->sendCrtpPacket(link, &packet, &responsePacket))
        {
            m_links.linkNotifySuccessfullNullpacket(link);
            handleReponsePacket(link, &responsePacket);
            return true;
        }
        this->onFailedMessage(link);
        return false;
    }

    /**
     * Try to send a packet from a given port of the link. 
     * Getting the packet doesnot remeove it from queues yet. 
     * Only if received it will get removed from queue.
    */
    bool sendPacketFromPort(libcrtp::CrtpLinkIdentifier *link, libcrtp::CrtpPort port)
    {
        libcrtp::CrtpPacket outPacket;
        libcrtp::CrtpPacket responsePacket;
        if (!m_links.linkGetPacket(link, port, &outPacket))
            return false; // Something went wrong when choosing packet

        if (this->sendCrtpPacket(link, &outPacket, &responsePacket))
        {
            m_links.linkNotifySuccessfullMessage(link, port);
            handleReponsePacket(link, &responsePacket);
            return true;
        }
        this->onFailedMessage(link);        
        return false;
    }

    void onFailedMessage(libcrtp::CrtpLinkIdentifier *link)
    {
        if (m_links.linkNotifyFailedMessage(link))
        {
            destroyLink(link);
            crtpLinkEndCallback(link);
        }
        else
        {
            double linkQuality = m_links.linkGetLinkQuality(link);
            if (linkQuality < 0.5)
            {
                RCLCPP_DEBUG(this->get_logger(), "Link Quality for CF 0x%X low! (%d %%)", (uint8_t)(link->address & 0xFF), (uint8_t)(linkQuality * 100));
            }
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

    void crtpLinkEndCallback(libcrtp::CrtpLinkIdentifier *link)
    {
        auto msg = crtp_interfaces::msg::CrtpLink();
        msg.channel = link->channel;
        for (int i = 0; i < 5; i++)
            msg.address[4 - i] = (link->address & ((uint64_t)0xFF << i * 8)) >> i * 8;
        msg.datarate = link->datarate;
        link_end_pub->publish(msg);
    }

    bool sendCrtpPacket(
        libcrtp::CrtpLinkIdentifier *link,
        libcrtp::CrtpPacket *packet,
        libcrtp::CrtpPacket *responsePacket)
    {
        libcrazyradio::Crazyradio::Ack ack;
        //RCLCPP_WARN(this->get_logger(),"[%d; %d]! Starting send", link->channel,  (uint8_t)(link->address & 0xFF));


        m_radio.sendCrtpPacket(link, packet, ack);
        // RCLCPP_WARN(this->get_logger(),"%X; [%d; %d]!", (uint8_t)(link->address & 0xFF), packet->port, packet->channel );
        if (link->isBroadcast)
        {
            return true;
        }
        if (!ack.ack) {
            //RCLCPP_WARN(this->get_logger(),"[%d; %d]! ACK ISSUE", link->channel,  (uint8_t)(link->address & 0xFF));
            return false; // RCLCPP_WARN(this->get_logger(),"Crazyflie with id 0x%X not reachable!", (uint8_t)(link->address & 0xFF) );
        }
        else if (!ack.size)
        {
            /* The Bug in https://github.com/bitcraze/crazyflie-firmware/issues/703 prevents a response from beeing sent back from the crazyflie.
             *  The message however gets succesfully received by the crazyflie.
             *  For now we just assume that a nullpacket would have been sent from crazyflie, in order not to break any other code.
             */
            RCLCPP_WARN(this->get_logger(), "Empty response #703");
            memcpy(responsePacket, &libcrtp::nullPacket, sizeof(libcrtp::CrtpPacket));
            return true;
        }
        else
        {
            libcrazyradio::Crazyradio::ackToCrtpPacket(&ack, responsePacket);
            //RCLCPP_WARN(this->get_logger(),"[%d; %d]! Success", link->channel,  (uint8_t)(link->address & 0xFF));
            return true;
        }
        return false;
    }

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
    }

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
    libcrazyradio::Crazyradio m_radio;
    libcrtp::CrtpLinkContainer m_links;

    rclcpp::CallbackGroup::SharedPtr crtp_send_callback_group;
    rclcpp::CallbackGroup::SharedPtr radio_callback_group;

    rclcpp::TimerBase::SharedPtr radio_timer;
    uint8_t m_radioPeriodMs;

    rclcpp::Service<crtp_interfaces::srv::CrtpPacketSend>::SharedPtr send_crtp_packet_service;
    rclcpp::Publisher<crtp_interfaces::msg::CrtpResponse>::SharedPtr send_response_pub;
    rclcpp::Publisher<crtp_interfaces::msg::CrtpLink>::SharedPtr link_end_pub;
    rclcpp::Subscription<crtp_interfaces::msg::CrtpLink>::SharedPtr link_close_sub;

    std::mutex m_radioMutex;
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
