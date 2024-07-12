#define BOOST_BIND_NO_PLACEHOLDERS
#include <chrono>
#include <string.h>
#include <sstream>
#include <map>
//Ros2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "libcrazyradio/Crazyradio.hpp"
#include "libcrazyradio/CrtpLink.hpp"


#include "crtp_interface/srv/crtp_packet_send.hpp"
#include "crtp_interface/msg/crtp_response.hpp"


#include <condition_variable>
#include <mutex>
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


class CrazyradioNode : public rclcpp::Node
{
    public: 
        CrazyradioNode(const rclcpp::NodeOptions & options) : 
            Node("crazyradio_cpp", options),
            m_radio(libcrazyradio::Crazyradio(0)),
            m_links()
        {
            auto qos = rclcpp::QoS(500);
            qos.reliable();
            qos.keep_all();
            qos.durability_volatile();

            crtp_send_callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            send_crtp_packet_service = this->create_service<crtp_interface::srv::CrtpPacketSend>(
                "crazyradio/send_crtp_packet",
                std::bind(&CrazyradioNode::sendCrtpPacketCallback, this, _1, _2,_3),
                qos.get_rmw_qos_profile(),
                crtp_send_callback_group);
            
            send_response_pub = this->create_publisher<crtp_interface::msg::CrtpResponse>(
                "crazyradio/crtp_response", 10);
            
            antenna_timer = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&CrazyradioNode::antennaCallback, this));
                        
            RCLCPP_WARN(this->get_logger(),"Started Crazyradio Node");
        }
    private: 
        void antennaCallback()
        {   
            std::unique_lock<std::mutex> mlock(m_linksMutex);
            
            libcrtp::CrtpPacket  outPacket;
            libcrtp::CrtpPacket  responsePacket;       
            libcrtp::CrtpResponseCallback callback;

            libcrtp::CrtpLink * link;
            libcrtp::CrtpPort port;

            while (m_links.getHighestPriorityLink(&link, &port))
            {
                if (link->getPacket(port, &outPacket)) 
                {  
                    if (this->sendCrtpPacket(link, &outPacket, &responsePacket)) 
                    {
                        if (!link->isBroadcast())
                        {
                            if (link->releasePacket(&responsePacket, callback)) 
                            {
                                callback(&responsePacket);
                            } else 
                            {
                                sendCrtpUnresponded(&responsePacket, link); 
                            }
                        }
                    }
                }
            } 
            
            /*
            * Send out nullpackets, should we only do this, if there are packets to be responded to?
            * Keep logging in mind
            */
            if (m_links.getRandomLink(&link) && !link->isBroadcast())
            {
                libcrtp::CrtpPacket packet = libcrtp::nullPacket;
                link->addPacket(&packet, NULL);
            }
        }

        void sendCrtpUnresponded(libcrtp::CrtpPacket * packet, libcrtp::CrtpLink * link)
        {
            uint64_t address = link->getAddress();
            std::vector<uint8_t> addr;
            for (int i = 0; i < 5; i++) addr.push_back( (address & (0xFF << i )     ) >> i);
            
            auto resp = crtp_interface::msg::CrtpResponse();
            resp.channel = link->getChannel();           
            resp.address = addr;
            resp.packet.port = (uint8_t)packet->port;
            resp.packet.channel = packet->channel;
            for (int i = 0; i < packet->dataLength; i++) resp.packet.data[i] = packet->data[i];
            resp.packet.data_length = packet->dataLength;
            send_response_pub->publish(resp);
        }

        bool sendCrtpPacket(
            libcrtp::CrtpLink * link,
            libcrtp::CrtpPacket * packet,
            libcrtp::CrtpPacket * responsePacket)
        {  
            libcrazyradio::Crazyradio::Ack ack;

            m_radio.sendCrtpPacket(link, packet, ack);
            if (link->isBroadcast()) 
            {
                return true;
            }
            if (!ack.ack)       RCLCPP_WARN(this->get_logger(),"Crazyflie with id 0x%X not reachable!", (uint8_t)(link->getAddress() & 0xFF) );
            else if (!ack.size) RCLCPP_WARN(this->get_logger(),"Empty response #703");
            else {               
                libcrazyradio::Crazyradio::ackToCrtpPacket(&ack, responsePacket);
                return true;
            }       
            return false;  
        }


        void sendCrtpPacketCallback(
            const std::shared_ptr<rclcpp::Service<crtp_interface::srv::CrtpPacketSend>> service_handle,
            const std::shared_ptr<rmw_request_id_t> header,
            const std::shared_ptr<crtp_interface::srv::CrtpPacketSend::Request> request)
        {
            std::unique_lock<std::mutex> mlock(m_linksMutex);

            // Insert Link
            uint64_t address = 0;
            for (int i = 0; i < 5; i++) address |= (uint64_t)request->address[i] << (8 * (4 - i));
            m_links.addLink(request->channel, address, request->datarate);
            
            // Create Packet 
            libcrtp::CrtpPacket packet = {
                    (libcrtp::CrtpPort)request->packet.port,
                    request->packet.channel,
                    {/*data*/}, 
                    request->packet.data_length,
                    request->expects_response,
                    request->matching_bytes,
                    request->obeys_ordering}; 
            for (int i = 0; i < request->packet.data_length; i++) packet.data[i] = request->packet.data[i];          

            // Start thread which puts packet onto the queue and waits for response, if this is necessary            
            std::thread t([this, packet, channel = request->channel, address,service_handle, header,  mlock = std::move(mlock)]() mutable {
                using namespace std::chrono_literals;

                volatile bool callback_called = false;

                auto response_callback = [service_handle, header, &callback_called] (libcrtp::CrtpPacket * pkt) 
                {
                    auto response = crtp_interface::srv::CrtpPacketSend::Response();
                    response.packet.port = (uint8_t)pkt->port;
                    response.packet.channel = pkt->channel;
                    response.packet.data_length = pkt->dataLength;
                    for (int i = 0; i < pkt->dataLength; i++) response.packet.data[i] = pkt->data[i];
                    service_handle->send_response(*header, response);
                    
                    callback_called = true;
                };


                libcrtp::CrtpLink * link;
                if (this->m_links.getLink(&link, channel, address)) { // release if not 
                    if (packet.expectsResponse) 
                    {
                        link->addPacket(&packet, response_callback);
                    } else  
                    {
                        link->addPacket(&packet, NULL);
                        response_callback(&packet); // So we dont wait infinetely
                    }
                }
                
                mlock.unlock();

                while (!callback_called) std::this_thread::sleep_for(10ms);
                //std::cerr << "Thread going down\n";              
                });
            t.detach();
        }

    private:   
        libcrazyradio::Crazyradio m_radio;
        libcrtp::CrtpLinkContainer m_links;

        rclcpp::CallbackGroup::SharedPtr crtp_send_callback_group;

        rclcpp::TimerBase::SharedPtr antenna_timer; 

        rclcpp::Service<crtp_interface::srv::CrtpPacketSend>::SharedPtr send_crtp_packet_service;
        rclcpp::Publisher<crtp_interface::msg::CrtpResponse>::SharedPtr send_response_pub;

        std::mutex m_radioMutex;
        std::mutex m_linksMutex;
};


int main(int argc, char ** argv) 
{
    (void) argc;
    (void) argv;

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<CrazyradioNode>(options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}