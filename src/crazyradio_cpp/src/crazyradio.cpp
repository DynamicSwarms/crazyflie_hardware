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
            m_radio(libcrazyradio::Crazyradio(0))

        {
            auto qos = rclcpp::QoS(500);
            qos.reliable();
            qos.keep_all();
            qos.durability_volatile();


            crtp_send_callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            send_crtp_packet_service = this->create_service<crtp_interface::srv::CrtpPacketSend>("crazyradio/send_crtp_packet", std::bind(&CrazyradioNode::sendCrtpPacketCallback, this, _1, _2,_3), qos.get_rmw_qos_profile(), crtp_send_callback_group);
            send_response_pub = this->create_publisher<crtp_interface::msg::CrtpResponse>("crazyradio/crtp_response", 10);
            
            antenna_timer = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&CrazyradioNode::antennaCallback, this));
            /*
            m_radio.setChannel(100);
            m_radio.setAddress(0xE7E7E7E700);
            m_radio.setDatarate(libcrazyradio::Crazyradio::Datarate::Datarate_2MPS);

            libcrazyradio::Crazyradio::Ack ack;
            
                                    // port and channel
            const uint8_t data [] = {0x2 << 4 | 0x2, 23, 0x00, 0x03 }; // set led color ring
            m_radio.sendPacket(data, 4, ack);


            for (int n = 0; n < 5; n++) {
                const uint8_t data [] = {0xf3}; // send nullpacket
                m_radio.sendPacket(data, 1, ack);
            

                if (!ack.ack) RCLCPP_WARN(this->get_logger(),"Not succesfull");
                for (int i = 0; i < ack.size; i++) {
                    RCLCPP_WARN(this->get_logger(),"%x", ack.data[i]);    
                }
            }
            */
            RCLCPP_WARN(this->get_logger(),"I am alive");

            /*

            std::cerr << "Starting Test: \n";
            libcrtp::CrtpLink linkA = libcrtp::CrtpLink(100, 0xF0, 2);
            libcrtp::CrtpLink linkB = libcrtp::CrtpLink(100, 0xF1, 2);
            
            libcrtp::CrtpLink linkReal = libcrtp::CrtpLink(100, 0xE7E7E7E700, 2); // Corresponsing to a real 


            libcrtp::CrtpPacket exPacket = {libcrtp::CrtpPort::COMMANDER, 3, {3,3,3}, 3, false, 0, true}; // port, channel, data, length
            linkA.addPacket(&exPacket, std::bind(&CrazyradioNode::packetCallback, this, std::placeholders::_1));

            libcrtp::CrtpPacket pkt2 = {libcrtp::CrtpPort::CONSOLE, 0, {0,0,0}, 3, false, 0, true}; // port, channel, data, length
            linkB.addPacket(&pkt2, std::bind(&CrazyradioNode::packetCallback, this, std::placeholders::_1));

            libcrtp::CrtpPacket pkt3 = {libcrtp::CrtpPort::PARAMETERS, 2, {2,2,2}, 3, false, 0, true}; // port, channel, data, length
            linkA.addPacket(&pkt3,std::bind(&CrazyradioNode::packetCallback, this, std::placeholders::_1));

            libcrtp::CrtpPacket pktNull = {libcrtp::CrtpPort::LINK_LAYER, 3, {}, 0, false, 0, true}; // port, channel, data, length
            linkReal.addPacket(&pktNull, std::bind(&CrazyradioNode::packetCallback, this, std::placeholders::_1));


            
            
            m_links.insert({{linkA.getChannel(), linkA.getAddress()}, linkA});
            m_links.insert({{linkB.getChannel(), linkB.getAddress()}, linkB});
            
            m_links.insert({{linkReal.getChannel(), linkReal.getAddress()}, linkReal});

            */
            
            RCLCPP_WARN(this->get_logger(),"Started Crazyradio Node");
        }
    private: 

        void packetCallback(libcrtp::CrtpPacket * packet)  
        {
            RCLCPP_WARN(this->get_logger(),"Callback called");
        }

        void antennaCallback()
        {   
            libcrtp::CrtpPort highestPriorityPort = libcrtp::CrtpPort::NO_PORT;
            do {
                // gets highest priority link
                highestPriorityPort = libcrtp::CrtpPort::NO_PORT;
                std::pair<uint8_t, uint64_t> bestKey = {0,0};
                for (const auto& [key, link] : m_links) 
                {       
                    libcrtp::CrtpPort port = link.getPriorityPort();
                    if (port < highestPriorityPort) {
                        highestPriorityPort = port;
                        bestKey = key;
                    } 
                }

                auto link_it = m_links.find(bestKey);
                if (highestPriorityPort != libcrtp::CrtpPort::NO_PORT && link_it != m_links.end()) {
                    libcrtp::CrtpPacket  pkt;
                    
                    bool success = link_it->second.getPacket(highestPriorityPort, &pkt);

                    if (success) 
                    {  
                        libcrtp::CrtpPacket  responsePacket; 
                        bool sendSuccess = sendCrtpPacket(&pkt, &responsePacket,&link_it->second);
                        if (sendSuccess) {
                            libcrtp::CrtpResponseCallback callback; 
                            if (link_it->second.releasePacket(&responsePacket, callback)) {
                                //RCLCPP_INFO(this->get_logger(), "Calling callback");
                                //sendCrtpUnresponded(&responsePacket, &link_it->second); 
                                callback(&responsePacket); // This callback needs to be used at some point
                            } else {
                                sendCrtpUnresponded(&responsePacket, &link_it->second); 
                                //RCLCPP_INFO(this->get_logger(), "Received Packet which wasnt to be responded to");
                            }
                        }
                    }
                }
            } while (highestPriorityPort != libcrtp::CrtpPort::NO_PORT);
            //RCLCPP_INFO(this->get_logger(), "Hello from ROS2");
        }

        void sendCrtpUnresponded(libcrtp::CrtpPacket * packet, libcrtp::CrtpLink * link)
        {
            auto resp = crtp_interface::msg::CrtpResponse();
            resp.channel = link->getChannel();
            uint64_t address = link->getAddress();
            std::vector<uint8_t> addr;
            for (int i = 0; i < 5; i++) addr.push_back( (address & (0xFF << i )     ) >> i);
            resp.address = addr;
            resp.packet.port = (uint8_t)packet->port;
            resp.packet.channel = packet->channel;
            for (int i = 0; i < packet->dataLength; i++) resp.packet.data[i] = packet->data[i];
            resp.packet.data_length = packet->dataLength;
            send_response_pub->publish(resp);
        }

        bool sendCrtpPacket(libcrtp::CrtpPacket * packet, libcrtp::CrtpPacket * responsePacket, libcrtp::CrtpLink * link)
        {
            std::stringstream ss;
            ss <<  "Link:" << (int)link->getAddress()  <<"Packet:" << (int)packet->port << (int)packet->channel << " D: " << (int)packet->data[0];// << "\n";
            //RCLCPP_WARN(this->get_logger(),ss.str().c_str());


            link->setRadio(&m_radio); // Sets Channel/Address/Datarate of radio to link-specific settings
            libcrazyradio::Crazyradio::Ack ack;
            uint8_t data[32];
            data[0] = packet->port << 4 | packet->channel;
            memcpy(&data[1], &packet->data, packet->dataLength);
            m_radio.sendPacket(data, 1 + packet->dataLength , ack);

            //libcrtp::CrtpPacket responsePacket;
            if (!ack.ack) RCLCPP_WARN(this->get_logger(),"Not succesfull");
            else if (!ack.size) RCLCPP_WARN(this->get_logger(),"Empty response #703");
            else {               
                responsePacket->port = (libcrtp::CrtpPort)((ack.data[0] & 0b11111100) >> 4);
                responsePacket->channel = ack.data[0] & 0b11;
                for (int i = 0; i < ack.size; i++) responsePacket->data[i] = ack.data[i+1];
                responsePacket->dataLength = ack.size -1;
                
                //RCLCPP_WARN(this->get_logger(),"Succesfull Response");
                return true;
            }       
            return false;  
        }


        void sendCrtpPacketCallback(
            const std::shared_ptr<rclcpp::Service<crtp_interface::srv::CrtpPacketSend>> service_handle,
            const std::shared_ptr<rmw_request_id_t> header,
            const std::shared_ptr<crtp_interface::srv::CrtpPacketSend::Request> request)
        {
            // Choose/Insert Link
            uint8_t channel = request->channel;
            uint64_t address = 0;
            for (int i = 4; i >= 0; i--) address |= (uint64_t)request->address[i] << (8* (i+1));
            uint8_t datarate = request->datarate;
            std::pair<uint8_t, uint64_t> linkKey = {channel, address};
            m_links.insert({linkKey, libcrtp::CrtpLink(channel, address, datarate)}); // If already in m_links this wont duplicate

            // Create Packet and put into Link
            libcrtp::CrtpPort port = (libcrtp::CrtpPort)request->packet.port;
            uint8_t crtp_channel = request->packet.channel;
            uint8_t dataLength = request->packet.data_length;
            libcrtp::CrtpPacket packet = {port, crtp_channel, {/*data*/}, dataLength,// port, channel, data, length
                        request->expects_response, request->matching_bytes, request->obeys_ordering}; // expects_response, matching_bytes, obeys_ordering
            for (int i = 0; i < dataLength; i++) packet.data[i] = request->packet.data[i];          

            // Start thread which puts it onto the queue and waits for response            
            std::thread t([this, packet, linkKey,service_handle, header]() mutable {
                /* ** takes a long time to respond ** */
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
                    //std::cerr << "Inside function \n";

                    // RCLCPP_WARN(this->get_logger(),"Nested Callback Response");
                    callback_called = true;
                };
                auto link = this->m_links.find(linkKey); // A sad cpp construct
                
                if (packet.expectsResponse) {
                    if (link != this->m_links.end()) link->second.addPacket(&packet, response_callback);
                } else {
                    if (link != this->m_links.end()) link->second.addPacket(&packet, NULL);
                    response_callback(&packet);
                }

                while (!callback_called) std::this_thread::sleep_for(10ms);
                //std::cerr << "Thread going down\n";              
                });
            t.detach();
            
            
            
            /*
            libcrazyradio::Crazyradio::Ack ack;
            uint8_t data[32];
            data[0] = request->packet.port << 4 | request->packet.channel;
            memcpy(&data[1], &request->packet.data, request->packet.data_length);
            m_radio.setChannel(request->channel);
            switch (request->datarate) {
                case 2: 
                    m_radio.setDatarate(libcrazyradio::Crazyradio::Datarate::Datarate_2MPS);
                    break;
                case 1: 
                    m_radio.setDatarate(libcrazyradio::Crazyradio::Datarate::Datarate_1MPS);
                    break;
                default: 
                    m_radio.setDatarate(libcrazyradio::Crazyradio::Datarate::Datarate_250KPS);
            }
            
            uint64_t address = 0;
            for (int i = 4; i >= 0; i--) address |= (uint64_t)request->address[i] << (8* (i+1));
            m_radio.setAddress(address);
            */


            //for (int i = 0; i < request->packet.data_length +1; i++ ) RCLCPP_WARN(this->get_logger(),"Data %x", data[i] );
            //RCLCPP_WARN(this->get_logger(),"Datarate: %d", request->datarate);
            //RCLCPP_WARN(this->get_logger(),"Channel: %d", request->channel);
            
            /*
            m_radio.sendPacket(data, request->packet.data_length + 1, ack);
            if (!ack.ack) RCLCPP_WARN(this->get_logger(),"Not succesfull");
            else if (!ack.size) RCLCPP_WARN(this->get_logger(),"Empty response #703");
            else {
                auto resp = crtp_interface::msg::CrtpResponse();
                resp.channel = request->channel;
                std::vector<uint8_t> addr;
                for (int i = 0; i < 5; i++) addr.push_back(request->address[i]);
                resp.address = addr;
                resp.packet.port = (ack.data[0] & 0b11111100) >> 4;
                resp.packet.channel = ack.data[0] & 0b11;
                for (int i = 0; i < ack.size; i++) resp.packet.data[i] = ack.data[i+1];
                resp.packet.data_length = ack.size -1;
                send_response_pub->publish(resp);
            }           
*/
      
    //        

            //RCLCPP_WARN(this->get_logger(),"Address %lX", address );
            //    for (int i = 0; i < ack.size; i++) {
            //        RCLCPP_WARN(this->get_logger(),"%x", ack.data[i]);    
            //}

        }
    private:    
        libcrazyradio::Crazyradio m_radio;
        std::map<std::pair<uint8_t, uint64_t>, libcrtp::CrtpLink> m_links;

        rclcpp::CallbackGroup::SharedPtr crtp_send_callback_group;

        rclcpp::TimerBase::SharedPtr antenna_timer; 

        rclcpp::Service<crtp_interface::srv::CrtpPacketSend>::SharedPtr send_crtp_packet_service;
        rclcpp::Publisher<crtp_interface::msg::CrtpResponse>::SharedPtr send_response_pub;
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