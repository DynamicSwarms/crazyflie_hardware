#define BOOST_BIND_NO_PLACEHOLDERS
#include <chrono>
#include <string.h>

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


            std::cerr << "Starting Test: \n";
            libcrtp::CrtpLink linkA = libcrtp::CrtpLink(100, 0xF0, 2);
            libcrtp::CrtpLink linkB = libcrtp::CrtpLink(100, 0xF1, 2);
            
            libcrtp::CrtpPacket exPacket = {libcrtp::CrtpPort::PARAMETERS, 3, {3, 3, 3}, 3, false, 0, true}; // port, channel, data, length
            linkA.addPacket(&exPacket);

            libcrtp::CrtpPacket pkt2 = {libcrtp::CrtpPort::CONSOLE, 3, {1,1,1}, 3, false, 0, true}; // port, channel, data, length
            linkB.addPacket(&pkt2);

            std::map<std::pair<uint8_t, uint8_t>, libcrtp::CrtpLink> links;
            links.insert({{linkA.getChannel(), linkA.getAddress()}, linkA});
            links.insert({{linkB.getChannel(), linkB.getAddress()}, linkB});

            libcrtp::CrtpPort highestPriorityPort = libcrtp::CrtpPort::NO_PORT;
            do {
                // gets highest priority link
                highestPriorityPort = libcrtp::CrtpPort::NO_PORT;
                std::pair<uint8_t, uint8_t> bestKey = {0,0};
                for (const auto& [key, link] : links) 
                {       
                    libcrtp::CrtpPort port = link.getPriorityPort();
                    std::cerr << "Link address: " << (int)link.getAddress() << "port" << (int)port << "\n";
                    if (port < highestPriorityPort) {
                        highestPriorityPort = port;
                        bestKey = key;
                    } 
                }

                auto link_it = links.find(bestKey);
                if (highestPriorityPort != libcrtp::CrtpPort::NO_PORT && link_it != links.end()) {
                    libcrtp::CrtpPacket  newPacket;
                    bool success = link_it->second.getPacket(highestPriorityPort, &newPacket);

                    std::cerr << "Success??" << success << "\n";
                    if (success) 
                    {
                        std::cerr << "Packet Channel: " << (int)newPacket.channel << "\n";
                        std::cerr << "Packet first Data: " << (int)newPacket.data[0] << "\n";
                    }
                }
            } while (highestPriorityPort != libcrtp::CrtpPort::NO_PORT);
            
            RCLCPP_WARN(this->get_logger(),"Done Testing");
        }
    private: 
        //void sendCrtpPacketCallback(
        //    const std::shared_ptr<crtp_interface::srv::CrtpPacketSend::Request> request,
        //            std::shared_ptr<crtp_interface::srv::CrtpPacketSend::Response> response)
        //{
        void sendCrtpPacketCallback(
            const std::shared_ptr<rclcpp::Service<crtp_interface::srv::CrtpPacketSend>> service_handle,
            const std::shared_ptr<rmw_request_id_t> header,
            const std::shared_ptr<crtp_interface::srv::CrtpPacketSend::Request> request)
        {
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
            
            //for (int i = 0; i < request->packet.data_length +1; i++ ) RCLCPP_WARN(this->get_logger(),"Data %x", data[i] );
            //RCLCPP_WARN(this->get_logger(),"Datarate: %d", request->datarate);
            //RCLCPP_WARN(this->get_logger(),"Channel: %d", request->channel);
            

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

      
            std::thread t([=](){
                /* ** takes a long time to respond ** */
                using namespace std::chrono_literals;
                std::this_thread::sleep_for(10s);
                auto response = crtp_interface::srv::CrtpPacketSend::Response();
                service_handle->send_response(*header, response);
                });

            t.detach();

            //RCLCPP_WARN(this->get_logger(),"Address %lX", address );
            //    for (int i = 0; i < ack.size; i++) {
            //        RCLCPP_WARN(this->get_logger(),"%x", ack.data[i]);    
            //}

        }
    private:    
        libcrazyradio::Crazyradio m_radio;

        rclcpp::CallbackGroup::SharedPtr crtp_send_callback_group;

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