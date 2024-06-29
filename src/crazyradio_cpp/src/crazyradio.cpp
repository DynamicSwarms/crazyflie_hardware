#define BOOST_BIND_NO_PLACEHOLDERS
#include <chrono>
#include <string.h>


//Ros2
#include "rclcpp/rclcpp.hpp"

#include "libcrazyradio/Crazyradio.hpp"
#include "crtp_interface/srv/crtp_packet_send.hpp"
#include "crtp_interface/msg/crtp_response.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class CrazyradioNode : public rclcpp::Node
{
    public: 
        CrazyradioNode(const rclcpp::NodeOptions & options) : 
            Node("crazyradio_cpp", options),
            m_radio(libcrazyradio::Crazyradio(0))

        {

            send_crtp_packet_service = this->create_service<crtp_interface::srv::CrtpPacketSend>("crazyradio/send_crtp_packet", std::bind(&CrazyradioNode::sendCrtpPacketCallback, this, _1, _2));
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
        }
    private: 
        void sendCrtpPacketCallback(
            const std::shared_ptr<crtp_interface::srv::CrtpPacketSend::Request> request,
                    std::shared_ptr<crtp_interface::srv::CrtpPacketSend::Response> response)
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

            //RCLCPP_WARN(this->get_logger(),"Address %lX", address );
            //    for (int i = 0; i < ack.size; i++) {
            //        RCLCPP_WARN(this->get_logger(),"%x", ack.data[i]);    
            //}

        }
    private:    
        libcrazyradio::Crazyradio m_radio;

        rclcpp::Service<crtp_interface::srv::CrtpPacketSend>::SharedPtr send_crtp_packet_service;
        rclcpp::Publisher<crtp_interface::msg::CrtpResponse>::SharedPtr send_response_pub;
};


int main(int argc, char ** argv) 
{
    (void) argc;
    (void) argv;

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<CrazyradioNode>(options));
    rclcpp::shutdown();
    return 0;
}