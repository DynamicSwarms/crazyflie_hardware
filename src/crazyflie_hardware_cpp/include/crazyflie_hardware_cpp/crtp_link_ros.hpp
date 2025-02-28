#include "rclcpp/rclcpp.hpp"
#include "crtp_cpp/link/crtp_link.hpp"
#include "crtp_interfaces/srv/crtp_packet_send.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


class RosLink : public CrtpLink
{
public:
    RosLink(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, int channel, std::array<uint8_t, 5> address, int datarate);



    void send_packet_no_response(CrtpRequest request) override;

    std::optional<CrtpPacket> send_packet(CrtpRequest request)override;
    
    std::vector<CrtpPacket> send_batch_request(const std::vector<CrtpRequest>)override;
private: 
    void fill_crtp_request(std::shared_ptr<crtp_interfaces::srv::CrtpPacketSend::Request> req, const CrtpRequest& request);
    CrtpPacket response_to_packet(std::shared_ptr<crtp_interfaces::srv::CrtpPacketSend::Response> response);



private:
    rclcpp::CallbackGroup::SharedPtr callback_group; 

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
    rclcpp::Client<crtp_interfaces::srv::CrtpPacketSend>::SharedPtr send_crtp_packet_client;

};