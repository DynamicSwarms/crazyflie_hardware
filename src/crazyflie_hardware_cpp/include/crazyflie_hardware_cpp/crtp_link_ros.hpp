#include "rclcpp/rclcpp.hpp"
#include "crtp_cpp/link/crtp_link.hpp"
#include "crtp_interfaces/srv/crtp_packet_send.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


class RosLink : public CrtpLink
{
public:
    RosLink(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, int channel, std::tuple<int> address, int datarate);

    void send_packet_no_response(CrtpRequest request) override;

    std::optional<CrtpPacket> send_packet(CrtpRequest request)override;
    
    std::vector<CrtpPacket> send_batch_request(const std::vector<CrtpRequest>)override;

private:
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
    rclcpp::Client<crtp_interfaces::srv::CrtpPacketSend>::SharedPtr send_crtp_packet_client;

};