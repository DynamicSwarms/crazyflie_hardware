#include "crazyflie_hardware_cpp/crtp_link_ros.hpp"


RosLink::RosLink(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, int channel, std::tuple<int> address, int datarate) 
      : CrtpLink(channel, address, datarate)
      , node(node)
    {
      send_crtp_packet_client = node->create_client<crtp_interfaces::srv::CrtpPacketSend>("crazyradio/send_crtp_packet");
    }





void RosLink::send_packet_no_response(CrtpRequest request) 
{
    auto req = std::make_shared<crtp_interfaces::srv::CrtpPacketSend::Request>();
    req->link.channel = channel; 
    req->link.datarate = datarate;
    req->packet.port = request.packet.port;
    req->packet.channel = request.packet.channel;

    for (int i = 0; i < request.packet.data_length; i++) req->packet.data[i] = request.packet.data[i];
    req->packet.data_length = request.packet.data_length;

    send_crtp_packet_client->async_send_request(req);


    RCLCPP_WARN(node->get_logger(), "Sending no response! %d", request.packet.data_length);
}

std::optional<CrtpPacket> RosLink::send_packet(CrtpRequest request)
{
    return CrtpPacket();

}
std::vector<CrtpPacket> RosLink::send_batch_request(const std::vector<CrtpRequest>)
{
    std::vector<CrtpPacket> vec;
    return vec;
}

    std::shared_ptr<rclcpp::Node> node;
