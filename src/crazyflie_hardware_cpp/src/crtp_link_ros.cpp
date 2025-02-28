#include "crazyflie_hardware_cpp/crtp_link_ros.hpp"


RosLink::RosLink(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, int channel, std::array<uint8_t, 5> address, int datarate) 
      : CrtpLink(channel, address, datarate)
      , node(node)
    {
      callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      auto qos = rclcpp::QoS(500);
      qos.reliable();
      qos.keep_all();
      qos.durability_volatile();


      send_crtp_packet_client = node->create_client<crtp_interfaces::srv::CrtpPacketSend>(
            "crazyradio/send_crtp_packet", 
            qos.get_rmw_qos_profile(),
            callback_group);
    }



void RosLink::fill_crtp_request(std::shared_ptr<crtp_interfaces::srv::CrtpPacketSend::Request> req, const CrtpRequest& request)
{
    req->link.channel = channel; 
    req->link.address = address;
    req->link.datarate = datarate;
    req->packet.port = request.packet.port;
    req->packet.channel = request.packet.channel;
    for (int i = 0; i < request.packet.data_length; i++) 
    {
        req->packet.data[i] = request.packet.data[i];
    }
    req->packet.data_length = request.packet.data_length;
}


void RosLink::send_packet_no_response(CrtpRequest request) 
{
    auto req = std::make_shared<crtp_interfaces::srv::CrtpPacketSend::Request>();
    fill_crtp_request(req, request);
    send_crtp_packet_client->async_send_request(req);

    RCLCPP_WARN(node->get_logger(), "Sending no response! %d", request.packet.data_length);
}

std::optional<CrtpPacket> RosLink::send_packet(CrtpRequest request)
{
    auto req = std::make_shared<crtp_interfaces::srv::CrtpPacketSend::Request>();
    fill_crtp_request(req, request);

    RCLCPP_WARN(node->get_logger(), "Sending with response! %d", request.packet.data_length);
    return CrtpPacket();
}
std::vector<CrtpPacket> RosLink::send_batch_request(const std::vector<CrtpRequest>)
{
    std::vector<CrtpPacket> vec;
    return vec;
}

    std::shared_ptr<rclcpp::Node> node;
