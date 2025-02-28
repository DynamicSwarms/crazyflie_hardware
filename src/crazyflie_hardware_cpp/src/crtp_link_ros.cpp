#include "crazyflie_hardware_cpp/crtp_link_ros.hpp"
#include <iostream>
#include <chrono>



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

    req->expects_response = request.expects_response; 
    req->matching_bytes = request.matching_bytes; 
}

CrtpPacket RosLink::response_to_packet(std::shared_ptr<crtp_interfaces::srv::CrtpPacketSend::Response> response)
{
    CrtpPacket packet;
    packet.port = response->packet.port;
    packet.channel = response->packet.channel;
    packet.data_length = response->packet.data_length;

    for (int i = 0; i < response->packet.data_length; i++) 
    {
        packet.data[i] = response->packet.data[i];
    }
    return packet;
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
    RCLCPP_WARN(node->get_logger(), "Sending with response! p: %d, ch: %d, dl: %d, d1: %d er;%d, mb:%d",request.packet.port, request.packet.channel, request.packet.data_length, request.packet.data[0], request.expects_response, request.matching_bytes);

    auto req = std::make_shared<crtp_interfaces::srv::CrtpPacketSend::Request>();
    fill_crtp_request(req, request);

    auto result = send_crtp_packet_client->async_send_request(req);

    using namespace std::chrono_literals;

    auto status = result.wait_for(3s);  //not spinning here!
    if (status == std::future_status::ready)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Success");
        auto rr = result.get();
        auto pkt = response_to_packet(rr);
        auto r = rr->packet;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "p:%d,ch:%d, dl:%d, d1:%d, d2:%d, d3:%d", r.port, r.channel, r.data_length, r.data[0], r.data[1], r.data[2] );
        return pkt;

    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed single request responded");
    }

    return CrtpPacket();
}
std::vector<CrtpPacket> RosLink::send_batch_request(const std::vector<CrtpRequest> requests)
{ 
    RCLCPP_WARN(node->get_logger(), "Sending batch! %d", requests.size());
    std::vector<rclcpp::Client<crtp_interfaces::srv::CrtpPacketSend>::SharedFuture> results;

    auto req = std::make_shared<crtp_interfaces::srv::CrtpPacketSend::Request>();
    for (const auto& request : requests) {
        fill_crtp_request(req, request);
        auto res = send_crtp_packet_client->async_send_request(req);
        results.push_back(res);
    }

    std::vector<CrtpPacket> response_packets;
    for (const auto& result : results) {
        using namespace std::chrono_literals;

        auto status = result.wait_for(3s);  //not spinning here!
        if (status == std::future_status::ready)
        {
            auto pkt = response_to_packet(result.get());
            response_packets.push_back(pkt);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed in batch request");
        }
    }
    RCLCPP_WARN(node->get_logger(), "Batch finished with! %d", response_packets.size());

    return response_packets;
}

    std::shared_ptr<rclcpp::Node> node;
