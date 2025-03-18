#include "crazyflie_hardware_cpp/crtp_link_ros.hpp"
#include <iostream>
#include <chrono>
using std::placeholders::_1;

RosLink::RosLink(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, int channel, std::array<uint8_t, 5> address, int datarate)
    : CrtpLink(channel, address, datarate), node(node)
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

    send_crtp_packet_client->wait_for_service(std::chrono::milliseconds(500));
    while (!send_crtp_packet_client->wait_for_service(std::chrono::milliseconds(500)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(node->get_logger(), "Crazyradio not available, waiting again...");
    }

    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group;
    link_end_sub = node->create_subscription<crtp_interfaces::msg::CrtpLink>(
        "/crazyradio/crtp_link_end",
        10,
        std::bind(&RosLink::crtp_link_end_callback, this, _1),
        sub_opt);

    crtp_response_sub = node->create_subscription<crtp_interfaces::msg::CrtpResponse>(
        "crazyradio/crtp_response",
        10,
        std::bind(&RosLink::crtp_response_callback, this, _1),
        sub_opt);

    auto pub_opt = rclcpp::PublisherOptions();
    pub_opt.callback_group = callback_group;
    link_close_pub = node->create_publisher<crtp_interfaces::msg::CrtpLink>(
        "/crazyradio/close_crtp_link",
        10,
        pub_opt);
}

void RosLink::fill_crtp_request(std::shared_ptr<crtp_interfaces::srv::CrtpPacketSend::Request> req, const CrtpRequest &request)
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

CrtpPacket RosLink::ros_packet_to_packet(const crtp_interfaces::msg::CrtpPacket &ros_packet)
{
    CrtpPacket packet;
    packet.port = ros_packet.port;
    packet.channel = ros_packet.channel;
    packet.data_length = ros_packet.data_length;
    for (int i = 0; i < ros_packet.data_length; i++)
    {
        packet.data[i] = ros_packet.data[i];
    }
    return packet;
}

void RosLink::close_link()
{
    auto msg = crtp_interfaces::msg::CrtpLink();
    msg.channel = channel;
    msg.address = address;
    msg.datarate = datarate;
    link_close_pub->publish(msg);
}

void RosLink::add_callback(uint8_t port, const CrtpCallbackType &callback)
{
    callbacks[port].push_back(callback);
}

void RosLink::crtp_link_end_callback(const crtp_interfaces::msg::CrtpLink::SharedPtr msg)
{
    if (msg->address == address)
    {
        RCLCPP_WARN(node->get_logger(), "Link closed. Shutting down!");
        node->shutdown(); // This works only if we are configured.
    }
}

void RosLink::crtp_response_callback(const crtp_interfaces::msg::CrtpResponse::SharedPtr msg)
{
    if (msg->channel == channel && msg->address == address)
    {
        auto it = callbacks.find(msg->packet.port);
        if (it != callbacks.end())
        {
            for (const auto &callback : it->second)
            {
                CrtpPacket crtp_packet = ros_packet_to_packet(msg->packet);
                callback(crtp_packet);
            }
        }
    }
}

void RosLink::send_packet_no_response(CrtpRequest request)
{
    auto req = std::make_shared<crtp_interfaces::srv::CrtpPacketSend::Request>();
    fill_crtp_request(req, request);
    send_crtp_packet_client->async_send_request(req);

    // RCLCPP_WARN(node->get_logger(), "Sending no response! %d", request.packet.data_length);
}

std::optional<CrtpPacket> RosLink::send_packet(CrtpRequest request)
{
    // RCLCPP_WARN(node->get_logger(), "Sending with response! p: %d, ch: %d, dl: %d, d1: %d er;%d, mb:%d", request.packet.port, request.packet.channel, request.packet.data_length, request.packet.data[0], request.expects_response, request.matching_bytes);

    auto req = std::make_shared<crtp_interfaces::srv::CrtpPacketSend::Request>();
    fill_crtp_request(req, request);

    auto result = send_crtp_packet_client->async_send_request(req);

    using namespace std::chrono_literals;

    auto status = result.wait_for(1s);
    if (status == std::future_status::ready)
    {
        return response_to_packet(result.get());
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Failed single request responded");
        throw std::runtime_error("Failed single request responded!");
    }

    return CrtpPacket();
}
std::vector<CrtpPacket> RosLink::send_batch_request(const std::vector<CrtpRequest> requests)
{
    RCLCPP_WARN(node->get_logger(), "Sending batch! %d", requests.size());
    std::vector<rclcpp::Client<crtp_interfaces::srv::CrtpPacketSend>::SharedFuture> results;

    auto req = std::make_shared<crtp_interfaces::srv::CrtpPacketSend::Request>();
    for (const auto &request : requests)
    {
        fill_crtp_request(req, request);
        auto res = send_crtp_packet_client->async_send_request(req);
        results.push_back(res);
    }

    std::vector<CrtpPacket> response_packets;
    for (const auto &result : results)
    {
        using namespace std::chrono_literals;

        auto status = result.wait_for(1s);
        if (status == std::future_status::ready)
        {
            auto pkt = response_to_packet(result.get());
            response_packets.push_back(pkt);
        }
        else
        {
            RCLCPP_WARN(node->get_logger(), "Failed batch request");
        }
    }
    RCLCPP_WARN(node->get_logger(), "Batch finished with! %d", response_packets.size());
    return response_packets;
}