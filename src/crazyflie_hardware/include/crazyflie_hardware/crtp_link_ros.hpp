#include "rclcpp/rclcpp.hpp"
#include "crtp_cpp/link/crtp_link.hpp"
#include "crtp_interfaces/srv/crtp_packet_send.hpp"

#include "crtp_interfaces/msg/crtp_link.hpp"
#include "crtp_interfaces/msg/crtp_response.hpp"

#include <atomic>
#include <functional>
#include <mutex>
#include <utility>

class RosLink : public CrtpLink
{
public:

    RosLink(
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
        std::function<void()> shutdown_callback,
        int channel,
        std::array<uint8_t, 5> address,
        int datarate);

    bool try_initialize();

    void close_link() override;

    void add_callback(uint8_t port, const CrtpCallbackType& callback) override;

    void send_packet_no_response(CrtpRequest request) override;

    std::optional<CrtpPacket> send_packet(CrtpRequest request) override;
    
    std::vector<CrtpPacket> send_batch_request(const std::vector<CrtpRequest> requests) override;

    std::atomic_bool initialized{false};
private: 
    void fill_crtp_request(std::shared_ptr<crtp_interfaces::srv::CrtpPacketSend::Request> req, const CrtpRequest& request);
    CrtpPacket response_to_packet(std::shared_ptr<crtp_interfaces::srv::CrtpPacketSend::Response> response);
    CrtpPacket ros_packet_to_packet(const crtp_interfaces::msg::CrtpPacket& ros_packet);

    void crtp_link_end_callback(const crtp_interfaces::msg::CrtpLink::SharedPtr msg);
    void crtp_response_callback(const crtp_interfaces::msg::CrtpResponse::SharedPtr msg);



private:
    enum class LinkState {
        Connecting,
        Connected,
        Disconnecting,
        Disconnected,
    };

    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface;
    std::function<void()> shutdown_callback;
    std::string logger_name;
    
    rclcpp::CallbackGroup::SharedPtr callback_group; 
    rclcpp::Client<crtp_interfaces::srv::CrtpPacketSend>::SharedPtr send_crtp_packet_client;

    rclcpp::Subscription<crtp_interfaces::msg::CrtpLink>::SharedPtr link_end_sub;
    rclcpp::Subscription<crtp_interfaces::msg::CrtpResponse>::SharedPtr crtp_response_sub;

    rclcpp::Publisher<crtp_interfaces::msg::CrtpLink>::SharedPtr link_close_pub;

    std::map<uint8_t, std::vector<CrtpCallbackType>> callbacks;

    // Requests originate from several callback groups on a multi-threaded
    // executor. Keep synchronous requests ordered and stop accepting new work
    // as soon as the radio reports that this link has ended.
    std::mutex send_packet_mutex;
    std::atomic<LinkState> state{LinkState::Connecting};
    bool first_request{true};
};
