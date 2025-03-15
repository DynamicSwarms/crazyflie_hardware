#include "crazyflie_hardware_cpp/crtp_driver_cpp/logging.hpp"
using std::placeholders::_1;

Logging::Logging(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, CrtpLink * link)
    : LoggingLogic(link, std::string("mein_pfad"))
    , node(node)
{
    callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group;

    downdload_toc_sub = node->create_subscription<std_msgs::msg::Empty>(
                "~/download_logging_toc", 
                10,
                std::bind(&Logging::download_toc_callback, this, _1),
                sub_opt);

    get_toc_info_sub = node->create_subscription<std_msgs::msg::Empty>(
                "~/get_logging_toc_info", 
                10,
                std::bind(&Logging::get_toc_info_callback, this, _1),
                sub_opt);

    

    this->initialize_logging();
    RCLCPP_WARN(node->get_logger(), "Logging  initialized");
}

void Logging::crtp_response_callback(const CrtpPacket& packet)
{
    if (packet.channel == LOGDATA_CHANNEL && packet.data_length >= 4) 
    {
        uint8_t block_id = packet.data[0];
        uint8_t ts1 = packet.data[1];
        uint8_t ts2 = packet.data[2];
        uint8_t ts3 = packet.data[3];

        std::vector<uint8_t> data_payload(packet.data + 4, packet.data + packet.data_length); // Copy data after the first 4 bytes.

        std::vector<float> values = LoggingLogic::unpack_block(block_id, data_payload);

        if (values.size()) RCLCPP_WARN(node->get_logger(), "LogBlock ID:%d , %f", block_id, values[0]);

    }
    RCLCPP_WARN(node->get_logger(), "Logging received a packet with channel %X", packet.channel);
}

void Logging::initialize_logging()
{   
    LoggingLogic::reset();
    initialize_toc(); // Load toc from cf or from file

    std::vector<std::string> variables = {"pm.vbat"};
    LoggingLogic::add_block(1, variables);
    LoggingLogic::start_block(1, 100); // 1Hz
}



void Logging::download_toc_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    LoggingLogic::send_download_toc_items();
    LoggingLogic::write_to_file();

    //auto [nbr_of_items, crc] = ParametersLogic::send_get_toc_info();
    //bool success = ParametersLogic::load_from_file(crc);
    //RCLCPP_WARN(node->get_logger(), "%d", success);
}

void Logging::get_toc_info_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    auto [nbr_of_items, crc] = LoggingLogic::send_get_toc_info();
    RCLCPP_WARN(node->get_logger(), "%d, %X", nbr_of_items, crc);

}