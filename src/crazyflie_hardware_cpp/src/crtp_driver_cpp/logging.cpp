#include "crazyflie_hardware_cpp/crtp_driver_cpp/logging.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;

#define PM_BLOCK_ID 0
#define POSE_BLOCK_ID 1

Logging::Logging(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, CrtpLink *link)
    : LoggingLogic(link, std::string("mein_pfad")), node(node), log_pm(false), log_pose(false)
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

    RCLCPP_DEBUG(node->get_logger(), "Logging  initialized");
}

void Logging::start_logging_pose()
{
    RCLCPP_WARN(node->get_logger(), "Starting Pose logging.");
    std::vector<std::string> variables = {"stateEstimate.x", "stateEstimate.y", "stateEstimate.z", "stateEstimateZ.quat"};
    LoggingLogic::add_block(POSE_BLOCK_ID, variables);

    LoggingLogic::start_block(POSE_BLOCK_ID, 10); // 10 Hz

    log_pose_pub = node->create_publisher<crazyflie_interfaces::msg::PoseStampedArray>("/cf_positions", 10);
    log_pose = true;
}

void Logging::start_logging_pm()
{
    RCLCPP_WARN(node->get_logger(), "Starting PM logging.");
    std::vector<std::string> variables = {"pm.vbat", "pm.chargeCurrent", "pm.state"};
    LoggingLogic::add_block(PM_BLOCK_ID, variables);

    LoggingLogic::start_block(PM_BLOCK_ID, 100); // 1 Hz

    log_pm_pub = node->create_publisher<crazyflie_interfaces::msg::GenericLogData>("~/pm", 10);
    log_pm = true;
}

void Logging::crtp_response_callback(const CrtpPacket &packet)
{
    if (packet.channel == CONTROL_CHANNEL)
    {
        RCLCPP_WARN(node->get_logger(), "Received Control Packet: %d", packet.data[0]);
        // Should never receive because it is a responed packet.
    }
    if (packet.channel == LOGDATA_CHANNEL && packet.data_length >= 4)
    {
        uint8_t block_id = packet.data[0];
        uint8_t ts1 = packet.data[1];
        uint8_t ts2 = packet.data[2];
        uint8_t ts3 = packet.data[3];

        // RCLCPP_WARN(node->get_logger(), "Received Block with id %d", block_id);

        std::vector<uint8_t> data_payload(packet.data + 4, packet.data + packet.data_length); // Copy data after the first 4 bytes.

        std::vector<float> values = LoggingLogic::unpack_block(block_id, data_payload);
        std::vector<double> double_values(values.begin(), values.end());

        if (block_id == PM_BLOCK_ID && log_pm && values.size() == 3)
        {
            auto msg = crazyflie_interfaces::msg::GenericLogData();
            msg.values = double_values;
            log_pm_pub->publish(msg);
        }
        if (block_id == POSE_BLOCK_ID && log_pose && values.size() == 4)
        {
            auto posearray = crazyflie_interfaces::msg::PoseStampedArray();
            float q[4];
            quatdecompress(values[3], q);
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = node->get_clock()->now();
            pose.header.frame_id = node->get_name();

            pose.pose.position.x = values[0];
            pose.pose.position.y = values[1];
            pose.pose.position.z = values[2];

            pose.pose.orientation.x = q[0];
            pose.pose.orientation.y = q[1];
            pose.pose.orientation.z = q[2];
            pose.pose.orientation.w = q[3];
            posearray.poses.push_back(pose);

            log_pose_pub->publish(posearray);
        }
        // if (values.size()) RCLCPP_WARN(node->get_logger(), "LogBlock ID:%d , %f", block_id, values[0]);
    }
    // RCLCPP_WARN(node->get_logger(), "Logging received a packet with channel %X", packet.channel);
}

void Logging::initialize_logging()
{
    LoggingLogic::reset();
    initialize_toc(); // Load toc from cf or from file
}

void Logging::download_toc_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    LoggingLogic::send_download_toc_items();
    LoggingLogic::write_to_file();

    // auto [nbr_of_items, crc] = ParametersLogic::send_get_toc_info();
    // bool success = ParametersLogic::load_from_file(crc);
    // RCLCPP_WARN(node->get_logger(), "%d", success);
}

void Logging::get_toc_info_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    auto [nbr_of_items, crc] = LoggingLogic::send_get_toc_info();
    RCLCPP_WARN(node->get_logger(), "%d, %X", nbr_of_items, crc);
}