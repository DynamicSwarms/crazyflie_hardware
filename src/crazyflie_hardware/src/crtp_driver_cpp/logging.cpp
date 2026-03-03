#include "crazyflie_hardware/crtp_driver_cpp/logging.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

#define STATE_BLOCK_ID 0
#define POSE_BLOCK_ID 1

Logging::Logging(
    std::weak_ptr<rclcpp_lifecycle::LifecycleNode> node,
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface, 
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
    CrtpLink *link)
    : LoggingLogic(link, std::string("mein_pfad"))
    , m_node(node)
    , m_base_interface(node_base_interface)
    , m_topics_interface(node_topics_interface)
    , m_logger(node_logging_interface->get_logger().get_child("Logging"))
    , m_timers_interface(node_timers_interface)
    , m_clock_interface(node_clock_interface)
    , m_callback_group(node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
    , log_state(false)
    , log_pose(false)
    , next_log_block_id(std::max(STATE_BLOCK_ID, POSE_BLOCK_ID) + 1)
{
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = m_callback_group;

    downdload_toc_sub = rclcpp::create_subscription<std_msgs::msg::Empty>(
        node_topics_interface,
        "~/download_logging_toc",
        10,
        std::bind(&Logging::download_toc_callback, this, _1),
        sub_opt);

    get_toc_info_sub = rclcpp::create_subscription<std_msgs::msg::Empty>(
        node_topics_interface,
        "~/get_logging_toc_info",
        10,
        std::bind(&Logging::get_toc_info_callback, this, _1),
        sub_opt);

    m_add_log_block_server = rclcpp::create_service<crazyflie_interfaces::srv::AddLogging>(
        node_base_interface,
        node_services_interface,
        "~/add_logging",
        std::bind(&Logging::m_add_log_block_service, this, _1, _2),
        rmw_qos_profile_services_default,
        m_callback_group
    );

    m_remove_log_block_server = rclcpp::create_service<crazyflie_interfaces::srv::RemoveLogging>(
        node_base_interface,
        node_services_interface,
        "~/remove_logging",
        std::bind(&Logging::m_remove_log_block_service, this, _1, _2),
        rmw_qos_profile_services_default,
        m_callback_group
    );

    RCLCPP_DEBUG(m_logger, "Logging  initialized");
}

void Logging::start_logging_pose()
{
    RCLCPP_WARN(m_logger, "Starting Pose logging.");
    std::vector<std::string> variables = {"stateEstimate.x", "stateEstimate.y", "stateEstimate.z", "stateEstimateZ.quat"};
    LoggingLogic::add_block(POSE_BLOCK_ID, variables);
    LoggingLogic::start_block(POSE_BLOCK_ID, 5); // 20 Hz

    auto publisher_options = rclcpp::PublisherOptions();
    publisher_options.callback_group = m_callback_group;
    log_pose_pub = rclcpp::create_publisher<crazyflie_interfaces::msg::PoseStampedArray>(
        m_topics_interface,
        "/cf_positions",
        rclcpp::QoS(10),
        publisher_options
    );
    log_pose = true;
}

void Logging::start_logging_pm()
{
    RCLCPP_DEBUG(m_logger, "Starting State logging.");
    std::vector<std::string> variables = {"pm.vbat", "pm.chargeCurrent", "pm.state", "sys.canfly", "sys.isFlying", "sys.isTumbled"};
    LoggingLogic::add_block(STATE_BLOCK_ID, variables);

    LoggingLogic::start_block(STATE_BLOCK_ID, 50); // 2 Hz

    auto publisher_options = rclcpp::PublisherOptions();
    publisher_options.callback_group = m_callback_group;
    log_state_pub = rclcpp::create_publisher<crazyflie_interfaces::msg::LogDataGeneric>(
        m_topics_interface,
        "~/state",
        rclcpp::QoS(10),
        publisher_options
    );
    log_state = true;
}

void 
Logging::m_add_log_block_service(
    const std::shared_ptr<crazyflie_interfaces::srv::AddLogging::Request> request,
    std::shared_ptr<crazyflie_interfaces::srv::AddLogging::Response> response
)
{
    RCLCPP_INFO(m_logger, "Received request to add log block: %s", request->topic_name.c_str());
    if (m_create_log_block(request->topic_name, request->vars))
    {
        int period_ms_d10 = 1000 / request->frequency;
        period_ms_d10 /= 10; // convert ms to d10ms
        LoggingLogic::start_block(next_log_block_id - 1, period_ms_d10);
        response->success = true;
        RCLCPP_INFO(m_logger, "Successfully created log block: %s", request->topic_name.c_str());
    }
    else
    {
        response->success = false;
        RCLCPP_ERROR(m_logger, "Failed to create log block: %s.", request->topic_name.c_str());
    }
}

void 
Logging::m_remove_log_block_service(
    const std::shared_ptr<crazyflie_interfaces::srv::RemoveLogging::Request> request,
    std::shared_ptr<crazyflie_interfaces::srv::RemoveLogging::Response> response
)
{
    RCLCPP_INFO(m_logger, "Received request to remove log block: %s", request->topic_name.c_str());
    if (m_log_block_ids.count(request->topic_name)) {
        LoggingLogic::stop_block(m_log_block_ids[request->topic_name]);
        m_log_blocks.erase(m_log_block_ids[request->topic_name]);
        m_log_block_ids.erase(request->topic_name);
        response->success = true;
    } else {
        response->success = false;
    }
}

bool
Logging::m_create_log_block(
    const std::string &block_name,
    const std::vector<std::string> &variables)
{   
    if (m_log_block_ids.count(block_name))
    {
        RCLCPP_ERROR(m_logger, "Log block with name: %s already exists.", block_name.c_str());
        return false;
    }

    if (!LoggingLogic::add_block(next_log_block_id, variables))
    {
        RCLCPP_ERROR(m_logger, "Failed to create log block with name: %s. Variable not found in TOC.", block_name.c_str());
        return false;
    }

    m_log_blocks[next_log_block_id] = std::make_shared<LogBlock>(
        m_base_interface,
        m_topics_interface,
        m_logger,
        m_timers_interface,
        m_callback_group,
        block_name
    );
    m_log_block_ids[block_name] = next_log_block_id;
    next_log_block_id++;

    return true;
    RCLCPP_INFO(m_logger, "Created log block with name: %s", block_name.c_str());
}

void Logging::crtp_response_callback(const CrtpPacket &packet)
{
    if (packet.channel == CONTROL_CHANNEL)
    {
        RCLCPP_WARN(m_logger, "Received Control Packet: %d", packet.data[0]);
        // Should never receive because it is a responed packet.
    }
    if (packet.channel == LOGDATA_CHANNEL && packet.data_length >= 4)
    {
        uint8_t block_id = packet.data[0];
        uint8_t ts1 = packet.data[1];
        uint8_t ts2 = packet.data[2];
        uint8_t ts3 = packet.data[3];

        // RCLCPP_WARN(rclcpp::get_logger(logger_name), "Received Block with id %d", block_id);

        std::vector<uint8_t> data_payload(packet.data + 4, packet.data + packet.data_length); // Copy data after the first 4 bytes.

        std::vector<float> values = LoggingLogic::unpack_block(block_id, data_payload);
        std::vector<double> double_values(values.begin(), values.end());

        if (block_id == STATE_BLOCK_ID && log_state && values.size() == 6)
        {
            // RCLCPP_WARN(rclcpp::get_logger(logger_name), "%f, %f, %f, %f", values[3], values[4], values[5], values[6]);
            //  Values 5 is tumbled
            if ((int)values[5])
            {
                RCLCPP_WARN(m_logger, "System tumbled. Shutting Down");
                if (auto node_shared = m_node.lock()) node_shared->shutdown();
            }

            auto msg = crazyflie_interfaces::msg::LogDataGeneric();
            for (size_t i = 0; i < values.size(); i++) {
                msg.values.push_back(values[i]);
            }
            log_state_pub->publish(msg);
        }
        if (block_id == POSE_BLOCK_ID && log_pose && values.size() == 4)
        {
            auto posearray = crazyflie_interfaces::msg::PoseStampedArray();
            float q[4];
            quatdecompress(values[3], q);
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = m_clock_interface->get_clock()->now();
            pose.header.frame_id = m_base_interface->get_name();

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
        if (m_log_blocks.count(block_id))
        {
            m_log_blocks[block_id]->m_publish_log_data(double_values);
        }
        // if (values.size()) RCLCPP_WARN(rclcpp::get_logger(logger_name), "LogBlock ID:%d , %f", block_id, values[0]);
    }
    // RCLCPP_WARN(rclcpp::get_logger(logger_name), "Logging received a packet with channel %X", packet.channel);
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
    // RCLCPP_WARN(rclcpp::get_logger(logger_name), "%d", success);
}

void Logging::get_toc_info_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    (void)msg;
    auto [nbr_of_items, crc] = LoggingLogic::send_get_toc_info();
    RCLCPP_WARN(m_logger, "%d, %X", nbr_of_items, crc);
}