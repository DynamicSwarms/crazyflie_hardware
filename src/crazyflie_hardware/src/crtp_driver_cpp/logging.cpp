#include "crazyflie_hardware/crtp_driver_cpp/logging.hpp"
#include "crazyflie_hardware/ros_paths.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <cmath>

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
    std::shared_ptr<CrtpLink>link)
    : LoggingLogic(link, crazyflie_hardware::toc_cache_path("logging").string())
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

bool Logging::start_logging_pose()
{
    RCLCPP_WARN(m_logger, "Starting Pose logging.");
    std::vector<std::string> variables = {"stateEstimate.x", "stateEstimate.y", "stateEstimate.z", "stateEstimate.roll", "stateEstimate.pitch", "stateEstimate.yaw"};
    if (!LoggingLogic::add_block(POSE_BLOCK_ID, variables)) return false;
    if (!LoggingLogic::start_block(POSE_BLOCK_ID, 5)) return false; // 20 Hz

    auto publisher_options = rclcpp::PublisherOptions();
    publisher_options.callback_group = m_callback_group;
    log_pose_pub = rclcpp::create_publisher<crazyflie_interfaces::msg::PoseNamedArray>(
        m_topics_interface,
        "/cf_positions",
        rclcpp::QoS(10),
        publisher_options
    );
    log_pose = true;
    return true;
}

bool Logging::start_logging_pm()
{
    RCLCPP_DEBUG(m_logger, "Starting State logging.");
    std::vector<std::string> variables = {"pm.vbat", "pm.chargeCurrent", "pm.state", "sys.canfly", "sys.isFlying", "sys.isTumbled", "stateEstimate.yaw"};
    if (!LoggingLogic::add_block(STATE_BLOCK_ID, variables))
    {
        RCLCPP_ERROR(m_logger, "Failed to create state log block. Variable not found in TOC.");
        return false;
    }

    if (!LoggingLogic::start_block(STATE_BLOCK_ID, 50)) return false; // 2 Hz

    auto publisher_options = rclcpp::PublisherOptions();
    publisher_options.callback_group = m_callback_group;
    log_state_pub = rclcpp::create_publisher<crazyflie_interfaces::msg::LogDataGeneric>(
        m_topics_interface,
        "~/state",
        rclcpp::QoS(10),
        publisher_options
    );
    log_state = true;
    return true;
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
        response->success = LoggingLogic::start_block(next_log_block_id - 1, period_ms_d10);
        if (!response->success) return;
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
        bool stopped = false;
        try {
            stopped = LoggingLogic::stop_block(m_log_block_ids[request->topic_name]);
        } catch (const std::exception &exception) {
            // Link teardown can race this service callback. Convert the CRTP
            // failure into a normal service failure instead of letting an
            // exception escape the executor and terminate the process.
            RCLCPP_WARN(
                m_logger, "Failed to stop log block '%s': %s",
                request->topic_name.c_str(), exception.what());
        }
        if (!stopped) {
            response->success = false;
            return;
        }
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
        uint32_t timestamp = (ts3 << 16) | (ts2 << 8) | ts1; 

        // RCLCPP_WARN(rclcpp::get_logger(logger_name), "Received Block with id %d", block_id);

        std::vector<uint8_t> data_payload(packet.data + 4, packet.data + packet.data_length); // Copy data after the first 4 bytes.

        std::vector<double> values = LoggingLogic::unpack_block(block_id, data_payload);

        // "pm.vbat", "pm.chargeCurrent", "pm.state", "sys.canfly", "sys.isFlying", "sys.isTumbled", "stateEstimate.yaw"
        if (block_id == STATE_BLOCK_ID && log_state && values.size() == 7)
        {
            //  6th value isTumbled
            if ((int)values[5])
            {
                RCLCPP_WARN(m_logger, "System tumbled. Shutting Down");
                if (auto node_shared = m_node.lock()) node_shared->shutdown();
            }

            auto msg = crazyflie_interfaces::msg::LogDataGeneric();
            for (size_t i = 0; i < values.size(); i++) {
                msg.values.push_back(values[i]);
            }
            msg.timestamp = timestamp;
            log_state_pub->publish(msg);
        }
        if (block_id == POSE_BLOCK_ID && log_pose && values.size() == 6)
        {
            auto posearray = crazyflie_interfaces::msg::PoseNamedArray();
            posearray.header.stamp = m_clock_interface->get_clock()->now();
            posearray.header.frame_id = "world";

            
            crazyflie_interfaces::msg::PoseNamed pose;
            pose.header.stamp = m_clock_interface->get_clock()->now();
            pose.name = m_base_interface->get_name();
            pose.rotation_valid = true;
            
            pose.pose.position.x = values[0];
            pose.pose.position.y = values[1];
            pose.pose.position.z = values[2];

            const double roll = values[3] / 180.0 * M_PI; 
            const double pitch = - values[4] / 180.0 * M_PI; // legacy CF2 body frame has pitch inverted
            const double yaw = values[5] / 180.0 * M_PI; 

            const double cy = std::cos(yaw * 0.5);
            const double sy = std::sin(yaw * 0.5);
            const double cp = std::cos(pitch * 0.5);
            const double sp = std::sin(pitch * 0.5);
            const double cr = std::cos(roll * 0.5);
            const double sr = std::sin(roll * 0.5);

            pose.pose.orientation.w = cr * cp * cy + sr * sp * sy;
            pose.pose.orientation.x = sr * cp * cy - cr * sp * sy;
            pose.pose.orientation.y = cr * sp * cy + sr * cp * sy;
            pose.pose.orientation.z = cr * cp * sy - sr * sp * cy;
            posearray.poses.push_back(pose);

            log_pose_pub->publish(posearray);
        }
        if (m_log_blocks.count(block_id))
        {
            m_log_blocks[block_id]->m_publish_log_data(values);
        }
        // if (values.size()) RCLCPP_WARN(rclcpp::get_logger(logger_name), "LogBlock ID:%d , %f", block_id, values[0]);
    }
    // RCLCPP_WARN(rclcpp::get_logger(logger_name), "Logging received a packet with channel %X", packet.channel);
}

bool Logging::initialize_logging()
{
    return LoggingLogic::reset() && initialize_toc(); // Load toc from cf or from file
}

void Logging::download_toc_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    (void)msg;
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
