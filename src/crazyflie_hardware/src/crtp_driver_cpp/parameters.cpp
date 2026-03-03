#include "crazyflie_hardware/crtp_driver_cpp/parameters.hpp"
using std::placeholders::_1;

Parameters::Parameters(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_parameters_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
    CrtpLink *link)
    : ParametersLogic(link, std::string("mein_pfad"))
    , m_parameters_interface(node_parameters_interface)
    , m_logger(node_logging_interface->get_logger().get_child("Parameters"))
    , m_callback_group(node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
{
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = m_callback_group;

    m_downdload_toc_sub = rclcpp::create_subscription<std_msgs::msg::Empty>(
        node_topics_interface,
        "~/download_parameters_toc",
        10,
        std::bind(&Parameters::m_download_toc_callback, this, _1),
        sub_opt);

    m_get_toc_info_sub = rclcpp::create_subscription<std_msgs::msg::Empty>(
        node_topics_interface,
        "~/get_parameters_toc_info",
        10,
        std::bind(&Parameters::m_get_toc_info_callback, this, _1),
        sub_opt);

    RCLCPP_DEBUG(m_logger, "Parameters  initialized");
}

void Parameters::initialize_parameters()
{
    this->initialize_toc(); // Load toc from cf or from file

    for (const auto &entry : ParametersLogic::toc_entries)
    {
        auto group = entry.group;
        auto name = entry.name;
        std::ostringstream ss;
        ss << group << "." << name;
        if (entry.isInteger())
        {
            m_parameters_interface->declare_parameter(ss.str(), rclcpp::PARAMETER_INTEGER);
        }
        else if (entry.isDouble())
        {
            m_parameters_interface->declare_parameter(ss.str(), rclcpp::PARAMETER_DOUBLE);
        }
    }
    
    m_param_callback_handle = m_parameters_interface->add_on_set_parameters_callback(std::bind(&Parameters::m_set_parameter_callback, this, std::placeholders::_1));    
    
}

rcl_interfaces::msg::SetParametersResult Parameters::m_set_parameter_callback(const std::vector<rclcpp::Parameter> &parameters)
{
    // This gets called if a parameter gets set. We want to set it on the crazyflie as well.

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters)
    {
        std::string param_name = param.get_name();
        size_t dot = param_name.find('.');

        if (dot != std::string::npos)
        {
            std::string group = param_name.substr(0, dot);
            std::string name = param_name.substr(dot + 1);

            if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                result.successful = ParametersLogic::send_set_parameter(group, name, std::variant<int, double>((int)param.as_int()));
            }
            else if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                result.successful = ParametersLogic::send_set_parameter(group, name, std::variant<int, double>(param.as_double()));
            }
        }

        if (!result.successful)
            return result;
    }
    return result;
}

void Parameters::m_download_toc_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    (void)msg;
    ParametersLogic::send_download_toc_items();
    ParametersLogic::write_to_file();

    // auto [nbr_of_items, crc] = ParametersLogic::send_get_toc_info();
    // bool success = ParametersLogic::load_from_file(crc);
    // RCLCPP_WARN(node->get_logger(), "%d", success);
}

void Parameters::m_get_toc_info_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    (void)msg;
    auto [nbr_of_items, crc] = ParametersLogic::send_get_toc_info();
    RCLCPP_WARN(m_logger, "%d, %X", nbr_of_items, crc);
}