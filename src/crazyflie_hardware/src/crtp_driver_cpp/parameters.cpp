#include "crazyflie_hardware/crtp_driver_cpp/parameters.hpp"
using std::placeholders::_1;
using std::placeholders::_2;

Parameters::Parameters(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_parameters_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
    std::shared_ptr<CrtpLink>link)
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

    m_get_firmware_parameters_service = rclcpp::create_service<rcl_interfaces::srv::GetParameters>(
        node_base_interface,
        node_services_interface,
        "~/get_firmware_parameters",
        std::bind(&Parameters::m_get_firmware_parameters_callback, this, _1, _2),
        rmw_qos_profile_services_default,
        m_callback_group);

    RCLCPP_DEBUG(m_logger, "Parameters  initialized");
}

void Parameters::m_get_firmware_parameters_callback(
    const rcl_interfaces::srv::GetParameters::Request::SharedPtr request,
    rcl_interfaces::srv::GetParameters::Response::SharedPtr response)
{
    response->values.reserve(request->names.size());
    for (const auto &full_name : request->names)
    {
        rcl_interfaces::msg::ParameterValue value;
        const size_t dot = full_name.find('.');
        if (dot == std::string::npos)
        {
            response->values.push_back(value);
            continue;
        }

        try
        {
            auto firmware_value = ParametersLogic::send_get_parameter(
                full_name.substr(0, dot), full_name.substr(dot + 1));
            if (firmware_value)
            {
                if (std::holds_alternative<int64_t>(*firmware_value))
                {
                    value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
                    value.integer_value = std::get<int64_t>(*firmware_value);
                }
                else
                {
                    value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
                    value.double_value = std::get<double>(*firmware_value);
                }
            }
        }
        catch (const std::exception &exception)
        {
            RCLCPP_WARN(m_logger, "Could not read firmware parameter '%s': %s",
                full_name.c_str(), exception.what());
        }
        response->values.push_back(value);
    }
}

bool Parameters::initialize_parameters()
{
    if (!this->initialize_toc()) return false; // Load toc from cf or from file

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
    return true;
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
