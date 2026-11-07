#pragma once

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"

// Interfaces needed to create the gateway's lifecycle service clients.
struct GatewayNodeInterfaces
{
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics;
};
