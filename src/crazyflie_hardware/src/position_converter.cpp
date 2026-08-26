#include <memory>
#include <utility>

#include "crazyflie_interfaces/msg/pose_named.hpp"
#include "crazyflie_interfaces/msg/pose_named_array.hpp"
#include "object_tracker_interfaces/msg/position_named_array.hpp"
#include "rclcpp/rclcpp.hpp"

class PositionConverter : public rclcpp::Node
{
public:
  PositionConverter()
  : Node("position_converter")
  {
    publisher_ = create_publisher<crazyflie_interfaces::msg::PoseNamedArray>(
      "/cf_positions", rclcpp::QoS(10));
    subscription_ = create_subscription<object_tracker_interfaces::msg::PositionNamedArray>(
      "/tracker/positions", rclcpp::QoS(10),
      [this](const object_tracker_interfaces::msg::PositionNamedArray::SharedPtr msg) {
        convert_and_publish(*msg);
      });
  }

private:
  void convert_and_publish(
    const object_tracker_interfaces::msg::PositionNamedArray & input)
  {
    crazyflie_interfaces::msg::PoseNamedArray output;
    output.header = input.header;
    output.poses.reserve(input.positions.size());

    for (const auto & position : input.positions) {
      crazyflie_interfaces::msg::PoseNamed pose;
      pose.name = position.name;
      pose.rotation_valid = false;
      pose.header = position.header;
      pose.pose.position = position.position;
      pose.pose.orientation.w = 1.0;
      output.poses.push_back(std::move(pose));
    }

    publisher_->publish(output);
  }

  rclcpp::Publisher<crazyflie_interfaces::msg::PoseNamedArray>::SharedPtr publisher_;
  rclcpp::Subscription<object_tracker_interfaces::msg::PositionNamedArray>::SharedPtr
    subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionConverter>());
  rclcpp::shutdown();
  return 0;
}
