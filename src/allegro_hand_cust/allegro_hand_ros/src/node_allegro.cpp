
#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "allegro_hand_ros/allegro_hand_ros.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto hand_node = std::make_shared<rclcpp::Node>("my_hand");

  // hand_node->declare_parameter("hand_prefix", rclcpp::PARAMETER_STRING);
  // const std::string hand_prefix = hand_node->get_parameter("hand_prefix").as_string();
    // Link link_prefix parameter
  auto link_prefix_desc = rcl_interfaces::msg::ParameterDescriptor{};
  link_prefix_desc.description = "Link link_prefix parameter";
  hand_node->declare_parameter("link_prefix", "", link_prefix_desc);
  const std::string hand_prefix = hand_node->get_parameter("link_prefix").as_string();
  RCLCPP_INFO_STREAM(hand_node->get_logger(), "Hand prefix: " << hand_prefix);
  if (hand_prefix.empty()) {
      RCLCPP_WARN_STREAM(hand_node->get_logger(), "No prefix provided");
  }

  auto allegro_ros = std::make_shared<AllegroRos>(hand_node->shared_from_this(),hand_prefix);

  // auto allegro_ros_node = std::make_shared<AllegroRos>();
  // rclcpp::spin(allegro_ros_node);
  // std::cout << "Starting loop function node" << std::endl;
  // allegro_ros_node->loop_function();

  // rclcpp::spin(hand_node);
  std::cout << "Starting loop function node" << std::endl;
  allegro_ros->loop_function();

  rclcpp::shutdown();
  
  return 0;
}