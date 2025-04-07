#ifndef ROS2_PINOCCHIO_DEMO__PINOCCHIO_NODE_HPP_
#define ROS2_PINOCCHIO_DEMO__PINOCCHIO_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// Pinocchio headers
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/collision.hpp>

class PinocchioNode : public rclcpp::Node
{
public:
  PinocchioNode();  // Constructor

private:
  // Callback for robot description
  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg);
  
  // Setup Pinocchio model from URDF
  void setupPinocchio(const std::string & urdf_string);
  
  // Demonstrate Pinocchio operations
  void testPinocchioOperations();

  // ROS 2 Subscription
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  
  // Pinocchio model and data
  pinocchio::Model model_;
  pinocchio::Data data_;
  
  bool model_loaded_{false};  // Modern C++ initialization
};

#endif  // ROS2_PINOCCHIO_DEMO__PINOCCHIO_NODE_HPP_