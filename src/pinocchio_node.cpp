#include "ros2_pinocchio_demo/pinocchio_node.hpp"

// Ensure all required headers are included
#include <Eigen/Dense>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

PinocchioNode::PinocchioNode()
: Node("pinocchio_node"), model_loaded_{false}  // Modern initialization
{
  // Subscribe to robot description topic
  robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/robot_description", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      robotDescriptionCallback(msg);
    });
  
  RCLCPP_INFO(this->get_logger(), "Pinocchio node started, waiting for robot description...");
}

void PinocchioNode::robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (!model_loaded_) {
    RCLCPP_INFO(this->get_logger(), "Received robot description");
    setupPinocchio(msg->data);
    testPinocchioOperations();
  }
}

void PinocchioNode::setupPinocchio(const std::string & urdf_string)
{
  try {
    // Build model from URDF string
    pinocchio::urdf::buildModelFromXML(urdf_string, model_);
    
    // Initialize data structure
    data_ = pinocchio::Data(model_);
    
    RCLCPP_INFO(this->get_logger(), "Pinocchio model loaded successfully");
    RCLCPP_INFO(this->get_logger(), "Model has %d joints and %d frames", 
                model_.njoints, model_.nframes);
    
    model_loaded_ = true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load Pinocchio model: %s", e.what());
  }
}

void PinocchioNode::testPinocchioOperations()
{
  if (!model_loaded_) {
    RCLCPP_WARN(this->get_logger(), "Model not loaded, skipping tests");
    return;
  }

  // 1. Create random configuration
  Eigen::VectorXd q = pinocchio::randomConfiguration(model_);
  RCLCPP_INFO(this->get_logger(), "Random configuration generated");

  // 2. Forward Kinematics
  pinocchio::forwardKinematics(model_, data_, q);
  
  // Get transform for first operational frame
  if (model_.nframes > 1) {
    const std::string & frame_name = model_.frames[1].name;
    const pinocchio::FrameIndex frame_id = 1;
    
    pinocchio::updateFramePlacement(model_, data_, frame_id);
    const pinocchio::SE3 & frame_pose = data_.oMf[frame_id];
    
    RCLCPP_INFO(this->get_logger(), "Transform for frame '%s':", frame_name.c_str());
    RCLCPP_INFO(this->get_logger(), "Position: [%.3f, %.3f, %.3f]", 
                frame_pose.translation().x(),
                frame_pose.translation().y(),
                frame_pose.translation().z());
  }

  // 3. Compute Jacobian
  if (model_.nframes > 1) {
    const std::string & frame_name = model_.frames[1].name;
    const pinocchio::FrameIndex frame_id = 1;
    
    Eigen::MatrixXd jacobian(6, model_.nv);
    jacobian.setZero();
    
    pinocchio::computeFrameJacobian(model_, data_, q, frame_id, 
                                  pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
    
    RCLCPP_INFO(this->get_logger(), "Jacobian for frame '%s':", frame_name.c_str());
    RCLCPP_INFO(this->get_logger(), "Jacobian size: %zu x %zu", 
                jacobian.rows(), jacobian.cols());
  }

  // 4. Collision Checking (conceptual)
  if (model_.ncollisionPairs > 1) {
    RCLCPP_INFO(this->get_logger(), 
               "Collision checking would require proper geometry setup");
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PinocchioNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}