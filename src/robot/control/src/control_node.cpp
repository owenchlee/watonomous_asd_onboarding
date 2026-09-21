#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr path) {
  current_path_ = *path;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
  current_odom_ = *odom;
}

void ControlNode::controlLoop() {
  if (current_path_.poses.empty()) {
    //stop the robot, otherwise gazebo just keeps running the last cmd_vel forever
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }
  geometry_msgs::msg::Twist cmd = control_.computeVelocity(current_path_, current_odom_);
  cmd_vel_pub_->publish(cmd);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}