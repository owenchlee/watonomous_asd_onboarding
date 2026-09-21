#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

class ControlCore {
  public:
    ControlCore(const rclcpp::Logger& logger);
    //the main function of control
    //returns Twist which is a linear and angular velocity command for the robot
    geometry_msgs::msg::Twist computeVelocity(const nav_msgs::msg::Path& path, const nav_msgs::msg::Odometry& odom);
  
  private:
    rclcpp::Logger logger_;
    double lookahead_distance_;
    double max_linear_speed_;

    //converts to a yaw angle which is usable for robot steering math
    double extractYaw(const nav_msgs::msg::Odometry& odom);
    int findLookaheadPoint(const nav_msgs::msg::Path& path, double robot_x, double robot_y);
};

}

#endif
