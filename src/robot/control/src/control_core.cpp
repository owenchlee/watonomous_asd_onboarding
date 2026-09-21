#include "control_core.hpp"
#include <cmath>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger), lookahead_distance_(0.5), max_linear_speed_(0.3) {}

double ControlCore::extractYaw(const nav_msgs::msg::Odometry& odom) {
  //x y z w are the components of a quaternion, which is a way to represent 3D rotations
  double x = odom.pose.pose.orientation.x;
  double y = odom.pose.pose.orientation.y;
  double z = odom.pose.pose.orientation.z;
  double w = odom.pose.pose.orientation.w;
  
  //this formula extracts the YAW component from the 3D quaternion
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}


int ControlCore::findLookaheadPoint(const nav_msgs::msg::Path& path, double robot_x, double robot_y) {
  //loop through all points in the path
  for (size_t i = 0; i < path.poses.size(); i++) {
    double px = path.poses[i].pose.position.x;
    double py = path.poses[i].pose.position.y;
    double distance = std::sqrt(std::pow(px - robot_x, 2) + std::pow(py - robot_y, 2));
    //the first point that is further than the lookahead distance is the one we want to steer towards
    if (distance >= lookahead_distance_) {
      return static_cast<int>(i);
    }
  }
  return static_cast<int>(path.poses.size()) - 1;
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(const nav_msgs::msg::Path& path, const nav_msgs::msg::Odometry& odom) {
  geometry_msgs::msg::Twist cmd;

  //edge case: empty path, stop the robot
  if (path.poses.empty()) {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    return cmd;
  }

  //get the robot's current position and orientation from odometry
  double robot_x = odom.pose.pose.position.x;
  double robot_y = odom.pose.pose.position.y;
  double robot_yaw = extractYaw(odom);

  //edge case: robot is close to the final goal, stop
  double final_x = path.poses.back().pose.position.x;
  double final_y = path.poses.back().pose.position.y;
  double dist_to_goal = std::sqrt(std::pow(final_x - robot_x, 2) + std::pow(final_y - robot_y, 2));
  if (dist_to_goal < 0.3) {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    return cmd;
  }

  int idx = findLookaheadPoint(path, robot_x, robot_y);
  double target_x = path.poses[idx].pose.position.x;
  double target_y = path.poses[idx].pose.position.y;

  double dx = target_x - robot_x;
  double dy = target_y - robot_y;

  double target_angle = std::atan2(dy, dx);
  double angle_diff = target_angle - robot_yaw;

  while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
  while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

  //angular velocity proportional to steering angle, capped at a realistic turn rate
  double angular_z = 2.0 * angle_diff;
  if (angular_z > 1.5) angular_z = 1.5;
  if (angular_z < -1.5) angular_z = -1.5;
  cmd.angular.z = angular_z;

  //linear velocity scaled down as steering error grows, so the robot slows to turn sharply
  //instead of driving forward at full speed while spinning
  double heading_error_factor = std::max(0.0, 1.0 - std::abs(angle_diff) / M_PI);
  cmd.linear.x = max_linear_speed_ * heading_error_factor;

  return cmd;
}

}  



