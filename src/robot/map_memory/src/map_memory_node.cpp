#include "map_memory_node.hpp"
#include <cmath>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())), last_x_(0.0), last_y_(0.0), should_update_map_(false) {
  // Initialize the constructs and their parameters
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallBack, this, std::placeholders::_1));
  map_memory_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
  map_memory_pub_->publish(map_memory_.getMap());
}

void MapMemoryNode::odomCallBack(const nav_msgs::msg::Odometry::SharedPtr odom) {
  // Get the current position from the odometry message
  double x = odom->pose.pose.position.x;
  double y = odom->pose.pose.position.y;

  //update map if the robot has moved more than 1.5 meters from the last position
  double distance = std::sqrt(std::pow(x - last_x_, 2) + std::pow(y - last_y_, 2));
  if (distance >= 1.5) {
    last_x_ = x;
    last_y_ = y;
    should_update_map_ = true;
  }
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
  latest_costmap_ = *costmap;
}

void MapMemoryNode::updateMap() {
  if (should_update_map_) {
    //pass in the most rcent costmap and the robots position when it crossed the 1.5m threshold
    map_memory_.updateMap(latest_costmap_, last_x_, last_y_);
    //publish to map
    map_memory_pub_->publish(map_memory_.getMap());
    should_update_map_ = false;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}



