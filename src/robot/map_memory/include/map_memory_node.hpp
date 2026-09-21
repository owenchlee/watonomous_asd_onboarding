#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap);
    void odomCallBack(const nav_msgs::msg::Odometry::SharedPtr odom);
    void updateMap();

  private:
    robot::MapMemoryCore map_memory_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_memory_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double last_x_;
    double last_y_;
    double last_yaw_;
    bool should_update_map_;
    bool has_odom_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;
};

#endif 
