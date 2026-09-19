#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

//copies the contents of the file, the words in quotations is the path to that file
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
  
    // Place callback function here
    // function called lasercallback and it takes a value (takes a pointer to a laserscan message),
    // names it scan and it cannot be changed in the function, also returns nothing
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    

  private:
    robot::CostmapCore costmap_;
    // Place these constructs here
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
};
#endif
