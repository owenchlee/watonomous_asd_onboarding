#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void updateMap(const nav_msgs::msg::OccupancyGrid& costmap, double robot_x, double robot_y);
    nav_msgs::msg::OccupancyGrid getMap() const;

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid global_map_;
    double resolution_;
    int width_;
    int height_;
    double origin_x_;
    double origin_y_;
};

}  

#endif