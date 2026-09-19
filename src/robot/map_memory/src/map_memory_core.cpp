#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger), resolution_(0.1), width_(300), height_(300), origin_x_(-15.0), origin_y_(-15.0)
{
  //fill in global_map_'s info 
  global_map_.info.resolution = resolution_;
  global_map_.info.width = width_;
  global_map_.info.height = height_;
  global_map_.info.origin.position.x = origin_x_;
  global_map_.info.origin.position.y = origin_y_;
  //sets values to -1, which represents unknown space in the occupancy grid
  global_map_.data.assign(width_ * height_, -1);
}

void MapMemoryCore::updateMap(const nav_msgs::msg::OccupancyGrid& costmap, double robot_x, double robot_y) {
  //pulls in data out of costmap's info fields
  int costmap_width = costmap.info.width;
  int costmap_height = costmap.info.height;
  double costmap_resolution = costmap.info.resolution;
  double costmap_origin_x = costmap.info.origin.position.x;
  double costmap_origin_y = costmap.info.origin.position.y;

  //loops every cell of the incoming costmap
  for (int y = 0; y < costmap_height; y++) {
    for (int x = 0; x < costmap_width; x++) {
      //change the 2d coordinates into a 1d index to access the data vector
      int8_t value = costmap.data[y * costmap_width + x];
      //if unknown space "-1 value", skip it
      if (value < 0) continue;
      
      //determine how far away the cell is from the robot in meters
      double local_x = costmap_origin_x + x * costmap_resolution;
      double local_y = costmap_origin_y + y * costmap_resolution;

      //where is point in the actual world?
      double world_x = robot_x + local_x;
      double world_y = robot_y + local_y;

      //which cell in my map array does this world position correspond to?
      int global_x = static_cast<int>((world_x - origin_x_) / resolution_);
      int global_y = static_cast<int>((world_y - origin_y_) / resolution_);

      //if the cell is within the bounds of the global map, update it
      if (global_x >= 0 && global_x < width_ && global_y >= 0 && global_y < height_) {
        global_map_.data[global_y * width_ + global_x] = value;
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getMap() const {
  return global_map_;
}

} 
