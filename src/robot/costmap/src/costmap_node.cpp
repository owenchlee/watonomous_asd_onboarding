#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters

  //get information from lidar, and publish the costmap
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}
 
// Add the lasercallback function here
void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  //runs once for every distance
  for (size_t i = 0; i < scan->ranges.size(); i++) {
    //calculate the specific angle of that reading
    double angle = scan->angle_min + i * scan->angle_increment;
    //get the distance of that reading
    double range = scan->ranges[i];

    int x_grid, y_grid;
    costmap_.convertToGrid(range, angle, x_grid, y_grid);
    costmap_.markObstacle(x_grid, y_grid);
  }
  //run inflate obstales after all obstacles have been marked, so that the inflation 
  //is based on the final state of the costmap
  costmap_.inflateObstacles();
  // create an empty OccupancyGrid message, ready to fill in
  auto message = nav_msgs::msg::OccupancyGrid();
  // timestamp it with the current time
  message.header.stamp = this->now();
  // tell other nodes which coordinate frame this data is in
  message.header.frame_id = "sim_world";
  // fill in the grid's metadata, pulled from the core's getters
  message.info.resolution = costmap_.getResolution();
  message.info.width = costmap_.getWidth();
  message.info.height = costmap_.getHeight();
  message.info.origin.position.x = costmap_.getOriginX();
message.info.origin.position.y = costmap_.getOriginY();
  // copy in the actual grid data (occupied/free/inflated values)
  message.data = costmap_.getGrid();
  // send the finished message out on the /costmap topic
  costmap_pub_->publish(message);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}