#include "planner_node.hpp"
#include <cmath>
#include "geometry_msgs/msg/pose_stamped.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())), state_(State::WAITING_FOR_GOAL) {
  // Initialize the constructs and their parameters
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&PlannerNode::odomCallBack, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PlannerNode::timerCallback, this));
}

//this function converts inputs to the right format
//then calls the A* algorithm to plan a path
//the converts results back and publishes it
void PlannerNode::planPath() {
  //check if theres a goal or a map, if not, we cannot plan a path
  if (!goal_received_ || current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: missing map or goal");
    return;
  }

  //take in current data
  double resolution = current_map_.info.resolution;
  double origin_x = current_map_.info.origin.position.x;
  double origin_y = current_map_.info.origin.position.y;

  //convert to grid cell (current position (start) and goal)
  robot::CellIndex start(
    static_cast<int>((robot_pose_.position.x - origin_x) / resolution),
    static_cast<int>((robot_pose_.position.y - origin_y) / resolution)
  );

  robot::CellIndex goal(
    static_cast<int>((goal_.point.x - origin_x) / resolution),
    static_cast<int>((goal_.point.y - origin_y) / resolution)
  );

  //run the A* algorithm to get a path from start to goal
  std::vector<robot::CellIndex> cell_path = planner_.planPath(current_map_, start, goal);

  //build publish message
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->now();
  path_msg.header.frame_id = "sim_world";

  //loops through cells to turn it back into world coordinates
  for (const auto& cell : cell_path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = cell.x * resolution + origin_x;
    pose.pose.position.y = cell.y * resolution + origin_y;
    path_msg.poses.push_back(pose);
  }

  //publush the path to the /path topic
  path_pub_->publish(path_msg);
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  current_map_ = *map;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::odomCallBack(const nav_msgs::msg::Odometry::SharedPtr odom) {
  robot_pose_ = odom->pose.pose;
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goal) {
  goal_ = *goal;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    double dx = goal_.point.x - robot_pose_.position.x;
    double dy = goal_.point.y - robot_pose_.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < 0.5) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
