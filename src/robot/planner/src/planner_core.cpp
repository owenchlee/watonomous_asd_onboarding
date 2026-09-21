#include "planner_core.hpp"
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

namespace robot
{
// A* pathfinding algorithm.
// Explores the grid outward from start, always expanding the most promising
// cell next (lowest f_score = cost so far + estimated distance to goal).
//      f = g + h (where g = cost from start to current cell, h = cost from goal to current cell)
//          - however, h-score is a estimate as the cost assumes theres no walls
//
// open_set: cells still waiting to be explored, sorted by f_score
// g_score: cheapest known cost to reach each cell from start
// came_from: tracks the path backward, used to rebuild it once the goal is found
//
// For each cell popped from open_set: if it's the goal, walk backward through
// came_from to build the path, then reverse it into start->goal order.
// Otherwise, check its 8 neighbors, skipping ones out of bounds, unknown (-1),
// or too costly (>=50). If a neighbor's new cost is better than any previously
// found, update it and add it to open_set.
//
// Returns an empty path if the open_set runs out before reaching the goal.

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger) {}

std::vector<CellIndex> PlannerCore::planPath(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& start, const CellIndex& goal) {
  int width = map.info.width;
  int height = map.info.height;

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

  auto heuristic = [&](const CellIndex& a, const CellIndex& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
  };

  g_score[start] = 0.0;
  open_set.push(AStarNode(start, heuristic(start, goal)));

  std::vector<CellIndex> directions = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  };

  while (!open_set.empty()) {
    CellIndex current = open_set.top().index;
    open_set.pop();

    if (current == goal) {
      std::vector<CellIndex> path;
      while (current != start) {
        path.push_back(current);
        current = came_from[current];
      }
      path.push_back(start);
      std::reverse(path.begin(), path.end());
      return path;
    }

    for (const auto& dir : directions) {
      CellIndex neighbor(current.x + dir.x, current.y + dir.y);

      if (neighbor.x < 0 || neighbor.x >= width || neighbor.y < 0 || neighbor.y >= height) continue;

      int index = neighbor.y * width + neighbor.x;
      int8_t cell_value = map.data[index];
      //dont want to enter a potentially dangerous cell, so we skip unknown and occupied cells
      if (neighbor != start && (cell_value < 0 || cell_value >= 50)) continue;

      double move_cost = (dir.x != 0 && dir.y != 0) ? 1.4142 : 1.0;
      double tentative_g = g_score[current] + move_cost;

      if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
        g_score[neighbor] = tentative_g;
        double f = tentative_g + heuristic(neighbor, goal);
        came_from[neighbor] = current;
        open_set.push(AStarNode(neighbor, f));
      }
    }
  }

  return std::vector<CellIndex>();
}

}