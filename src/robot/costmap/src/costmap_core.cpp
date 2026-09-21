#include "costmap_core.hpp"
//to use cos and sin
#include <cmath>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : 
    //set starting values
    //bumped inflation radius up so the robot's chassis actually clears obstacles
    logger_(logger), resolution_(0.1), width_(200), height_(200), origin_x_(-10.0), origin_y_(-10.0), inflation_radius_(1.0), max_cost_(100)
{
    initializeCostmap();
}

//defining costmap function
void CostmapCore::initializeCostmap() {
    // fill every slot with a value, slots are determined by height and width
    //value is 0 for free space
    grid_.assign(width_ * height_, 0);
}

void CostmapCore::resetGrid() {
    initializeCostmap();
}

void CostmapCore::convertToGrid(double range, double angle, int& x_grid, int& y_grid) {
    //determine actual length
    double x = range * cos(angle);
    double y = range * sin(angle);
    //adjust based on origin to get grid coordinates
    double grid_x = x - origin_x_;
    double grid_y = y - origin_y_;
    //convert meters to cell count and round down
    x_grid = static_cast<int>(grid_x / resolution_);
    y_grid = static_cast<int>(grid_y / resolution_);
}

void CostmapCore::markObstacle(int x_grid, int y_grid) {
  if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_) {
    grid_[y_grid * width_ + x_grid] = 100;
  }
}

void CostmapCore::inflateObstacles() {
    //calculate how many cells in a 1m radius
    int inflation_cells = static_cast<int>(inflation_radius_ / resolution_);

    //scan every cell in the grid
    for (int y = 0; y < height_; y++) {
    for (int x = 0; x < width_; x++) {
        //process cells occupied by obstacles (and then build out from there, like inflating a balloon)
        if (grid_[y * width_ + x] == max_cost_) {
        for (int dy = -inflation_cells; dy <= inflation_cells; dy++) {
            for (int dx = -inflation_cells; dx <= inflation_cells; dx++) {
            int nx = x + dx;
            int ny = y + dy;
            //bounds check to make sure we don't go outside the grid
            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
                //if the distance is less than or equal to the inflation radius, give it a warning value
                if (distance <= inflation_radius_) {
                int cost = static_cast<int>(max_cost_ * (1.0 - distance / inflation_radius_));
                int index = ny * width_ + nx;
                //makes sure the cost is the highest
                //for example if both a and b overlap, and a is closer to the obstacle than b
                //then a will have a higher cost than b, so we want to make sure that b doesn't overwrite a's cost
                if (cost > grid_[index]) {
                    grid_[index] = cost;
                }
                }
            }
            }
        }
        }
    }
    }
}

// getters
std::vector<int8_t> CostmapCore::getGrid() const {
  return grid_;
}

int CostmapCore::getWidth() const {
  return width_;
}

int CostmapCore::getHeight() const {
  return height_;
}

double CostmapCore::getResolution() const {
  return resolution_;
}

double CostmapCore::getOriginX() const {
  return origin_x_;
}

double CostmapCore::getOriginY() const {
  return origin_y_;
}

}  