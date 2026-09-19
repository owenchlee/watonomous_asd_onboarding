#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_
//add the vector libary
#include <vector>
#include "rclcpp/rclcpp.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);
    //make new function that outputs both x_grid and y_grid
    void convertToGrid(double range, double angle, int& x_grid, int& y_grid);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();
    //getters to allow origin values to be read outside of costmap core
    std::vector<int8_t> getGrid() const;
    int getWidth() const;
    int getHeight() const;
    double getResolution() const;
    double getOriginX() const;
    double getOriginY() const;

  private:
    rclcpp::Logger logger_;
    //create a vector that holds 8 bit inetgers that fill up the grid
    std::vector<int8_t> grid_;
    //resolution represents how many meters each cell in the grid represents
    //if its 0.1 meters per cell, each cell in the grid represents 10x10 centimeters in the real world
    double resolution_;
    //represents how many cells wide and tall the grid is
    int width_;
    int height_;
    //initialize starting point of cells
    double origin_x_;
    double origin_y_;
    //inflation radius represents the radius around each obstacle to be marked as occupied
    double inflation_radius_;
    //cost = how likely it is that a cell is occupied, 0 = free, 100 = occupied
    int max_cost_;
    //declare a function that initializes the costmap,called in the constructor
    void initializeCostmap();

};

}  

#endif  