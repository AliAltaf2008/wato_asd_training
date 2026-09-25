#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    void integrateCostmap();

  private:
    robot::MapMemoryCore map_memory_;
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmapSub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPub;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid latestCostmap;
    nav_msgs::msg::OccupancyGrid globalMap;

    double currentX = 0.0;
    double currentY = 0.0;
    double lastUpdateX = 0.0;
    double lastUpdateY = 0.0;

    //Robot checks/updates
    bool haveOdom = false;
    bool costmapUpdated = false;
    bool shouldUpdateMap = false;

    const double distanceThreshold = 1.5;
};

#endif 
