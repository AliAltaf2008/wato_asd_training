#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"


#include "planner_core.hpp"

enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };

struct CellIndex {
  int x;
  int y;
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
  bool operator==(const CellIndex &other) const { return (x == other.x && y == other.y); }
  bool operator!=(const CellIndex &other) const { return (x != other.x || y != other.y); }
};

struct CellIndexHash {
  std::size_t operator()(const CellIndex &idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

struct AStarNode {
  CellIndex index;
  double f_score;
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF {
  bool operator()(const AStarNode &a, const AStarNode &b) {
    return a.f_score > b.f_score;
  }
};

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    bool goalReached();
    nav_msgs::msg::Path planPath();
    
  private:
    robot::PlannerCore planner_;

    //subscribers to planner
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goalSub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
   
    //publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub;

    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid currentMap;
    geometry_msgs::msg::PointStamped currentGoal;

    double robotX = 0.0;
    double robotY=0.0;

    bool goalReceived = false;
    State state_ = State::WAITING_FOR_GOAL;
};

#endif 
