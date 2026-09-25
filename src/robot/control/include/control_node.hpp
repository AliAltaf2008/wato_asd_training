#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "control_core.hpp"
#include <optional>
#include "geometry_msgs/msg/pose_stamped.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlLoop();
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint();
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target);

  private:
    robot::ControlCore control_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pathSub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velPub;
    rclcpp::TimerBase::SharedPtr timer_;

    //stores the most recent path to follow
    nav_msgs::msg::Path currentPath;

    double robotX = 0.0;
    double robotY=0.0;
    double robotYaw = 0.0;
    double lookaheadDistance = 1.0;
    double linearSpeed = 0.5;
    double goalTolerance = 0.1;
};

#endif
