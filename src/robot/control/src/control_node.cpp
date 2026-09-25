#include "control_node.hpp"
#include <optional>
#include <cmath>

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  pathSub = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1)
  );

  odomSub = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1)
  );

  velPub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  currentPath = *msg;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  //retrieve the robot's current x and y position
  robotX = msg->pose.pose.position.x;
  robotY = msg->pose.pose.position.y;
  //get the robot's orientation 
  auto q = msg->pose.pose.orientation;

  //converts q into a single heading angle
  robotYaw = std::atan2(2.0 * (q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z));
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  for(const auto &pose : currentPath.poses) {
    double dx = pose.pose.position.x-robotX;
    double dy = pose.pose.position.y-robotY;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    if(distance>=lookaheadDistance) {
      return pose;
    }
  }
  return std::nullopt;
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  geometry_msgs::msg::Twist cmd;
  //find the distance of the lookahead point
  double dx = target.pose.position.x-robotX;
  double dy=target.pose.position.y-robotY;

  //angle from the robot to the lookahead point and therefore must adjust the robot's yaw to this
  double targetAngle = std::atan2(dy,dx);
  double angleDiff = targetAngle - robotYaw;

  while(angleDiff>M_PI) angleDiff -= 2* M_PI;
  while(angleDiff<-M_PI) angleDiff += 2*M_PI;

  cmd.linear.x = linearSpeed;

  //steer towards correct angle
  cmd.angular.z = angleDiff;

  return cmd;
}

void ControlNode::controlLoop() {

  //just in case there's np hat, publish a default Twist to stop the robot and everything else
  if(currentPath.poses.empty()) {
    velPub->publish(geometry_msgs::msg::Twist());
    return;
  }

  auto finalPose = currentPath.poses.back();
  double dx=finalPose.pose.position.x-robotX;
  double dy = finalPose.pose.position.y-robotY;
  double distanceToGoal = std::sqrt(dx*dx+dy*dy);

  //at a certain distance from the goal, it is sufficient to stop and therefore have reached the goal
  if(distanceToGoal < goalTolerance) {
    velPub-> publish(geometry_msgs::msg::Twist());
    return;
  }

  //find a point on the path to steer towards
  auto lookahead = findLookaheadPoint();

  //stop if a valid lookahead point is not found
  if(!lookahead) {
    velPub-> publish(geometry_msgs::msg::Twist());
    return;
  }
  
  geometry_msgs::msg::Twist cmd = computeVelocity(*lookahead);

  velPub->publish(cmd);
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
