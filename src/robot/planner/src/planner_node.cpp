#include "planner_node.hpp"
#include <cmath>
#include <queue>
#include <unordered_set>
#include <unordered_map>


PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  mapSub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goalSub = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1)
  );

  odomSub = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1)
  );

  pathPub = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  currentMap = *msg;

  if(state_==State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    nav_msgs::msg::Path path = planPath();
    pathPub->publish(path);
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  currentGoal = *msg;
  //other funcs can now check goalReceived and safely execute while calling goalCallback
  goalReceived = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  //notifies planner of the current position of the robot and whether it has reached the goal or not
  robotX = msg->pose.pose.position.x;
  robotY = msg->pose.pose.position.y;
}

bool PlannerNode::goalReached() {
  double changeX = robotX-currentGoal.point.x;
  double changeY = robotY-currentGoal.point.y;
  double distance = std::sqrt(changeX*changeX+changeY*changeY);
  //check if the distance is within the threshold
  return distance < 0.5;
}

void PlannerNode::timerCallback() {
  if(!goalReceived || state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }

  if(goalReached()) {
    state_ = State::WAITING_FOR_GOAL;
  } else {
    nav_msgs::msg::Path path = planPath();
    //take the return of planPath() and publish it to the topic /path
    pathPub->publish(path);
  }
}

nav_msgs::msg::Path PlannerNode::planPath() {
  nav_msgs::msg::Path path;
  //time stamp so other nodes know when the path was created
  path.header.stamp=this->get_clock()->now();
  path.header.frame_id = "sim_world";
  double res = currentMap.info.resolution;
  double originX = currentMap.info.origin.position.x;
  double originY= currentMap.info.origin.position.y;
  int width = currentMap.info.width;
  int height = currentMap.info.height;

  CellIndex start(
    //converts the robot's real-world pos into a grid cell
    static_cast<int>((robotX-originX) / res),
    static_cast<int>((robotY-originY) / res)
  );

  CellIndex goal (
    static_cast<int>((currentGoal.point.x - originX) / res),
    static_cast<int>((currentGoal.point.y - originY) / res)
    //conversion applied to the goal point as well
  );

  //stores the unexplored cells and gives back the lowest f_score first
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> openList;
  //stores the explored cells
  std::unordered_set<CellIndex, CellIndexHash> closedList;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> cameFrom;
  std::unordered_map<CellIndex, double, CellIndexHash> gScore;
  //set the robot's starting cell to 0.0 
  gScore[start] = 0.0;
  //calc the remaining distance to the goal 
  double h0 = std::abs(goal.x - start.x) + std::abs(goal.y-start.y);
  openList.push(AStarNode(start, h0));

  //four-way directional movement with x moving up or down and y moving up or down
  int dx4[] = {1, -1, 0, 0};
  int dy4[] = {0, 0, 1, -1};

  while(!openList.empty()) {
    AStarNode current = openList.top();
    openList.pop();
    if(closedList.find(current.index) != closedList.end()) {
      continue;
    }
    if(current.index == goal) {
      std::vector<CellIndex> reversePath;
      CellIndex cell = current.index;
      while(!(cell==start)) {
        reversePath.push_back(cell);
        cell = cameFrom[cell];
      }
      // start tracing the path contructed backwards
      reversePath.push_back(start);

      for(int i =reversePath.size() - 1; i>=0; --i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = originX + reversePath[i].x * res;
        pose.pose.position.y = originY + reversePath[i].y * res;
        path.poses.push_back(pose);
        //walks the reversed list backwards from goal to start and converts each cell back to meters
      }

      return path;
    }

    closedList.insert(current.index);

    //check all four neighboring cells
    for(int i =0; i<4; ++i) {
      int nx = current.index.x + dx4[i];
      int ny = current.index.y + dy4[i];

      //bounds check
      if(nx < 0 || nx >=width || ny < 0 || ny >=height) {
        continue;
      }

      int neighborIndex = ny*width + nx;
      //check the cost value of the neighboring cell
      int8_t cellValue = currentMap.data[neighborIndex];

      //set a threshold for when the robot cannot go through a cell (inflation reaches 100 right at an obstacle so 65 is a reasonable threshold)
      if(cellValue >=65) {
        continue;
      }

      CellIndex neighbor(nx, ny);
      if(closedList.find(neighbor) != closedList.end()) {
        continue;
      }

      double updatedG = gScore[current.index] + 1.0;

      if(gScore.find(neighbor) ==gScore.end() || updatedG < gScore[neighbor]) {
        gScore[neighbor] = updatedG;
        cameFrom[neighbor] = current.index;
        double h = std::abs(goal.x-nx) + std::abs(goal.y-ny);
        openList.push(AStarNode(neighbor, updatedG + h));
      }
    }
  }

  return path;
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}