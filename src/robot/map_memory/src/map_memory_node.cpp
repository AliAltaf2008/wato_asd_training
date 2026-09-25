#include "map_memory_node.hpp"
#include <cmath>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmapSub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders:: _1));

  odomSub = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1)
  );

  mapPub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::timerCallback, this));

  //important vars and multiplying everything by 3 to ensure all the data is within the bounds
  globalMap.info.resolution = 0.1;
  globalMap.info.width = 300;
  globalMap.info.height = 300;
  globalMap.info.origin.position.x = -15.0;
  globalMap.info.origin.position.y = -15.0;
  globalMap.data.assign(300*300, 0);
  mapPub->publish(globalMap);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  costmapUpdated = true;
  latestCostmap = *msg;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  currentX = msg->pose.pose.position.x;
  currentY = msg->pose.pose.position.y;
  haveOdom = true;

  double newX= currentX - lastUpdateX;
  double newY = currentY-lastUpdateY;
  double distance = std::sqrt(newX*newX+newY*newY);

  if(distance >= distanceThreshold) {
    shouldUpdateMap = true;
  }
}

void MapMemoryNode::timerCallback() {
  //check if it is necessary to merge costmpa and globalmap
  if(!haveOdom || !costmapUpdated || !shouldUpdateMap) {
    return;
  }
  integrateCostmap();
  lastUpdateX = currentX;
  lastUpdateY=currentY;
  shouldUpdateMap=false;
  mapPub->publish(globalMap);

}

void MapMemoryNode::integrateCostmap() {
  double resolution = latestCostmap.info.resolution;
  int widthCM = latestCostmap.info.width;
  int heightCM =latestCostmap.info.height;
  double originX_CM = latestCostmap.info.origin.position.x;
  double originY_CM = latestCostmap.info.origin.position.y;
  for (int costmapY = 0; costmapY < heightCM; ++costmapY) 
  {
    for (int costmapX = 0; costmapX  < widthCM; ++costmapX) 
    {
      int costmapIndex = costmapY * widthCM + costmapX;
      int8_t value = latestCostmap.data[costmapIndex];

      double localX = originX_CM + costmapX * resolution;
      double localY = originY_CM + costmapY * resolution;

      double worldX = currentX + localX;
      double worldY = currentY + localY;

      int globalX = static_cast<int>((worldX - globalMap.info.origin.position.x) / globalMap.info.resolution);
      int globalY = static_cast<int>((worldY-globalMap.info.origin.position.y) / globalMap.info.resolution);


      // bounds check just in case 
      if (globalX < 0 || globalX >= globalMap.info.width || globalY < 0 || globalY >= globalMap.info.height)
      {
        continue;
      }

      int globalIndex = globalY * globalMap.info.width + globalX;
      if(value > globalMap.data[globalIndex]) {
        globalMap.data[globalIndex] = value;
      }
      
    }
    
  }
}




int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
