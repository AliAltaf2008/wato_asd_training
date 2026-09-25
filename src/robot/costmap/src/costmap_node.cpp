#include "costmap_node.hpp"
#include <cmath>

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  RCLCPP_INFO(this->get_logger(), "Got a laser scan with %zu ranges", scan->ranges.size());

  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = this->get_clock()->now();
  grid.header.frame_id = "robot/chassis/lidar";  
  grid.info.resolution = 0.1;
  grid.info.width = 100;
  grid.info.height = 100;
  grid.info.origin.position.x = -5.0;
  grid.info.origin.position.y = -5.0;
  grid.data.assign(100 * 100, 0);

  for(size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan -> angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];

    double x = range * std::cos(angle);
    double y = range *std::sin(angle);

    int gridx = static_cast<int>(((x-(-5.0)) / grid.info.resolution)); // make it relative to the origin
    int gridy = static_cast<int>((y - (-5.0)) / grid.info.resolution);

    if (gridx >= 0 && gridx < 100) {
      if(gridy >= 0 && gridy < 100) {
        int flatten = gridy * 100 + gridx; // make it 1D
        grid.data[flatten] = 100; // make it 1D
      }
    }

  }

  double inflationRadius = 1.0;
  int maxCost = 100;
  int inflationCells = static_cast<int> (inflationRadius / grid.info.resolution);
  std::vector<int8_t> inflatedData = grid.data;

  for(int y = 0; y < grid.info.height; ++y) {
    for(int x = 0; x < grid.info.width; ++x) {
      int idx = y * grid.info.width + x; // flatten
      if (grid.data[idx] == 100) {
        for (int infecty = -inflationCells; infecty<=inflationCells; ++infecty) {
          for (int infectx = -inflationCells; infectx < inflationCells; ++infectx)
          {
            int nx = x+ infectx;
            int ny = y + infecty;

            //bounds check
            if (nx < 0 || nx >= grid.info.width || ny < 0 || ny >= grid.info.height) {
                continue;
            }

            double distance = std::sqrt(infectx * infectx + infecty * infecty) * grid.info.resolution; //convert it to meters
            if (distance > inflationRadius) {
              continue;
            }

            int cost = static_cast<int>(maxCost * (1.0 - (distance / inflationRadius)));
            int nidx = ny * grid.info.width + nx;

            if (cost > inflatedData[nidx]) {
              inflatedData[nidx] = cost; //stronger obstacles will be noticed more than weaker one
          }
        }
        }
      }
    }
  }

  grid.data = inflatedData;


  costmap_pub_->publish(grid);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}