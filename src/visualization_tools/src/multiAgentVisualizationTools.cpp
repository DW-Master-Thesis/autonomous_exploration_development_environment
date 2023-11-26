#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <std_msgs/msg/float32.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

double exploredVoxelSize = 0.5;

pcl::PointCloud<pcl::PointXYZI>::Ptr exploredCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredCloud2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::VoxelGrid<pcl::PointXYZI> exploredDwzFilter;

float exploredVolume = 0;
vector<string> robotNames;

shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pubExploredAreasPtr;
shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> pubExploredVolumePtr;
vector<shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>>> subExploredAreasPtrs;

void exploredAreaHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*msg, *cloud);

  *exploredCloud += *cloud;
  exploredCloud2->clear();
  exploredDwzFilter.setInputCloud(exploredCloud);
  exploredDwzFilter.filter(*exploredCloud2);

  pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud = exploredCloud;
  exploredCloud = exploredCloud2;
  exploredCloud2 = tempCloud;

  sensor_msgs::msg::PointCloud2 exploredAreasMsg;
  pcl::toROSMsg(*exploredCloud, exploredAreasMsg);
  exploredAreasMsg.header.stamp = msg->header.stamp;
  exploredAreasMsg.header.frame_id = "map";
  pubExploredAreasPtr->publish(exploredAreasMsg);

  exploredVolume = exploredCloud->points.size() * exploredVoxelSize * exploredVoxelSize * exploredVoxelSize;
  std_msgs::msg::Float32 exploredVolumeMsg;
  exploredVolumeMsg.data = exploredVolume;
  pubExploredVolumePtr->publish(exploredVolumeMsg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("multiAgentVisualizationTools");

  nh->declare_parameter<double>("exploredVoxelSize", exploredVoxelSize);
  nh->declare_parameter<vector<string>>("robotNames", vector<string>());

  nh->get_parameter("exploredVoxelSize", exploredVoxelSize);
  nh->get_parameter("robotNames", robotNames);

  for (int i = 0; i < robotNames.size(); i++)
  {
    auto robotName = robotNames[i];
    // Log robot names
    RCLCPP_INFO(nh->get_logger(), "Robot %d: %s", i, robotName.c_str());
    auto subExploredAreasPtr = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
        robotName + "/explored_areas", 5, exploredAreaHandler);
    subExploredAreasPtrs.push_back(subExploredAreasPtr);
  }

  pubExploredAreasPtr = nh->create_publisher<sensor_msgs::msg::PointCloud2>("explored_areas", 5);
  pubExploredVolumePtr = nh->create_publisher<std_msgs::msg::Float32>("explored_volume", 5);

  exploredDwzFilter.setLeafSize(exploredVoxelSize, exploredVoxelSize, exploredVoxelSize);

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);
    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
