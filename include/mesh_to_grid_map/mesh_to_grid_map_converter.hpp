#ifndef MESH_TO_GRID_MAP_CONVERTER_H
#define MESH_TO_GRID_MAP_CONVERTER_H

#include <string>

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_msgs/PolygonMesh.h>

#include <grid_map_core/GridMap.hpp>

namespace mesh_to_grid_map {

constexpr double kDefaultGridMapResolution = 0.05;
static const std::string kDefaultLayerName = "elevation";
constexpr bool kDefaultLatchGridMapPub = true;
constexpr bool kDefaultSaveToRosBag = false;
static const std::string kDefaultRosbagTopicName = "grid_map";
constexpr bool kDefaultVerbose = false;

class MeshToGridMapConverter {

public:
  MeshToGridMapConverter(ros::NodeHandle nh, ros::NodeHandle nh_private);

private:
  // Initial interactions with ROS
  void subscribeToTopics();
  void advertiseTopics();
  void getParametersFromRos();

  // Datacallback
  void meshCallback(const pcl_msgs::PolygonMesh& mesh);

  // Node Handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Data subscribers.
  ros::Subscriber mesh_sub_;

  // Publishers
  ros::Publisher grid_map_pub_;

  // Last grid map
  //std::unique_ptr<grid_map::GridMap> last_map_ptr_;

  // Grid Map Parameters
  double grid_map_resolution_;
  std::string layer_name_;

  // Control Parameters
  bool latch_grid_map_pub_;
  bool verbose_;

  // Saving parameters
  bool save_to_rosbag_;
  std::string rosbag_file_path_;
  std::string rosbag_topic_name_;

};

} // namespace mesh_to_grid_map

#endif