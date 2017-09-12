#ifndef MESH_TO_GRID_MAP_CONVERTER_H
#define MESH_TO_GRID_MAP_CONVERTER_H

#include <string>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <geometry_msgs/PoseStamped.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/MarkerArray.h>

#include <grid_map_core/GridMap.hpp>

namespace mesh_to_grid_map {

constexpr double kDefaultGridMapResolution = 0.05;
static const std::string kDefaultLayerName = "elevation";
constexpr bool kDefaultLatchGridMapPub = true;
constexpr bool kDefaultSaveToRosBagOnPublish = false;
static const std::string kDefaultGridMapRosbagTopicName = "grid_map";
static const std::string kDefaultMeshRosbagTopicName = "mesh";
static const std::string kDefaultGoalPoseRosbagTopicName = "goal_pose";
static const std::string kDefaultRectangleLocationsRosbagTopicName =
    "rectangle_location_";
static const std::string kDefaultRectangleImageRosbagTopicName =
    "rectangle_image_";
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

  void goalPoseCallback(const geometry_msgs::PoseStamped goal_pose_msg);

  void rectangleLocationsCallback(
      const visualization_msgs::MarkerArray rectangle_locations_msg);

  void rectangleImagesCallback(const sensor_msgs::Image rectangle_image_msg);

  // Save callback
  bool saveMapDataCallback(std_srvs::Empty::Request& request,
                           std_srvs::Empty::Response& response);

  // Saves the grid map
  bool saveData();

  // Node Handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Data subscribers.
  ros::Subscriber mesh_sub_;
  ros::Subscriber goal_pose_sub_;
  ros::Subscriber rectangle_locations_sub_;
  std::vector<ros::Subscriber> rectangle_image_subs_;

  // Publishers
  ros::Publisher grid_map_pub_;

  // Services
  ros::ServiceServer save_map_data_srv_;

  // Store all topics
  std::unique_ptr<grid_map::GridMap> grid_map_ptr_;
  std::unique_ptr<pcl_msgs::PolygonMesh> mesh_ptr_;
  std::unique_ptr<geometry_msgs::PoseStamped> goal_pose_ptr_;
  std::vector<geometry_msgs::PoseStamped> rectangle_locations_vector_;
  std::vector<sensor_msgs::Image> rectangle_images_vector_;

  // Grid Map Parameters
  double grid_map_resolution_;
  std::string layer_name_;

  // Control Parameters
  bool latch_grid_map_pub_;
  bool verbose_;

  // Saving parameters
  bool save_to_rosbag_on_publish_;
  std::string rosbag_file_path_;
  std::string rosbag_grid_map_topic_name_;
  std::string rosbag_mesh_topic_name_;
  std::string rosbag_goal_pose_topic_name_;
  std::string rosbag_rectangle_locations_topic_name_;
  std::string rosbag_rectangle_images_topic_name_;
};

}  // namespace mesh_to_grid_map

#endif