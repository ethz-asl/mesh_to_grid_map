#include "mesh_to_grid_map/mesh_to_grid_map_converter.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <rosbag/bag.h>

#include <grid_map_msgs/GridMap.h>
#include <grid_map_pcl/GridMapPclConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

namespace mesh_to_grid_map {

MeshToGridMapConverter::MeshToGridMapConverter(ros::NodeHandle nh,
                                               ros::NodeHandle nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      grid_map_resolution_(kDefaultGridMapResolution),
      layer_name_(kDefaultLayerName),
      latch_grid_map_pub_(kDefaultLatchGridMapPub),
      save_to_rosbag_on_publish_(kDefaultSaveToRosBagOnPublish),
      rosbag_grid_map_topic_name_(kDefaultGridMapRosbagTopicName),
      rosbag_mesh_topic_name_(kDefaultMeshRosbagTopicName),
      rosbag_goal_pose_topic_name_(kDefaultGoalPoseRosbagTopicName),
      rosbag_rectangle_locations_topic_name_(
          kDefaultRectangleLocationsRosbagTopicName),
      rosbag_rectangle_images_topic_name_(
          kDefaultRectangleImageRosbagTopicName),
      verbose_(kDefaultVerbose) {
  // Initial interaction with ROS
  subscribeToTopics();
  advertiseTopics();
  getParametersFromRos();
}

void MeshToGridMapConverter::subscribeToTopics() {
  mesh_sub_ =
      nh_.subscribe("mesh", 10, &MeshToGridMapConverter::meshCallback, this);
  goal_pose_sub_ = nh_.subscribe(
      "goal_pose", 10, &MeshToGridMapConverter::goalPoseCallback, this);
  rectangle_locations_sub_ =
      nh_.subscribe("rectangle_locations", 10,
                    &MeshToGridMapConverter::rectangleLocationsCallback, this);
}

void MeshToGridMapConverter::advertiseTopics() {
  grid_map_pub_ = nh_private_.advertise<grid_map_msgs::GridMap>(
      "grid_map", 1, latch_grid_map_pub_);
  save_map_data_srv_ = nh_private_.advertiseService(
      "save_map_data", &MeshToGridMapConverter::saveMapDataCallback, this);
}

void MeshToGridMapConverter::getParametersFromRos() {
  nh_private_.param("grid_map_resolution", grid_map_resolution_,
                    grid_map_resolution_);
  nh_private_.param("layer_name", layer_name_, layer_name_);
  nh_private_.param("latch_grid_map_pub", latch_grid_map_pub_,
                    latch_grid_map_pub_);
  nh_private_.param("save_to_rosbag_on_publish", save_to_rosbag_on_publish_,
                    save_to_rosbag_on_publish_);
  nh_private_.param("rosbag_filepath", rosbag_file_path_, rosbag_file_path_);
  nh_private_.param("rosbag_grid_map_topic_name", rosbag_grid_map_topic_name_,
                    rosbag_grid_map_topic_name_);
  nh_private_.param("rosbag_mesh_topic_name", rosbag_mesh_topic_name_,
                    rosbag_mesh_topic_name_);
  nh_private_.param("rosbag_goal_pose_topic_name", rosbag_goal_pose_topic_name_,
                    rosbag_goal_pose_topic_name_);
  nh_private_.param("rosbag_rectangle_locations_topic_name",
                    rosbag_rectangle_locations_topic_name_,
                    rosbag_rectangle_locations_topic_name_);
  nh_private_.param("rosbag_rectangle_images_topic_name",
                    rosbag_rectangle_images_topic_name_,
                    rosbag_rectangle_images_topic_name_);
  nh_private_.param("verbose", verbose_, verbose_);
}

void MeshToGridMapConverter::meshCallback(
    const pcl_msgs::PolygonMesh& mesh_msg) {
  mesh_ptr_.reset(new pcl_msgs::PolygonMesh(mesh_msg));

  if (verbose_) {
    ROS_INFO("Mesh received, starting conversion.");
  }

  // Converting from message to an object
  pcl::PolygonMesh polygon_mesh;
  pcl_conversions::toPCL(mesh_msg, polygon_mesh);

  // Creating the grid map
  grid_map::GridMap map;
  map.setFrameId(mesh_msg.header.frame_id);

  // Creating the converter
  grid_map::GridMapPclConverter grid_map_pcl_converter;
  grid_map_pcl_converter.initializeFromPolygonMesh(polygon_mesh,
                                                   grid_map_resolution_, map);
  const std::string layer_name(layer_name_);
  grid_map_pcl_converter.addLayerFromPolygonMesh(polygon_mesh, layer_name, map);

  // Printing some debug info about the mesh and the map
  if (verbose_) {
    ROS_INFO_STREAM("Number of polygons: " << polygon_mesh.polygons.size());
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
             map.getLength().x(), map.getLength().y(), map.getSize()(0),
             map.getSize()(1));
  }

  // Publish grid map.
  map.setTimestamp(mesh_msg.header.stamp.toNSec());
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map, message);

  // Publishing the grid map message.
  grid_map_pub_.publish(message);
  if (verbose_) {
    ROS_INFO("Published a grid map message.");
  }

  // Saving the gridmap to the object
  grid_map_ptr_.reset(new grid_map::GridMap(map));
}

void MeshToGridMapConverter::goalPoseCallback(
    const geometry_msgs::PoseStamped goal_pose_msg) {
  goal_pose_ptr_.reset(new geometry_msgs::PoseStamped(goal_pose_msg));
}

void MeshToGridMapConverter::rectangleLocationsCallback(
    const visualization_msgs::MarkerArray rectangle_locations_msg) {
  rectangle_locations_vector_.clear();
  rectangle_images_vector_.clear();
  rectangle_image_subs_.clear();

  for (const visualization_msgs::Marker rectangle_marker :
       rectangle_locations_msg.markers) {
    if (rectangle_marker.type == visualization_msgs::Marker::SPHERE) {
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.pose = rectangle_marker.pose;
      rectangle_image_subs_.emplace_back(nh_.subscribe(
          std::string("rectangle_") +
              std::to_string(rectangle_locations_vector_.size()) +
              std::string("_image"),
          10, &MeshToGridMapConverter::rectangleImagesCallback, this));
      rectangle_locations_vector_.push_back(pose_msg);
    }
  }
}

void MeshToGridMapConverter::rectangleImagesCallback(
    const sensor_msgs::Image rectangle_image_msg) {
  for (sensor_msgs::Image stored_image_msg : rectangle_images_vector_) {
    if (rectangle_image_msg.header.seq == stored_image_msg.header.seq) {
      stored_image_msg = rectangle_image_msg;
      return;
    }
  }
  rectangle_images_vector_.push_back(rectangle_image_msg);
}

bool MeshToGridMapConverter::saveMapDataCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  // Check all the information exists
  if (!grid_map_ptr_ || !mesh_ptr_ ||
      rectangle_images_vector_.size() != rectangle_locations_vector_.size()) {
    ROS_ERROR("Still waiting on map data to save.");
    return false;
  }
  if (!goal_pose_ptr_ || rectangle_locations_vector_.size() == 0) {
    ROS_WARN(
        "No rectangle information, did it fail or did you screw up the topic "
        "remap?");
  }
  return saveData();
}

bool MeshToGridMapConverter::saveData() {
  // Saving the map
  if (!rosbag_file_path_.empty()) {
    if (verbose_) {
      ROS_INFO_STREAM("Saved the map data to file: " << rosbag_file_path_);
    }

    grid_map_msgs::GridMap grid_map_msg;
    grid_map::GridMapRosConverter::toMessage(*grid_map_ptr_, grid_map_msg);
    ros::Time time = ros::Time::now();

    rosbag::Bag bag;
    bag.open(rosbag_file_path_, rosbag::bagmode::Write);
    bag.write(rosbag_mesh_topic_name_, time, *mesh_ptr_);
    for (size_t i = 0; i < rectangle_locations_vector_.size(); ++i) {
      bag.write(rosbag_rectangle_locations_topic_name_ + std::to_string(i),
                time, rectangle_locations_vector_[i]);
      bag.write(rosbag_rectangle_images_topic_name_ + std::to_string(i), time,
                rectangle_images_vector_[i]);
    }
    bag.write(rosbag_grid_map_topic_name_, time, grid_map_msg);
    bag.close();

  } else {
    ROS_ERROR(
        "No rosbag filepath specified (as ros param \"rosbag_file_path\"");
    return false;
  }
  return true;
}

}  // namespace mesh_to_grid_map