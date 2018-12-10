#include "mesh_to_grid_map/mesh_to_grid_map_converter.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

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
      verbose_(kDefaultVerbose),
      save_to_rosbag_on_publish_(kDefaultSaveToRosBagOnPublish),
      rosbag_topic_name_(kDefaultRosbagTopicName) {
  // Initial interaction with ROS
  subscribeToTopics();
  advertiseTopics();
  getParametersFromRos();
}

void MeshToGridMapConverter::subscribeToTopics() {
  mesh_sub_ =
      nh_.subscribe("mesh", 10, &MeshToGridMapConverter::meshCallback, this);
}

void MeshToGridMapConverter::advertiseTopics() {
  grid_map_pub_ = nh_private_.advertise<grid_map_msgs::GridMap>(
      "grid_map", 1, latch_grid_map_pub_);
  save_grid_map_srv_ = nh_private_.advertiseService(
      "save_grid_map", &MeshToGridMapConverter::saveGridMapCallback, this);
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
  nh_private_.param("rosbag_topic_name", rosbag_topic_name_,
                    rosbag_topic_name_);
  nh_private_.param("verbose", verbose_, verbose_);
}

void MeshToGridMapConverter::meshCallback(
    const pcl_msgs::PolygonMesh& mesh_msg) {
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

  // Saving the gridmap to a rosbag if requested
  if (save_to_rosbag_on_publish_) {
    saveGridmap(map);
  }

  // Saving the gridmap to the object
  last_grid_map_ptr_.reset(new grid_map::GridMap(map));
}

bool MeshToGridMapConverter::saveGridMapCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  // Check there's actually a grid map saved
  if (!last_grid_map_ptr_) {
    ROS_ERROR("No grid map produced yet to save.");
    return false;
  }
  return saveGridmap(*last_grid_map_ptr_);
}

bool MeshToGridMapConverter::saveGridmap(const grid_map::GridMap& map) {
  // Saving the map
  if (!rosbag_file_path_.empty()) {
    grid_map::GridMapRosConverter grid_map_ros_converter;
    if (verbose_) {
      ROS_INFO_STREAM(
          "Saved the grid map message to file: " << rosbag_file_path_);
    }
    grid_map_ros_converter.saveToBag(map, rosbag_file_path_,
                                     rosbag_topic_name_);
  } else {
    ROS_ERROR(
        "No rosbag filepath specified (as ros param \"rosbag_file_path\"");
    return false;
  }
  return true;
}


}  // namespace mesh_to_grid_map
