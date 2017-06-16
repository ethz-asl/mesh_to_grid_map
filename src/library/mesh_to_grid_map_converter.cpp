#include "mesh_to_grid_map/mesh_to_grid_map_converter.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/GridMap.hpp>
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
      verbose_(kDefaultVerbose) {
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
}

void MeshToGridMapConverter::getParametersFromRos() {
  nh_private_.param("grid_map_resolution", grid_map_resolution_,
                    grid_map_resolution_);
  nh_private_.param("layer_name", layer_name_, layer_name_);
  nh_private_.param("latch_grid_map_pub", latch_grid_map_pub_,
                    latch_grid_map_pub_);
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
}

}  // namespace mesh_to_grid_map