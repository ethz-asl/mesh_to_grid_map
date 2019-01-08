# mesh_to_grid_map
## Overview
A node which converts meshes to grid maps and publishes them.

**Keywords:** mesh, grid_map, conversion.

### Building from Source

#### Dependencies

- [Grid Map](https://github.com/ANYbotics/grid_map) (Universal grid map library for mobile robotic mapping),
- [PCL Catkin](https://github.com/ethz-asl/pcl_catkin) (Catkinized version of the latest version of PCL).


#### Building

To build from source, clone the latest version from this repository and its dependecies into your catkin workspace and compile the package using
```
cd ~/catkin_ws/src
git clone git@github.com:ethz-asl/mesh_to_grid_map.git
git clone git@github.com:ANYbotics/grid_map.git
git clone git@github.com:ethz-asl/pcl_catkin.git
cd pcl_catkin
git submodule update --init --recursive
cd  ~/catkin_ws/
catkin build mesh_to_grid_map
```

## Usage
* *[mesh_to_grid_map_test.launch](launch/mesh_to_grid_map_test.launch)* demonstrates a simple example for using the mesh_to_grid_map ROS node. This ROS node subscribes to the topic `mesh` and converts every new message to a grid_map map.

        roslaunch mesh_to_grid_map mesh_to_grid_map_test.launch

* *[mesh_to_grid_map_load_test.launch](launch/mesh_to_grid_map_load_test.launch)* demonstrates a simple example for using the mesh_to_grid_map ROS node. This ROS node loads an existing mesh from file and converts it to a grid_map map.

        roslaunch mesh_to_grid_map mesh_to_grid_map_load_test.launch
