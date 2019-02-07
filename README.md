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

#### Subscribed Topics

* **`mesh`** ([pcl_msgs/PolygonMesh](http://docs.ros.org/melodic/api/pcl_msgs/html/msg/PolygonMesh.html))

    The input mesh to convert to grid map.


#### Published Topics

* **`grid_map`** ([grid_map_msg/GridMap(https://github.com/ANYbotics/grid_map/blob/master/grid_map_msgs/msg/GridMap.msg)))

    The converted grid map, obtained from the input mesh.


#### Services

* **`load_mesh_from_file`** ([grid_map_srvs/ProcessFile](https://github.com/ANYbotics/grid_map/blob/master/grid_map_msgs/srv/ProcessFile.srv))

    Trigger the loading of an input mesh from a .ply file.

        rosservice call /mesh_to_grid_map_node/load_mesh_from_file "file_path: '/home/user/bags/mesh_file.ply' topic_name: ''"

* **`save_grid_map_to_file`** ([grid_map_srvs/ProcessFile](https://github.com/ANYbotics/grid_map/blob/master/grid_map_msgs/srv/ProcessFile.srv))

    Trigger the saving of the current grid map to a rosbag file.

        rosservice call /mesh_to_grid_map_node/save_grid_map_to_file "file_path: '/home/user/bags/grid_map.bag' topic_name: 'grid_map'"
