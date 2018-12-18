cmake_minimum_required(VERSION 2.8.0)
project(mesh_to_grid_map)

add_definitions(-std=c++11 -Wall -Werror)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})


cs_add_library(mesh_to_grid_map_converter
  src/library/mesh_to_grid_map_converter.cpp
)

cs_add_executable(mesh_to_grid_map_node
  src/mesh_to_grid_map_node.cpp
)

target_link_libraries(mesh_to_grid_map_node mesh_to_grid_map_converter ${catkin_LIBRARIES})

cs_install()
cs_export()
