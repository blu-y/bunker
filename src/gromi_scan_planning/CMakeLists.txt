cmake_minimum_required(VERSION 2.8.3)
project(gromi_scan_planning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslaunch
  message_generation
  dynamic_reconfigure
  genmsg
  nodelet
  nodelet_topic_tools
  pcl_conversions
  pcl_msgs
  pluginlib
  rosbag
  rosconsole
  roslib
  sensor_msgs
  tf
  tf2_eigen
  message_filters
  tf2
  tf2_ros
  tf2_sensor_msgs
  geometry_msgs
  visualization_msgs
  diagnostic_updater
  velodyne_msgs
  velodyne_driver
  angles
  pcl_ros
  gazebo_ros
  laser_geometry
  laser_filters
  nav_msgs
)

catkin_package(
#    DEPENDS
    LIBRARIES gromi_scan_planning
    CATKIN-DEPENDS roscpp laser_geometry laser_filters tf pcl_conversions pcl_ros roscpp sensor_msgs geometry_msgs nav_msgs
#    INCLUDE_DIRS ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
#    LIBRARIES ${Boost_LIBRARIES} ${catkin_LIBRARIES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} 
)

## Depedent cpp files
set(SOURCES ${SOURCES}
  src/frontier_detector.cpp
)

#include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# add_executable(laserscan_to_pointcloud_node src/laserscan_to_pointcloud_node.cpp)
# target_link_libraries(laserscan_to_pointcloud_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} )
add_executable(frontier_planner_node src/planner.cpp ${SOURCES})
target_link_libraries(frontier_planner_node ${catkin_LIBRARIES} )
# add_dependencies(laserscan_to_pointcloud_node ${catkin_EXPORTED_TARGETS})

