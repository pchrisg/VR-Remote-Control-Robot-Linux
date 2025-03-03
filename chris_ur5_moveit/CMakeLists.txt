cmake_minimum_required(VERSION 3.1.3)
project(chris_ur5_moveit)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  ros_tcp_endpoint
  message_generation
  moveit_msgs
  sensor_msgs
  controller_manager
  roslaunch
  trajectory_msgs
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)

roslaunch_add_file_check(launch)

add_service_files(DIRECTORY srv)

generate_messages(DEPENDENCIES
  geometry_msgs
  std_msgs
  moveit_msgs
  sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    message_runtime
    EIGEN3
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(chris_ur5_moveit src/chris_ur5_moveit.cpp)

target_link_libraries(chris_ur5_moveit ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(chris_ur5_moveit chris_ur5_moveit_gencpp)

install(TARGETS chris_ur5_moveit DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})