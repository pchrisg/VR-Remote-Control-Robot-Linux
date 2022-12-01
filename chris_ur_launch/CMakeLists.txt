cmake_minimum_required(VERSION 3.0.2)
project(chris_ur_launch)

find_package(catkin REQUIRED COMPONENTS
  ur_client_library
)

catkin_package()

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)