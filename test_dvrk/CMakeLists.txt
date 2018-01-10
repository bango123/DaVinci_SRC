cmake_minimum_required(VERSION 2.8.3)
project(test_dvrk)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  cisst_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
#  stereo_vision
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_dvrk
#  CATKIN_DEPENDS cisst_msgs geometry_msgs roscpp rospy std_msgs stereo_vision
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(setTest setTest.cpp)
add_executable(passThrough passThrough.cpp)


target_link_libraries(setTest ${catkin_LIBRARIES})
target_link_libraries(passThrough ${catkin_LIBRARIES})

