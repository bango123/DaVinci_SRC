cmake_minimum_required(VERSION 2.8.3)
project(StereoVision)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)

#Catkin components
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

#This uses OpenCV
find_package(OpenCV REQUIRED)

#We need qt5
find_package(Qt5Core REQUIRED)


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS camera_info_manager cv_bridge image_transport roscpp sensor_msgs std_msgs)


set (SOURCE_FILES
     DeckLinkAPI/DeckLinkAPIDispatch.cpp
     src/DeckLinkCaptureDelegate.cpp
     src/PublishImageWorker.cpp
     )

 # all header files
 set (HEADER_FILES
         DeckLinkAPI/DeckLinkAPI.h
         DeckLinkAPI/DeckLinkAPIConfiguration.h
         DeckLinkAPI/DeckLinkAPIDeckControl.h
         DeckLinkAPI/DeckLinkAPIDiscovery.h
         DeckLinkAPI/DeckLinkAPIModes.h
         DeckLinkAPI/DeckLinkAPITypes.h
         DeckLinkAPI/DeckLinkAPIVersion.h
         DeckLinkAPI/LinuxCOM.h
      include/StereoVision/DeckLinkCaptureDelegate.h
      include/StereoVision/PublishImageWorker.h
      )

include_directories (
      ${StereoVision_SOURCE_DIR}/include)

set (CMAKE_AUTOMOC ON)
add_library(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES} )
qt5_use_modules(${PROJECT_NAME} Core)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(testCameraCaptureDelegate examples/testingcameracapturedelegate.cpp)
target_link_libraries (testCameraCaptureDelegate ${PROJECT_NAME} ${catkin_LIBRARIES})
