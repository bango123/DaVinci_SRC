cmake_minimum_required(VERSION 2.8.12)
project(stereo_vision)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV 3.4.0 REQUIRED)

## Find all of the dependency libraries
#Catkin components
find_package(catkin REQUIRED COMPONENTS
  cisst_msgs
  camera_info_manager
  cv_bridge
  image_transport
  roscpp
  rosbag
  sensor_msgs
  std_msgs
)

find_package(Boost COMPONENTS filesystem system regex REQUIRED)


#This is a thing does everything!! (basically)
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS camera_info_manager cv_bridge image_transport roscpp sensor_msgs std_msgs rosbag roscpp cisst_msgs
   DEPENDS OpenCV Boost)

#The libraries source files
set (SOURCE_FILES
     DeckLinkAPI/DeckLinkAPIDispatch.cpp
     src/DeckLinkCaptureDelegate.cpp
     src/viewer.cpp
     )

 #The libraries header files
 set (HEADER_FILES
    DeckLinkAPI/DeckLinkAPI.h
    DeckLinkAPI/DeckLinkAPIConfiguration.h
    DeckLinkAPI/DeckLinkAPIDeckControl.h
    DeckLinkAPI/DeckLinkAPIDiscovery.h
    DeckLinkAPI/DeckLinkAPIModes.h
    DeckLinkAPI/DeckLinkAPITypes.h
    DeckLinkAPI/DeckLinkAPIVersion.h
    DeckLinkAPI/LinuxCOM.h
    include/stereo_vision/DeckLinkCaptureDelegate.h
    include/stereo_vision/viewer.h
    )

#All of the headers!!!
include_directories (${stereo_vision_SOURCE_DIR}/include
                     ${catkin_INCLUDE_DIRS}
                     ${OpenCV_INCLUDE_DIRS}
                     ${Boost_INCLUDE_DIRS})

#Make our library
add_library(${PROJECT_NAME}
            ${HEADER_FILES}
            ${SOURCE_FILES} )


#Link the required libraries for the new library we are making!
target_link_libraries(${PROJECT_NAME}
                      ${OpenCV_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES})



#Exectuable that does the stereopublishing
add_executable(stereo_publisher code/StereoPublisher.cpp)
target_link_libraries (stereo_publisher ${PROJECT_NAME} )

#Exectuable that does the imagepublishing for left or right
add_executable(image_publisher code/ImagePublisher.cpp)
target_link_libraries (image_publisher ${PROJECT_NAME} )

#Exectuable for the viewer
add_executable(stereo_viewer code/StereoViewer.cpp)
target_link_libraries (stereo_viewer ${PROJECT_NAME} )

#Exectuable for the sync + delay
add_executable(stereo_sync_delay code/stereo_sync_delay.cpp)
target_link_libraries (stereo_sync_delay ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#Exectuable for the bag2vid
add_executable(stereo_bag2vid code/stereo_bag2vid.cpp)
target_link_libraries (stereo_bag2vid ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#Exectuable for the videorecorder
add_executable(stereo_videorecorder code/stereo_videorecorder.cpp)
target_link_libraries (stereo_videorecorder ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})


#Install the library onto the system
install(TARGETS ${PROJECT_NAME}
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#Install the header files
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )
