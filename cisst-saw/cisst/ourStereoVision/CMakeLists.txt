cmake_minimum_required(VERSION 2.8.3)
project (ourStereoVision)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    roscpp
    sensor_msgs
    std_msgs
    image_transport
    camera_info_manager
    )

#find_package(OpenCV 3 REQUIRED PATHS /usr/local/share/OpenCV NO_DEFAULT_PATH)
find_package(OpenCV REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
cisst_set_directories (cisstQt)

link_directories(
  ${OpenCV_LINK_DIRS}
)

set (SOURCE_FILES
     DeckLinkAPI/DeckLinkAPIDispatch.cpp
     code/DeckLinkCaptureDelegate.cpp
     code/PublishImageWorker.cpp
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
     include/DeckLinkCaptureDelegate.h
     include/PublishImageWorker.h
     )

set (CMAKE_AUTOMOC ON)
add_library(ourStereoVision ${HEADER_FILES} ${SOURCE_FILES})

target_link_libraries(ourStereoVision ${catkin_LIBRARIES})
target_link_libraries (ourStereoVision ${QT_LIBRARIES})
target_link_libraries (ourStereoVision ${OpenCV_LIBS})
cisst_target_link_libraries( ourStereoVision cisstQt )

## Declare a cpp executable
add_executable(testCameraCaptureDelegate examples/testingcameracapturedelegate.cpp)
target_link_libraries (testCameraCaptureDelegate ourStereoVision ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ourStereoVision testCameraCaptureDelegate
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
