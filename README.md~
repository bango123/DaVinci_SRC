To the poor user who is trying to work with this system. There are a few things to note for the installation of the software.

1) Put this software into a catkin workspace. Basically mkdir -p catkin_ws/src/ and then put the software there. From there attempt to build after getting all of the dependencies.

2)Use this link as a starting point for building ciss-saw. The main thing to take away is the dependencies that you need to install. 

https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#13-building-using-catkin-build-tools-for-ros


3) The video capture cards in the computer are from a Black Magic. We will be using the Decklink SDK to configure the video. So please install this dependency on the computer before buliding



To run DVRK:
rosrun dvrk_robot dvrk_console_json -j ~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/ucsd-dVRK/console-MTMR-PSM1-MTML-PSM2-Teleop.json -i ~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/ros-io-PSM2.json -i ~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/ros-io-PSM1.json -i ~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/ros-io-MTML.json -i ~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/ros-io-MTMR.json

To have cameras running by themselves publishing to ROS:
~/catkin_ws/devel/lib/StereoVision/testCameraCaptureDelegate

From there, use image_viewer to display the images. Like so:
rosrun image_view image_view image:=/stereo/left/image_raw
rosrun image_view image_view image:=/stereo/right/image_raw

or if delay is running:
rosrun image_view image_view image:=/stereo/left_delay/image_raw
rosrun image_view image_view image:=/stereo/right_delay/image_raw

To run stereo calibration:
1) Run stereo calibration pacakge
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.025 right:=/stereo/right/image_raw left:=/stereo/left/image_raw right_camera:=/stereo/right left_camera:=/stereo/left approximate_sync:=true --approximate=0.01
Note: change --size and --square (m units) to fit the checkerboard used
2) Save the file (do not try to commit results, it WILL crash). Then extract the output to ~/.ros/camera_info/
3) Restart the camera streams
4) To run disparity run this command:
ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc stereo:=/stereo/ image:=image_raw _approximate_sync:=true
5) To view disparity run this:
rosrun image_view stereo_view stereo:=/stereo image:=image_raw _approximate_sync:=True _queue_size:=10
