To the poor user who is trying to work with this system. There are a few things to note for the installation of the software.

1) Put this software into a catkin workspace. Basically mkdir -p catkin_ws/src/ and then put the software there. From there attempt to build after getting all of the dependencies.

2)Use this link as a starting point for building ciss-saw. The main thing to take away is the dependencies that you need to install. 

https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#13-building-using-catkin-build-tools-for-ros


3) The video capture cards in the computer are from a Black Magic. We will be using the Decklink SDK to configure the video. So please install this dependency on the computer before buliding

To run the most up-to date stuff use launchfile:
    roscore
    roslaunch dvrk_robot dvrk_console.launch
    roslaunch stereo_vision stereo_vision_delay.launch

To record use the following launchfile, this will be saved to the NVME drive:
    roslaunch dvrk_robot dvrk_camera_console_record.launch <name-of-folder>

To convert bag cam folder to .avi file, this will be output in HOME folder:
    rosrun stereo_ros_blocks bag2vid <filepath-to-folder-with-cam-bags> <output-file-name>.avi

To run DVRK:
rosrun dvrk_robot dvrk_console_json -j ~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/ucsd-dVRK/console-MTMR-PSM1-MTML-PSM2-Teleop.json -i ~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/ros-io-PSM2.json -i ~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/ros-io-PSM1.json -i ~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/ros-io-MTML.json -i ~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/ros-io-MTMR.json



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
