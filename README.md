To the poor user who is trying to work with this system. There are a few things to note for the installation of the software.

1) Put this software into a catkin workspace. Basically mkdir -p catkin_ws/src/ and then put the software there. From there attempt to build after getting all of the dependencies.

2)Use this link as a starting point for building ciss-saw. The main thing to take away is the dependencies that you need to install. 

https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#13-building-using-catkin-build-tools-for-ros


3) The video capture cards in the computer are from a Black Magic. We will be using the Decklink SDK to configure the video. So please install this dependency on the computer before buliding
