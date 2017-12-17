The ros components built in here are to be used for working with the DaVinci.

camera_delay: 
        - This adds an amount of delay onto the image streams in rostopic /stereo/right_sync/image_raw and /stereo/left_sync/image_raw
	- The output delay stream will be in rostopic /stereo/right_delay/image_raw and /stereo/left_delay/image_raw
To run it:
        - rosrun camera_delay camera_delay
Has the following ros param:
        - \delay = delay value in ms being added


camera_sync:
        - This synchronzises the images and forces the the L/R images to published together and have the same timestamp.
        - This should be used with stereo_vision since stereo_vision does NOT make sure the left and right images are published together.
        - Subscribes to /stereo/right/image_raw and /stereo/left/image_raw
To run it:
        - rosrun camera_sync camera_sync


