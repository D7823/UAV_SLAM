# Intro to each folder
Above ROS packages are used for the ROS software development, including orginal sources files and some customized codes.

scripts: python scripts to inplement individual nodes, such as BLE_DECK ROS driver and crazyflie controller.

video_stream_opencv: video/streaming driver for the stereoPi

camera_calibration: camera calibration for the stereoPi. This is should be done before performing SLAM task and generating 
                    disparity map

side_x_side_stereo: A ROS node for cutting the raw frame into left/right frame and publish lef/right camera information.
                    It also includes the launch file to run the whole system.

orb-slam2: node for orb-slam2 system

darket: node for running the yolo object detection

Go to each folder and build each nodes. Recommand order: video_stream_opencv->side_x_side_stereo->camera_calibration->orb_slam2->darknet->other script-based nodes
