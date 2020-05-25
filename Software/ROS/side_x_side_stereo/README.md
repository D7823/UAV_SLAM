# ROS node to split left/right image and camera information

This node is based on this ROS package: https://github.com/dbc/side_x_side_stereo

Since the stereoPi driver gives one large frame as output, we use this node to split it to left and right frame and publish the camera information generated from camera calibration to stere_image_proc node for rectification.

Intro to the files:

src: source codes of the node

* backup.cpp: original code from the author
* side_by_side_stereo_node.cpp: code for splitting images
* stereo_splitter_rectifier.cpp: code for splitting images and publishing camera information (Mostly used in our system)

launch: launch files to run the whole system

* disparity.launch: launch file for running stereoPi driver, splitter/rectifier node, stereo_image_proc node
* whole.launch: launch file for running stereoPi driver, solitter/rectifier node, stereo_image_proc node, orbslam2 node

Note: the launch files has to be modified before running (check the comment inside the files) 

## Build
1. Copy the whole folder to your catkin workspace

2. run
```
catkin build
```
in your catkin workspace

3. check/run after sourcing the setup.sh file
```
rosrun side_x_side_stereo_SR_node
```
use rostopic to check whether there are published topics specified in the parameters section
```
rostopic list
```

4. Install the stereo_image_proc node for future usage, can use the apt-get:
```
sudo apt-get install ros-kinetic(or melodic)-sterep-image-proc
```

## Parameters

#### Subscribing:
By default:

* /rtsp/image_raw

#### Publishing:
By default:

* /stereopi/left/image_raw
* /stereopi/right/image_raw
* /stereopi/left/camera_info
* /stereopi/right/camera_info

## Special Notes
1. The stereo_SR_node takes the camera calibration result as input but it is currently hardcorded. If having new calibration result, modification is needed.

2. After building all required nodes and modification to the launch files, simply use roslaunch to run the launch files.




