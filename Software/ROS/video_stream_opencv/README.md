# StereoPi Driver
This node is to build the stereoPi driver

### 1. Install video_stream_opencv ros package 

Clone the video_stream_opencv package to your catkin space:
```
git clone git@github.com:ros-drivers/video_stream_opencv.git
```
To build the node catkin run
```
catkin build
```
in your catkin folder

Check the building by running a sample video(remeber to source it after successfully built):
```
roslaunch video_stream_opencv video_file.launch
```


### 2. Copy the files
