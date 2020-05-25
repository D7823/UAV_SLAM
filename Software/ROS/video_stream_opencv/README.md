# StereoPi Driver
This node is to build the stereoPi driver based on the package: https://github.com/ros-drivers/video_stream_opencv

### 1. Install video_stream_opencv ros package 

Clone the video_stream_opencv package to your catkin space:
```
git clone git@github.com:ros-drivers/video_stream_opencv.git
```
To build the node catkin run
```
catkin build
```
in your catkin folder.

Check the building by running a sample video(remeber to source it after successfully built):
```
roslaunch video_stream_opencv video_file.launch
```


### 2. Copy the files

Copy the launch files in this folder to the launch folder of the video_stream_opencv folder

* video_monopi.launch: launch file for the video captured by stereoPi in 2D mode -- mono_Pi.mp4

* video_stereo.launch: launch file for the video captured by stereoPi in 3D mode -- stereo_Pi.mp4

* stereopi_rtsp.launch: launch file for streaming video from stereoPi in 3D mode though network rtsp protocol

(Modify the rtsp address if static IP address is different or using a dynamic IP address with the router, more about how to
configure static IP address for stereoPi in the following section. Make sure the stereopi running the S.L.P image and using RTSP streaming)

Copy the mp4 files in this folder to the test folder of the video_stream_opencv folder

### 3. Test
replace the xxx.launch to your target launch file and run  
```
roslaunch video_stream_opencv xxx.launch
```

### 4. static IP address set up for stereoPi (optional)
Directly connect the stereoPi and the ubuntu machine using static IP address can significantly improve the streaming latency.

The stereoPi can be accessed through SSH and to set up the static IP address, the file system should be unmounted first. Instructions can be found here: https://wiki.stereopi.com/index.php?title=SLP_(StereoPi_Livestream_Playground)_Raspbian_Image








