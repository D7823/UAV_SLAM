# Camera Calibration

Camera calibration is a process to get the intrinsic parameters of the camera and those parameters are required in the tasks of stereo image pair rectification and SLAM.

For more info on camera calibration, you may check: https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html

We use the [camera_calibration](http://wiki.ros.org/camera_calibration) ros node to do the calibration for the stereoPi. You should install this node first.
```
sudo apt-get install ros-kinetic(or melodic)-camera-calibration
```

And follow [this tutorial](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration) to finish the calibration process.

#### Process
Here is the camera calibration process with the camera calibration ros node and the stereoPi driver

1. Connecet the stereoPi(in 3D mode) and your machine to the same network and then Run the driver and calibration node, example:
```
roscore
roslaunch video_stream_opencv stereopi_rtsp.launch
rosrun side_x_side_stereo side_x_side_stereo_node
rosrun camera_calibration cameracalibrator.py --approximate 0.5 --size 8x6 --square 0.108 right:=/stereopi/right/image_raw left:=/stereopi/left/image_raw
```
You should see the interface of the camera_calibration node with the real-time streaming.

Parameters:
* approximate: the time of streaming latency in seconds
* size: the internal corner count of the chessboard
* square: the length of the square in meters

2. 

    a. Hold the chessboard in front of the stereopi and move it around till it collects enough frames for calibration. 
    b. Click the calibration button
    c. Wait for calibration finish and keep holding the chessboard, check the epi value if lower than 0.25 then the carlibration is okay
    d. save the calibration result and generate the yaml file for slam or other tasks following their file format





