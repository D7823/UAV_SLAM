# ORB-SLAM2 Node

We use this implementation in our system: https://github.com/appliedAI-Initiative/orb_slam_2_ros

1. Follow the building procedure in the above website to build this node in your catkin workspace

2. Copy the yaml files to the /orb_slam2/config folder (following the same file formate when you have new calibration results)

3. Copy the launch files to the /ros/launch folder

4. Copy the rviz files to the /ros/config folder

5. Test

Testing the monocular slam:
```
roscore
rosrun rviz rviz -d /your-path-to/rviz_config_mono.rviz
roslaunch video_stream_opencv video_monoPi.launch
roslaunch orb_slam2_ros orb_slam2_pi_mono.launch
```
You should see some slam results on the rviz. If not, check each launch file to ensure the topic consistance.
(video_monoPi.launch checks image topic name, video file; orb_slam2_pi_mono.launch check for image topic name)

Testing the stereo slam (require the stereo_image_proc package):
```
roscore
rosrun rviz rviz -d /your-path-to/rviz_config_mono.rviz
```
If you have modified the whole.launch file in side_x_side_stereo folder, simply run:
```
roslaunch side_x_side_stereo whole.launch
```
Otherwise:
```
roslaunch video_stream_opencv video_stereoPi.launch
rosrun side_x_side_stereo stereo_SR_node
ROS_NAMESPACE=stereopi rosrun stereo_image_proc stereo_image_proc
roslaunch orb_slam2_ros orb_slam2_pi_stereo.launch
```

# Special Notes
1. The current orb-slam2 node is not able to self-rectified (at least not able to work when I tried it out) so it requires the stereo_image_proc node to do the rectification. 

2. Some parameters can be manipulated in the launch files, such as the "min_num_kf_in_map" which defines how many keyframe should get to not reset the map

3. The map can be saved through rosservice, example:
```
rosservice call /orb_slam2_stereo/save_map map.bin
```
