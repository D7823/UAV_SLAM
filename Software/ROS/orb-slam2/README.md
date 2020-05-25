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


