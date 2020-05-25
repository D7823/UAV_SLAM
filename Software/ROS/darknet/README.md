# Darket YOLO

We use this yolo implementation to do object detection: https://github.com/leggedrobotics/darknet_ros

1. Follow the building instructions in the above website to build the darknet node in your catkin space

2. Copy the launch file to darknet_ros/launch folder. It takes the rectified left image for object detection. Just use roslaunch to run it when you are running the stereo version orb-SLAM2.

3. Testing. 

- Find a image and then use image_publisher to publish it in the ROS
- use rostopic to find the published image name
- replace the image name in the yolo_v3.launch to the published image name you found
- roslaunch darknet_ros yolo_v3.launch
