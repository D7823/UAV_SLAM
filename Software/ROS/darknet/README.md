# Object detection and target distance caculator

We use this darknet yolo implementation to do object detection: https://github.com/leggedrobotics/darknet_ros

1. Follow the building instructions in the above website to build the darknet node in your catkin space

2. Testing the yolo with an image:  

      1. Find a image online and then use image_publisher to publish it in the ROS

      2. use rostopic to find the published image name

      3. replace the image name in the yolo_v3.launch to the published image name you found

      4. roslaunch darknet_ros yolo_v3.launch
      
3. Copy the launch file to darknet_ros/launch folder. It takes the rectified left image for object detection. You can use roslaunch to run it when you are running the stereo version ORB-SLAM2.

4. Creat a script folder under /path/ and copy the distance_calculator.py to the script folder.
      
   ...

You might also use the disparity.launch file in the side_x_side_stereo project to extract left frame from videos for object detection and check the distance_Calculator node in our system. 
