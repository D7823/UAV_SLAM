#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Twist

def move():
    # Starts a new node
    rospy.init_node('CF_commander', anonymous=False)
    distance_publisher = rospy.Publisher("cf_command", Twist, queue_size=1)
    dis_msg = Twist()
    time.sleep(1)
    dis_msg.linear.x = 0.5
    dis_msg.linear.y = 0
    dis_msg.linear.z = 0
    dis_msg.angular.x = 0
    dis_msg.angular.y = 0
    dis_msg.angular.z = 0
    distance_publisher.publish(dis_msg)
    time.sleep(5)
    dis_msg.linear.x = 0
    dis_msg.linear.y = 0.5
    distance_publisher.publish(dis_msg)
    time.sleep(5)
    dis_msg.linear.y = 0
    dis_msg.angular.x = 90
    distance_publisher.publish(dis_msg)
    rospy.spin()

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException: pass
