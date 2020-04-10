#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/2M'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

class drone():
	def __init__(self):
		cflib.crtp.init_drivers(enable_debug_driver=False)
		#self.scf=SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache'))
		rospy.init_node('Crazyflie', anonymous=False)
		self.sensor_data_publisher = rospy.Publisher("OA", String, queue_size=10)
		self.command_subscriber = rospy.Subscriber("cf_command", Twist, self.update_command)
		self.rate = rospy.Rate(1)

		# eventhough using the twist message, it represents distance instead of velocity
		# should generate our own message later
		# the velocity use default, linear = 0.2, angular = 360/5
		# +/- xlin distance to front/back, +/- ylin distance to left/right
		# +/- zlin distance to up/down, +/- xa angle turnning to left/right
	def update_command(self, cmd):
		xlin = cmd.linear.x
		ylin = cmd.linear.y
		zlin = cmd.linear.z
		# only turn left or right
		xang = cmd.angular.x
		rospy.loginfo(rospy.get_caller_id() + "Command received")
		self.excute_command(xlin, ylin, zlin, xang)

	# only perform one kind of motion at a time
	def excute_command(self, xl, yl, zl, xa):
		with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
			with MotionCommander(scf) as mc:
				if xl != 0:
            				mc.forward(xl)
            			elif yl != 0:
            				mc.left(yl)
            			elif zl != 0:
            				mc.up(zl)
            			elif xa > 0:
            				mc.turn_left(xa)
            			elif xa < 0:
            				mc.turn_right(-xa)
            			else:
            				mc.stop()
    
   	 # crazyflie listening for command and publishing data 
    	def active(self):
    		while not rospy.is_shutdown():
        		hello_str = "Crazyflie fake data %s" % rospy.get_time()
        		rospy.loginfo(hello_str)
        		self.sensor_data_publisher.publish(hello_str)
        		self.rate.sleep()

if __name__ == '__main__':
    try:
	cf = drone()
	cf.active()
    except rospy.ROSInterruptException:
        pass
