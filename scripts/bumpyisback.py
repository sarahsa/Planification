X#!/usr/bin/env python
import rospy
import socket
import sys
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

turnleft = False
turnright = False

robotwideness = 0.6
collision_range = 0.5

class Bumpynode(object):
	def __init__(self):
		self.LidarSub = rospy.Subscriber('scan', LaserScan, self.callback)

	def callback(self, msg):
	global turnleft
		global turnright
	angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
		xy = np.array([[msg.ranges[i]*np.cos(angles[i]) for i in range(len(msg.ranges))],
				[msg.ranges[i]*np.sin(angles[i]) for i in range(len(msg.ranges))]])
		print('Range Min : ' + str(min(msg.ranges)))
		
		if min(xy[0,:]) < collision_range and 0 < min(xy[1,:]) < (robotwideness/2):
		turnright = True
		turnleft = False
	else if min(xy[0,:]) < collision_range and 0 > min(xy[1,:]) and abs(min(xy[1,:])) > (robotwideness/2):
				turnright = False
				turnleft = True 
	else:
				turnright = False
				turnleft = True

class Keyboardreceiver(object):
	def __init__(self):
		self.KeyBoardTrans = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
		print(' yolo ')
	def callback(self, msg):
		global turnleft
		global turnright

		if turnleft:
			msg.linear.x = 0.2; msg.linear.y = 0; msg.linear.z = 0
			msg.angular.x = 0; msg.angular.y = 0; msg.angular.z = 0
			self.KeyBoardTrans.publish(msg)
	else if turnright:
			msg.linear.x = 0; msg.linear.y = 0; msg.linear.z = 0
			msg.angular.x = 0; msg.angular.y = 0; msg.angular.z = -0.2
			self.KeyBoardTrans.publish(msg)
			rospy.sleep(1.)
	else:

			msg.linear.x = 0; msg.linear.y = 0; msg.linear.z = 0
			msg.angular.x = 0; msg.angular.y = 0; msg.angular.z = 0.2
			self.KeyBoardTrans.publish(msg)
			rospy.sleep(1.)




if __name__ == '__main__':
	try:
		rospy.init_node('Bumpyisback')
		bumpy_node = Bumpynode()
		keyboard_node = Keyboardreceiver()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

