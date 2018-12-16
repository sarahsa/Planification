#!/usr/bin/env python
import rospy
import socket
import sys

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

isobstacle = False
collision_range = 0.5

#Creation du node pour traitement de data du laser 
class Bumpynode(object):
    def __init__(self):
        self.LidarSub = rospy.Subscriber('scan', LaserScan, self.callback)

    def callback(self, msg):
	global isobstacle
        print('Range Min : ' + str(min(msg.ranges)))
        if min(msg.ranges) < collision_range:
            isobstacle = True
            print('Inside!')
        else:
            isobstacle = False


class Keyboardreceiver(object):
    def __init__(self):
        self.KeyBoardrec = rospy.Subscriber('cmd_vel_input', Twist, self.callback)
        self.KeyBoardTrans = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        #For debugging purposes
	print('Keyboard is working') 

    def callback(self, msg):
        global isobstacle
	print('isObstacle : ' + str(isobstacle))

        if not isobstacle:
            self.KeyBoardTrans.publish(msg)

        else:

            msg.linear.x = -0.2; msg.linear.y = 0; msg.linear.z = 0
            msg.angular.x = 0; msg.angular.y = 0; msg.angular.z = 0
            self.KeyBoardTrans.publish(msg)
            rospy.sleep(1.)




if __name__ == '__main__':
    try:
        rospy.init_node('Bumpy')
        bumpy_node = Bumpynode()
        keyboard_node = Keyboardreceiver()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

