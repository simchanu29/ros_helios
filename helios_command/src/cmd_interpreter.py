#!/usr/bin/env python
# coding=utf-8

# Author : Simon

import rospy
import numpy as np
from helios_command import PwmCmd
from geometry_msgs.msg import Twist

def cb_pwm(self, msg):

    print 'msg.linear.x :', msg.linear.x  # pin en int
    print 'msg.angular.z :', msg.angular.z  # commande en float

    # Gestion de la commande
	lin = msg.linear.x*10.0
	ang = msg.angular.z/np.pi*100.0

    # Calcul de la commande
	msg1 = PwmCmd()
	msg1.command = lin + ang
	msg1.pin = 12

	msg2 = PwmCmd()
	msg2.command = lin + ang
	msg2.pin = 13
	
    # Envoi de la commande
    pub.publish(msg1)
	pub.publish(msg2)

if __name__ == '__main__':

    rospy.init_node('driver_maestro')
    rospy.loginfo("driver_maestro Node Initialised")

	sub = rospy.Subscriber('twist_key', Twist)
    pub = rospy.Publisher('pwm_cmd', PwmCmd, cb_pwm)

    rospy.spin()
