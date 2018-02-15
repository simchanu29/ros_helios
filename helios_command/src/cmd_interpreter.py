#!/usr/bin/env python
# coding=utf-8

# Author : Simon

import rospy
import numpy as np
from helios_command.msg import PwmCmd
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64, Float64

class Model_char(object):
    """
    Modèle de déplacement mode char avec configurations
    """
    def __init__(self, max_cmd=0.7, coeff_rot=1.0, coeff_lin=1.0):
        self.max_cmd = max_cmd
        self.coeff_rot = coeff_rot
        self.coeff_lin = coeff_lin

    def command(self, lin, ang):
        """
        lin et ang vont de -1.0 à 1.0
        """
        # modification lineaire des commande
        lin = lin*self.coeff_lin
        ang = ang*self.coeff_rot

        # bornage de la commande angulaire par le maximum
        np.clip(ang, self.max_cmd, -self.max_cmd)

        # Calcul de la différence en absolu
        diff = np.abs(lin) + np.abs(ang) - self.max_cmd

        m1 = lin + ang - np.sign(lin)*diff
        m2 = lin - ang - np.sign(lin)*diff

        return [m1, m2]

    def cb_max(self, msg):
        self.max_cmd = msg.data

def cb_pwm(msg):

    print 'msg.linear.x :', msg.linear.x  # pin en int
    print 'msg.angular.z :', msg.angular.z  # commande en float

    # Gestion de la commande
    # lin = msg.linear.x*100.0
    # ang = msg.angular.z*100.0
    cmd = char.command(lin, ang)

    # HOTFIX se référer au fichier de config TODO
    # Calcul de la commande
    msg1 = PwmCmd()
    msg1.command = cmd[0]*100.0
    msg1.pin = 0

    msg2 = PwmCmd()
    msg2.command = cmd[1]*100.0
    msg2.pin = 1

    # Envoi de la commande
    pub_pwm.publish(msg1)
    pub_pwm.publish(msg2)
    pub_mot_left.publish(int(4000*(1+cmd[0])))
    pub_mot_right.publish(int(4000*(1+cmd[1])))

if __name__ == '__main__':

    rospy.init_node('cmd interpreter')
    rospy.loginfo("driver Node Initialised")

    maximum = rospy.get_param('~maximum', "0.7")
    coeff_rot = rospy.get_param('~coeff_rot', "1.0")
    coeff_lin = rospy.get_param('~coeff_lin', "1.0")
    char = Model_char(float(maximum), float(coeff_rot), float(coeff_lin))

    sub = rospy.get_param("~sub_topic", "cmd_vel")
    rospy.Subscriber(sub , Twist, cb_pwm)
    rospy.Subscriber('cmd_max_speed' , Float64, cb_max)

    pub_pwm = rospy.Publisher('pwm_cmd', PwmCmd, queue_size=1)

    # HOTFIX pour fabrice BathyBoatNav TODO a mettre propre
    pub_mot_left = rospy.Publisher('left_mot', Int64, queue_size=1000)
    pub_mot_right = rospy.Publisher('right_mot', Int64, queue_size=1000)

    rospy.spin()
