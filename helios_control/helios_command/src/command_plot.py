#!/usr/bin/env python
# coding=utf-8

import rospy
import numpy as np
import Utility.geometry as geom
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_msgs.msg import Int16
import pyqtgraph as plt
import tf


class Plotter():
    def __init__(self):

        # Subscriber vars
        self.motors = {}
        self.pose = Pose()
        self.twist = Twist()
        self.theta = 0.0
        self.x = float(rospy.get_param('~x_init', 0.0))
        self.y = float(rospy.get_param('~y_init', 0.0))
        self.cmd_vx = 0.0
        self.cmd_theta = 0.0
        self.path = [[], []]
        self.last_wp = Pose().position
        self.next_wp = Pose().position

        # Init plot
        self.win = plt.GraphicsWindow()
        self.fig = self.win.addPlot(title="Display command")
        self.fig.setXRange(self.x - 20, self.x + 20)
        self.fig.setYRange(self.y - 20, self.y + 20)

        # Init path_planned
        self.plt_planned_path = self.fig.plot()
        # Init path_active
        self.plt_active_path = self.fig.plot()
        # Init trace
        self.plt_trace = self.fig.plot()
        # Init boat
        self.plt_boat = self.fig.plot()
        # Init cmd
        self.plt_twist = self.fig.plot()

        self.trace = [[], [], []]

    def update_pose(self, msg):
        self.pose = msg
        self.x = self.pose.position.x
        self.y = self.pose.position.y
        self.theta = tf.transformations.euler_from_quaternion((self.pose.orientation.x,
                                                               self.pose.orientation.y,
                                                               self.pose.orientation.z,
                                                               self.pose.orientation.w))[2]

    def update_twist(self, msg):
        self.twist = msg
        self.cmd_vx = self.twist.linear.x
        self.cmd_theta = self.twist.angular.z

    def update_planned_path(self, msg):
        self.path[0] = []
        self.path[1] = []
        for pose in msg.poses:
            self.path[0].append(pose.pose.position.x)
            self.path[1].append(pose.pose.position.y)

    def update_active_path(self, msg):
        self.last_wp = msg.poses[0].pose.position
        self.next_wp = msg.poses[1].pose.position

    def update_trace(self):
        MAX_SIZE = 500
        self.trace[0].append(self.x)
        self.trace[1].append(self.y)
        self.trace[2].append(self.theta)
        if len(self.trace[0]) > MAX_SIZE:
            del (self.trace[0][0])
        if len(self.trace[1]) > MAX_SIZE:
            del (self.trace[1][0])
        if len(self.trace[2]) > MAX_SIZE:
            del (self.trace[2][0])

    def process(self):
        # Les angles représentés ici sont en ENU

        # print "====== Plotting boat"
        hull = geom.draw_kayak(self.theta, self.x, self.y)
        self.plt_boat.setData(hull[0], hull[1], pen=plt.mkPen('l', width=2))
        self.update_trace()
        self.plt_trace.setData(self.trace[0], self.trace[1], pen=plt.mkPen('g'))

        # print "====== Plotting twist"
        angle_coeff = 1.0
        vec = np.array([[0.0, self.cmd_vx * np.cos(self.cmd_theta*angle_coeff)],
                        [0.0, self.cmd_vx * np.sin(self.cmd_theta*angle_coeff)],
                        [1.0, 1.0]])
        vec = geom.homothetie_vec(vec, self.theta,
                                  self.x, self.y)
        self.plt_twist.setData(vec[0],
                               vec[1], pen=plt.mkPen('b', width=2))

        # print "====== Plotting planned path"
        self.plt_planned_path.setData(self.path[0], self.path[1], pen=plt.mkPen('l', width=1))

        # print "====== Plotting active path"
        self.plt_active_path.setData([self.last_wp.x, self.next_wp.x], [self.last_wp.y, self.next_wp.y],
                                     pen=plt.mkPen('r', width=2))


if __name__ == '__main__':
    rospy.init_node('command_plot')

    plot = Plotter()

    rospy.Subscriber('pose_est', Pose, plot.update_pose)
    rospy.Subscriber('cmd_vel', Twist, plot.update_twist)
    rospy.Subscriber('path', Path, plot.update_planned_path)
    rospy.Subscriber('line', Path, plot.update_active_path)

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        plot.process()
        plt.QtGui.QApplication.processEvents()
        rate.sleep()
