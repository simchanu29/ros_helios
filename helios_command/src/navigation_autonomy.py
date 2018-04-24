#!/usr/bin/env python
# coding=utf-8

# Author : Simon CHANU
# Date : 06/03/2018
# Usage : Si on n'est pas en mode manuel, lorsque un waypoint est atteint, on envoie la ligne suivante

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Int16
import Utility.geodesy as geod


class PathPlanner:
    def __init__(self, dist_harvest):
        self.wp_harvestable = False
        self.pose = Pose()
        self.last_wp = PoseStamped()
        self.next_wp = PoseStamped()
        self.current_line = Path()
        self.current_line.poses = [self.last_wp, self.next_wp]
        self.path = Path()
        self.path.poses = self.current_line.poses
        self.line_number = 0
        self.dist_harvest = dist_harvest

    def update_line(self):
        self.last_wp = self.path.poses[self.line_number]
        self.next_wp = self.path.poses[min(self.line_number+1, len(self.path.poses)-1)]
        self.current_line.poses = [self.last_wp, self.next_wp]
        rospy.loginfo("update_line : following last_wp x:%f y:%f new_wp x:%f y:%f", self.last_wp.pose.position.x, self.last_wp.pose.position.y, self.next_wp.pose.position.x, self.next_wp.pose.position.y)

    def cb_path(self, msg):
        self.path = msg
        rospy.loginfo("New mission received : %s", self.path.poses)

        self.line_number = 0
        self.update_line()

        if self.wp_harvestable:
            rospy.loginfo("path : sendig last_wp x:%f y:%f new_wp x:%f y:%f", self.last_wp.pose.position.x, self.last_wp.pose.position.y, self.next_wp.pose.position.x, self.next_wp.pose.position.y)
            pub_line.publish(self.current_line)

    def cb_pose(self, msg):
        self.pose = msg

    def cb_cmd_state(self, msg):
        # 0 : manual mode
        # 1 : linefollow
        # 2 : stationkeeping
        last_state_wp_harvest = self.wp_harvestable
        self.wp_harvestable = (msg.data==0)

        if not last_state_wp_harvest and self.wp_harvestable:
            rospy.loginfo("cmd_state : sendig last_wp x:%f y:%f new_wp x:%f y:%f", self.last_wp.pose.position.x, self.last_wp.pose.position.y, self.next_wp.pose.position.x, self.next_wp.pose.position.y)
            pub_line.publish(self.current_line)

    def check_harvest_wp(self):
        x0 = self.last_wp.pose.position.x
        y0 = self.last_wp.pose.position.y
        x1 = self.next_wp.pose.position.x
        y1 = self.next_wp.pose.position.y

        dist = geod.dist_m(x0, y0, x1, y1)
        if dist<self.dist_harvest:
            rospy.loginfo("Wp can be harvested, dist=%f<%f? & %s", dist, self.dist_harvest, self.wp_harvestable)
            self.publish_line()

    def publish_line(self):
        if self.wp_harvestable:
            rospy.loginfo("Line %f harvested", self.line_number)
            self.line_number = min(self.line_number+1, len(self.path.poses)-2)
            self.update_line()
            rospy.loginfo("publish_line : sendig last_wp x:%f y:%f new_wp x:%f y:%f", self.last_wp.pose.position.x, self.last_wp.pose.position.y, self.next_wp.pose.position.x, self.next_wp.pose.position.y)
            pub_line.publish(self.current_line)


if __name__ == '__main__':

    rospy.init_node('navigation_autonomy')

    path_planner = PathPlanner(rospy.get_param('harvest_distance', 5))

    # Subscriber
    rospy.Subscriber('path', Path, path_planner.cb_path)
    rospy.Subscriber('pose_est', Pose, path_planner.cb_pose)
    rospy.Subscriber('cmd_state', Int16, path_planner.cb_cmd_state)

    # Publisher
    pub_line = rospy.Publisher('line', Path, queue_size=1)

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        path_planner.check_harvest_wp()
        rate.sleep()
