//#pragma once

#include <boost/scoped_ptr.hpp>
#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "algoreg/lineFollow.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Path.h"
#include <geodesy/utility.h>
//#include <iostream>
//#include <math.h>

enum State{
    manual = 0,
    linefollowing = 1,
    stationKeeping = 2
};

class Regulation
{
public:
    Regulation(){
        // Subscribers
        sub_pose = node.subscribe("pose_est", 1, &Regulation::updatePose, this);
        sub_twist = node.subscribe("twist_est", 1, &Regulation::updateTwist, this);
        sub_line = node.subscribe("line", 1, &Regulation::updateFollowedLine, this);

        sub_state = node.subscribe("cmd_state", 1, &Regulation::updateCmdState, this);
        sub_speed = node.subscribe("cmd_max_speed", 1, &Regulation::updateMaxSpeed, this);

        // Publishers
        cmdVel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        // Internal variables
        cmd_state = manual;
        cmd_max_speed = 1.0; // m/s
        cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        cmd_vel.angular.z = 0;
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.linear.z = 0;
        pose.position.z = 0;
        pose.position.x = 0;
        pose.position.y = 0;
        line.poses = {geometry_msgs::PoseStamped(), geometry_msgs::PoseStamped()};
        const tf::Quaternion q = tf::createQuaternionFromYaw(0);
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();

        // Algo
        algoHead.reset(new LineFollow());
    }

    void updatePose(const geometry_msgs::Pose::ConstPtr& msg){
        pose.orientation = msg->orientation;
        pose.position = msg->position;
        ROS_DEBUG("I received an estimated position: ([%f], [%f], [%f])", pose.position.x, pose.position.y, pose.position.z);
    }

    void updateTwist(const geometry_msgs::Twist::ConstPtr& msg){
        twist.angular = msg->angular;
        twist.linear = msg->linear;
    }

    void updateFollowedLine(const nav_msgs::Path::ConstPtr& msg){
        line.poses[0] = msg->poses[0];
        line.poses[1] = msg->poses[1];
    }

    void updateCmdState(const std_msgs::Int8::ConstPtr& msg){
        cmd_state = msg->data;
    }

    void updateMaxSpeed(const std_msgs::Float64::ConstPtr& msg){
        cmd_max_speed = msg->data;
    }

    double computeSpeedFromHeading(double distance, double errorHeading){
        /**
         * Ralentit si on arrive proche de la cible et si le cap par rapport à l'angle voulu est mauvais
         */
        const double d_max = 15; // m
        const double err_angle_max = 1.57; // rad
        errorHeading = fmin(fabs(errorHeading), err_angle_max);
        distance = fmin(distance, d_max);
        return distance/d_max * (err_angle_max - errorHeading) * cmd_max_speed / err_angle_max;
    }

    void computelineFollow(){

        // Gestion des donnees
        const std::vector<double> wp_a = {
                line.poses[0].pose.position.x,
                line.poses[0].pose.position.y
        };
        const std::vector<double> wp_b = {
                line.poses[1].pose.position.x,
                line.poses[1].pose.position.y
        };
        const double headLine = atan2(wp_b[1]-wp_a[1],wp_b[0]-wp_a[0]); // ENU convention, angle from east

        const double head = tf::getYaw(pose.orientation);
        const std::vector<double> state_vec = {
                pose.position.x,
                pose.position.y,
                head
        };

        // Desired heading algo
        const double wantedHead = algoHead->run(wp_a, wp_b, state_vec); // par rapport au repère global

        // Desired speed from heading
        const double wantedSpeed = computeSpeedFromHeading(distance(state_vec[0], state_vec[1], wp_b[0], wp_b[1]), angle_rad(head,- wantedHead));

        // Commande
        cmd_vel.angular.z = angle_rad(wantedHead,- head); // rad
        cmd_vel.linear.x = wantedSpeed; // m/s

        printf("ang aim = [%f]\n",cmd_vel.angular.z);
        printf("lin vel = [%f]\n",cmd_vel.linear.x);
    }

    void stationKeep(){

    }

    void updateCommand(){
        printf(" --== Updating command ==-- \n");

        if(cmd_state==linefollowing){
            computelineFollow();
        } else if(cmd_state==stationKeeping){
            stationKeep();
        }

    }

    void spin(){

        ros::Rate loop(10);
        while (ros::ok()){

            // call all waiting callbacks
            ros::spinOnce();

            if(cmd_state!=manual){

                updateCommand();

                // publish the command
                cmdVel_pub.publish(cmd_vel);
            }

            loop.sleep();

        }
    }

private:
    // Node
    ros::NodeHandle node;

    // Subscriber
    ros::Subscriber sub_pose;
    ros::Subscriber sub_twist;
    ros::Subscriber sub_line;
    ros::Subscriber sub_state;
    ros::Subscriber sub_speed;

    // Publishers
    ros::Publisher cmdVel_pub;

    // Internal variables
    int cmd_state;
    double cmd_max_speed;
    geometry_msgs::Twist twist;
    geometry_msgs::Pose pose;
    geometry_msgs::Twist cmd_vel;
    nav_msgs::Path line;

    // Algo
    std::unique_ptr<Algo> algoHead;
};


int main(int argc, char **argv)
{
    // Node initialization
    std::cout << "Node initialization " << std::endl;
    ros::init(argc, argv, "regulation");

    Regulation regulation;

    regulation.spin();
    return 0;
}
