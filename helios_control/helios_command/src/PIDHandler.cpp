//
// Created by simon on 25/06/18.
// Licensed to Simon CHANU
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64.h>

class PIDHandler {
    /**
     * Gère les 2 PID pour la commande linéaire et la commande angulaire
     */
public:
    PIDHandler(){
        // Sub
        sub_cmd = node.subscribe("cmd_vel", 1, &PIDHandler::updateCmdVel, this);
        sub_pose = node.subscribe("pose_est", 1, &PIDHandler::updateHeading, this);
        sub_twist = node.subscribe("twist_est", 1, &PIDHandler::updateVel, this);
        sub_pid_vel = node.subscribe("pid_vel/control_effort", 1, &PIDHandler::updatePIDVel, this);
        sub_pid_head = node.subscribe("pid_head/control_effort", 1, &PIDHandler::updatePIDHead, this);

        // Pub
        pub_pidStateHead = node.advertise<std_msgs::Float64>("pid_head/state", 1);
        pub_pidCmdHead = node.advertise<std_msgs::Float64>("pid_head/setpoint", 1);
        pub_pidStateSpeed = node.advertise<std_msgs::Float64>("pid_vel/state", 1);
        pub_pidCmdSpeed = node.advertise<std_msgs::Float64>("pid_vel/setpoint", 1);
        pub_cmd_thrust = node.advertise<geometry_msgs::Twist>("cmd_thrust", 1);

        cmd_thrust.linear.x = 0.0;
        cmd_thrust.angular.z = 0.0;
        heading.data = 0.0;
    }

    void updateHeading(const geometry_msgs::Pose::ConstPtr& msg){
        heading.data = tf::getYaw(msg->orientation);
        pub_pidStateHead.publish(heading);
    }

    void updateVel(const geometry_msgs::Twist::ConstPtr& msg){
        std_msgs::Float64 vel;
        vel.data = msg->linear.x;
        pub_pidStateSpeed.publish(vel);
    }

    void updateCmdVel(const geometry_msgs::Twist::ConstPtr& msg){
        std_msgs::Float64 cmdHead;
        std_msgs::Float64 cmdVel;
        cmdHead.data = msg->angular.z + heading.data; // On se replace dans un repere global
        cmdVel.data = msg->linear.x;
        pub_pidCmdHead.publish(cmdHead);
        pub_pidCmdSpeed.publish(cmdVel);
    }

    void updatePIDVel(const std_msgs::Float64::ConstPtr& msg){
        cmd_thrust.linear.x = msg->data;
    }

    void updatePIDHead(const std_msgs::Float64::ConstPtr& msg){
        cmd_thrust.angular.z = msg->data;
    }

    void spin(){
        ros::Rate loop(10);
        while (ros::ok()){
            // call all waiting callbacks
            ros::spinOnce();

            // publish the command
            pub_cmd_thrust.publish(cmd_thrust);

            loop.sleep();
        }
    }

private:
    // Node
    ros::NodeHandle node;

    // Subscribers
    ros::Subscriber sub_cmd;
    ros::Subscriber sub_pose;
    ros::Subscriber sub_twist;
    ros::Subscriber sub_pid_vel;
    ros::Subscriber sub_pid_head;

    // Publishers
    ros::Publisher pub_pidStateSpeed;
    ros::Publisher pub_pidCmdSpeed;
    ros::Publisher pub_pidStateHead;
    ros::Publisher pub_pidCmdHead;
    ros::Publisher pub_cmd_thrust;

    // Internal variables
    std_msgs::Float64 heading;
    geometry_msgs::Twist cmd_thrust;
};

int main(int argc, char **argv)
{
    // Node initialization
    std::cout << "Node initialization " << std::endl;
    ros::init(argc, argv, "PIDHandler");

    PIDHandler pid;

    pid.spin();
    return 0;
}

