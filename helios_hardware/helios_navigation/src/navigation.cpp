#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geodesy/utility.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Path.h"
#include "tf/tf.h"

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>

#include "geodesy/utility.h"
//#include <proj_api.h>

//#include <iostream>
//#include <math.h>

geometry_msgs::Vector3 invRotateVector3Msg(geometry_msgs::Quaternion rotation, geometry_msgs::Vector3 vector){
    tf::Quaternion rotationTF;
    tf::quaternionMsgToTF(rotation, rotationTF);
    tf::Vector3 vectorTF;
    tf::vector3MsgToTF(vector, vectorTF);
    tf::Vector3 rotated_vectorTF = tf::quatRotate(rotationTF.inverse(), vectorTF);
    geometry_msgs::Vector3 rotated_vector;
    tf::vector3TFToMsg(rotated_vectorTF, rotated_vector);
    return rotated_vector;
}

class Navigation
{
public:
    Navigation(){
        // Subscribers
        sub_fix = node.subscribe("/gps/fix", 1, &Navigation::updateGpsFix, this);
        sub_vel = node.subscribe("/gps/vel", 1, &Navigation::updateGpsVel, this);
        sub_imu = node.subscribe("imu", 1, &Navigation::updateImu, this);

        // Publishers
        pose_pub = node.advertise<geometry_msgs::Pose>("pose_est", 1);
        twist_pub = node.advertise<geometry_msgs::Twist>("twist_est", 1);
        marker_pub = node.advertise<visualization_msgs::Marker>("rviz_robot", 1);

        // Internal variables
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;

        pose.position.z = 0;
        pose.position.x = 0;
        pose.position.y = 0;

        tf::Quaternion q = tf::createQuaternionFromRPY(0,0,0);
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();

        meters_coord.x = 0;
        meters_coord.y = 0;
    }

    void updateGpsVel(const geometry_msgs::TwistStamped::ConstPtr& msg){
        gpsvel.header = msg->header;
        gpsvel.twist = msg->twist;
        ROS_DEBUG("gps vel received");
    }

    void updateGpsFix(const sensor_msgs::NavSatFix::ConstPtr& msg){
      gpsfix.header = msg->header;
      gpsfix.status = msg->status;
      gpsfix.latitude = msg->latitude;
      gpsfix.longitude = msg->longitude;
      gpsfix.altitude = msg->altitude;
      gpsfix.position_covariance = msg->position_covariance;
      gpsfix.position_covariance_type = msg->position_covariance_type;
      ROS_DEBUG("gps fix: ([%f], [%f])", gpsfix.longitude, gpsfix.latitude);

      // Passage en coordonnées meters
      meters_coord = latlon2meters(gpsfix.latitude, gpsfix.longitude);
//      meters_coord.x = gpsfix.longitude;
//      meters_coord.y = gpsfix.latitude;
    }

    void updateImu(const sensor_msgs::Imu::ConstPtr& msg){
        imu.header = msg->header;
        imu.orientation = msg->orientation;
        imu.orientation_covariance = msg->orientation_covariance;
        imu.angular_velocity = msg->angular_velocity;
        imu.angular_velocity_covariance = msg->angular_velocity_covariance;
        imu.linear_acceleration = msg->linear_acceleration;
        imu.linear_acceleration_covariance = msg->linear_acceleration_covariance;
        ROS_DEBUG("imu: ([%f])", tf::getYaw(imu.orientation));

    }

    void sensor_fusion(){
        // TODO : faire la fusion de donnée avec un filtre de kalman

        // position
        pose.position.x = meters_coord.x;
        pose.position.y = meters_coord.y;
        pose.position.z = gpsfix.altitude;
        pose.orientation = imu.orientation;

        // speed : linear
        // Passage de la vitesse gps dans un repere local

        ROS_INFO("gpsvel.twist.linear.x = %f", gpsvel.twist.linear.x);
        ROS_INFO("gpsvel.twist.linear.y = %f", gpsvel.twist.linear.y);
        ROS_INFO("gpsvel.twist.linear.z = %f", gpsvel.twist.linear.z);
        twist.linear = invRotateVector3Msg(pose.orientation, gpsvel.twist.linear);
        ROS_INFO("twist.linear.x = %f", twist.linear.x);
        ROS_INFO("twist.linear.y = %f", twist.linear.y);
        ROS_INFO("twist.linear.z = %f", twist.linear.z);

//        twist.linear.x = gpsvel.twist.linear.x;
//        twist.linear.y = gpsvel.twist.linear.y;
//        twist.linear.z = gpsvel.twist.linear.z;

        // speed : angular
        twist.angular = imu.angular_velocity;

        // Broadcast robot tf
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "global";
        transformStamped.child_frame_id = "robot";
        transformStamped.transform.translation.x = pose.position.x;
        transformStamped.transform.translation.y = pose.position.y;
        transformStamped.transform.translation.z = pose.position.z;
        tf2::Quaternion q;
        transformStamped.transform.rotation.x = pose.orientation.x;
        transformStamped.transform.rotation.y = pose.orientation.y;
        transformStamped.transform.rotation.z = pose.orientation.z;
        transformStamped.transform.rotation.w = pose.orientation.w;

        // Publish Marker
        robot.header.frame_id = "robot";
        robot.header.stamp = ros::Time::now();
        robot.ns = "";
        robot.id = 0;
        robot.type = visualization_msgs::Marker::ARROW;
        robot.action = visualization_msgs::Marker::ADD;
        robot.pose.position.x = 0;
        robot.pose.position.y = 0;
        robot.pose.position.z = 0;
        robot.pose.orientation.x = 0;
        robot.pose.orientation.y = 0;
        robot.pose.orientation.z = 0;
        robot.pose.orientation.w = 0;
        robot.scale.x = 10.0;
        robot.scale.y = 2.0;
        robot.scale.z = 2.0;
        robot.color.r = 0.0f;
        robot.color.g = 1.0f;
        robot.color.b = 0.0f;
        robot.color.a = 1.0;
        robot.lifetime = ros::Duration();
    }

    void spin(){

        ros::Rate loop(10);

        while (ros::ok()){

            // call all waiting callbacks
            ros::spinOnce();

                sensor_fusion();

                // publish data
                pose_pub.publish(pose);
                twist_pub.publish(twist);
                br.sendTransform(transformStamped);
                marker_pub.publish(robot);

            loop.sleep();

        }
    }

private:
    // Node
    ros::NodeHandle node;

    // Subscriber
    ros::Subscriber sub_fix;
    ros::Subscriber sub_vel;
    ros::Subscriber sub_imu;

    // Publishers
    ros::Publisher pose_pub;
    ros::Publisher twist_pub;
    ros::Publisher marker_pub;

    // Internal variables
    geometry_msgs::Pose pose;
    geometry_msgs::Twist twist;

    sensor_msgs::Imu imu;
    sensor_msgs::NavSatFix gpsfix;
    geometry_msgs::TwistStamped gpsvel;

    visualization_msgs::Marker robot;

    // tf
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    Coordinates meters_coord;
};


int main(int argc, char **argv)
{
    // Node initialization
    std::cout << "Node initialization " << std::endl;
    ros::init(argc, argv, "sensor_fusion");

    Navigation navigation;

    navigation.spin();
    return 0;
}
