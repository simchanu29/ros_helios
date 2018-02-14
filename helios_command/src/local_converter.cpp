#include <tf/transform_datatypes.h>
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/TimeReference.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TwistStamped.h"
#include "geodesy/utility.h"
#include "tf/transform_datatypes.h"
//#include <iostream>
//#include <math.h>

class Map
{
public:
    Map(){
        // Subscribers
        gpsFix_sub = node.subscribe("gps/fix", 1, &Map::updateGpsFix, this);
//        gpsVel_sub = node.subscribe("gps/vel", 1, &Map::updateGpsVel, this);
//        timeRef_sub = node.subscribe("gps/time_reference", 1, &Map::updateTimeRef, this);
        imu_sub = node.subscribe("imu/imu", 1, &Map::updateImu, this);

        // Publishers
        pose_pub = node.advertise<geometry_msgs::Pose>("pose_est", 1);

        // Internal variables
        pose.position.z = 0;
        pose.position.x = 0;
        pose.position.y = 0;

        tf::Quaternion q = tf::createQuaternionFromRPY(0,0,0);
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
    }

    void updateGpsFix(const sensor_msgs::NavSatFix::ConstPtr& msg){
        pose.position.x = longDeg2meters(msg->longitude, latOrigin, longOrigin);
        pose.position.y = latDeg2meters(msg->latitude, latOrigin);
        pose.position.z = msg->altitude;
//	printf("I received an estimated position: ([%f], [%f], [%f])\n", msg->latitude, msg->longitude, msg->altitude);
//	printf("I received an estimated position: ([%f], [%f], [%f])\n", pose.position.x, pose.position.y, pose.position.z);

        ROS_DEBUG("I received an estimated position: ([%f], [%f], [%f])", pose.position.x, pose.position.y, pose.position.z);
    }

//    void updateGpsVel(const geometry_msgs::TwistStamped::ConstPtr& msg){
//        ROS_DEBUG("I received an estimated position: ([%f], [%f], [%f])", x, y, z);
//    }

//    void updateTimeRef(const sensor_msgs::NavSatFix::ConstPtr& msg){
//        ROS_DEBUG("I received an estimated position: ([%f], [%f], [%f])", x, y, z);
//    }

    void updateImu(const sensor_msgs::Imu::ConstPtr& msg){

        const double yaw = tf::getYaw(msg->orientation) + M_PI/2.0;
        pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        printf("estimated yaw : [%f]\n",yaw);
    }

    void spin(){

        ros::Rate loop(10);

        while (ros::ok()){

            // call all waiting callbacks
            ros::spinOnce();

            // publish the command
            pose_pub.publish(pose);

            loop.sleep();

        }
    }

private:
    // Node
    ros::NodeHandle node;

    // Suscribers
    ros::Subscriber gpsFix_sub;
    ros::Subscriber gpsVel_sub;
    ros::Subscriber timeRef_sub;
    ros::Subscriber imu_sub;

    // Publishers
    ros::Publisher pose_pub;

    // Internal variables
    geometry_msgs::Pose pose;
    double latOrigin = 48.1992;
    double longOrigin = -3.0151;
    float x, y, z;
    float theta;
};


int main(int argc, char **argv)
{
    // Node initialization
    std::cout << "Node initialization " << std::endl;
    ros::init(argc, argv, "local_converter");

    Map map;

    map.spin();
    return 0;
}
