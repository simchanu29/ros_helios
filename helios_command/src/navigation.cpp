#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geodesy/utility.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Path.h"
//#include <iostream>
//#include <math.h>

class Regulation
{
public:
    Regulation(){
        // Subscribers
        gpsfix_sub = node.subscribe("/gps/fix", 1, &Regulation::updatePoseReal, this);
        gpsvel_sub = node.subscribe("/gps/vel", 1, &Regulation::updatePoseReal, this);
        imu_sub = node.subscribe("imu", 1, &Regulation::updateTwistReal, this);

        // Publishers
        pose_pub = node.advertise<geometry_msgs::Pose>("pose_est", 1);
        twist_pub = node.advertise<geometry_msgs::Twist>("twist_est", 1);

        // Internal variables
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;

        pose_est.position.z = 0;
        pose_est.position.x = 0;
        pose_est.position.y = 0;

        const tf::Quaternion q = tf::createQuaternionFromYaw(0);
        pose_est.orientation.z = q.z();
        pose_est.orientation.w = q.w();
        pose_est.orientation.x = q.x();
        pose_est.orientation.y = q.y();
    }

    void updateGpsVel(const geometry_msgs::TwistStamped::ConstPtr& msg){
        gpsvel.header = msg->header;
        gpsvel.twist = msg->twist;
        ROS_DEBUG("gps vel received");
    }


    void updateGpsFix(const sensor_msgs::NavSatFix::ConstPtr& msg){
      gpsfix.header = msg->header;
      gpsfix.status = msg->status;
      gpsfix.longitude = msg->latitude;
      gpsfix.altitude = msg->altitude;
      gpsfix.position_covariance = msg->position_covariance;
      gpsfix.position_covariance_type = msg->position_covariance_type;
      ROS_DEBUG("gps fix: ([%f], [%f])", gpsfix.longitude, gpsfix.latitude);

        // Passage en coordonnées Lambert 93
    }

    void updateImu(const sensor_msgs::Imu::ConstPtr& msg){
        twist_real.angular = msg->angular;
        twist_real.linear = msg->linear;
    }

    void spin(){

        ros::Rate loop(10);

        while (ros::ok()){

            // call all waiting callbacks
            ros::spinOnce();

            if(cmd_state!=manual){

                updateCommand();

                // publish the command
                cmdVel_pub.publish(twist);
            }

            loop.sleep();

        }
    }

private:
    // Node
    ros::NodeHandle node;

    // Suscribers
    ros::Subscriber cmdState_sub;
    ros::Subscriber twistReal_sub;
    ros::Subscriber anglePing_sub;
    ros::Subscriber obj_sub;
    ros::Subscriber poseReal_sub;

    // Publishers
    ros::Publisher cmdVel_pub;

    // Internal variables
    geometry_msgs::Twist pose;
    geometry_msgs::Twist twist_real;
    geometry_msgs::Pose pose_est;
    geometry_msgs::Twist twist;

    sensor_msgs::Imu imu;
    sensor_msgs::NavSatFix gpsfix;
    geometry_msgs::TwistStamped gpsvel;
};


int main(int argc, char **argv)
{
    // Node initialization
    std::cout << "Node initialization " << std::endl;
    ros::init(argc, argv, "bubble_reg");

    Navigation navigation;

    navigation.spin();
    return 0;
}
