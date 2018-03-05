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

#include <proj_api.h>

#include "geodesy/utility.h"
//#include <proj_api.h>

//#include <iostream>
//#include <math.h>


struct Coordinates{
    double x;
    double y;
};

Coordinates latlon2lamb(double lat, double lon){

    Coordinates coord;
    projPJ pj_lambert, pj_latlong;



    if (!(pj_lambert = pj_init_plus("+proj=lcc +lat_1=49 +lat_2=44 +lat_0=46.5 +lon_0=3 +x_0=700000 +y_0=6600000 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs ")))
    {
        printf("error lambert \n");
        exit(1);
    }

    if (!(pj_latlong = pj_init_plus("+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs")))
    {
        printf("error latlong \n");
        exit(1);
    }

    lat *= DEG_TO_RAD;
    lon *= DEG_TO_RAD;

    pj_transform(pj_latlong, pj_lambert, 1, 1, &lat, &lon, NULL );

    //printf("X: %lf \nY: %lf\n", x, y);
    coord.x = lat;
    coord.y = lon;
    return coord;
}


class Navigation
{
public:
    Navigation(){
        // Subscribers
        node.subscribe("/gps/fix", 1, &Navigation::updateGpsFix, this);
        node.subscribe("/gps/vel", 1, &Navigation::updateGpsVel, this);
        node.subscribe("imu", 1, &Navigation::updateImu, this);

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

        pose.position.z = 0;
        pose.position.x = 0;
        pose.position.y = 0;

        tf::Quaternion q = tf::createQuaternionFromRPY(0,0,0);
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();

        lambert_coord.x = 0;
        lambert_coord.y = 0;
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

      // Passage en coordonnées Lambert 93
      lambert_coord = latlon2lamb(gpsfix.latitude, gpsfix.longitude);
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
        pose.position.x = lambert_coord.x;
        pose.position.y = lambert_coord.y;
        pose.position.z = gpsfix.altitude;
        pose.orientation = imu.orientation;

        // speed : linear
        twist.linear.x = gpsvel.twist.linear.x;
        twist.linear.y = gpsvel.twist.linear.y;
        twist.linear.z = gpsvel.twist.linear.z;

        // speed : angular
        twist.angular = imu.angular_velocity;
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

            loop.sleep();

        }
    }

private:
    // Node
    ros::NodeHandle node;

    // Publishers
    ros::Publisher pose_pub;
    ros::Publisher twist_pub;

    // Internal variables
    geometry_msgs::Pose pose;
    geometry_msgs::Twist twist;

    sensor_msgs::Imu imu;
    sensor_msgs::NavSatFix gpsfix;
    geometry_msgs::TwistStamped gpsvel;

    Coordinates lambert_coord;
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
