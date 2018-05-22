#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

nav_msgs::Path line;
geometry_msgs::PoseStamped wp1;
geometry_msgs::PoseStamped wp2;
ros::Publisher pub;

int main(int argc, char **argv)
{
    // Node initialization
    std::cout << "Mission pub Node initialization " << std::endl;
    ros::init(argc, argv, "line_pub");
    ros::NodeHandle node;

    pub = node.advertise<nav_msgs::Path>("line", 1);

    ros::Rate loop(1);
    loop.sleep();

    // Coordonnées ENU : x=EST y=NORD
    wp1.pose.position.x = 0.0;
    wp1.pose.position.y = 0.0;
    wp2.pose.position.x = 100.0;
    wp2.pose.position.y = 50.0;
    line.poses = {wp1, wp2};

    while (ros::ok()){
      pub.publish(line);
      loop.sleep();
    }

    return 0;
}
