#include "ros/ros.h"
#include "nav_msgs/Path.h"

nav_msgs::Path line;
ros::Publisher pub;

int main(int argc, char **argv)
{
    // Node initialization
    std::cout << "Mission pub Node initialization " << std::endl;
    ros::init(argc, argv, "mission_pub");
    ros::NodeHandle node;

    pub = node.advertise<nav_msgs::Path>("line", 1);

    ros::Rate loop(1);
    loop.sleep();

    line.poses[0].pose.position.x = 0.0;
    line.poses[0].pose.position.y = 0.0;
    line.poses[1].pose.position.x = 1.0;
    line.poses[1].pose.position.y = 1.0;
    pub.publish(line);

    ros::spin();
    return 0;
}
