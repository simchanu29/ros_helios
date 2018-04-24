#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

nav_msgs::Path path;
geometry_msgs::PoseStamped wp;
std::vector<geometry_msgs::PoseStamped> plan;
ros::Publisher pub;

int main(int argc, char **argv)
{
    // Node initialization
    std::cout << "Mission pub Node initialization " << std::endl;
    ros::init(argc, argv, "path_pub");
    ros::NodeHandle node;

    pub = node.advertise<nav_msgs::Path>("path", 1);

    ros::Rate loop(1);
    loop.sleep();

    // Coordonnées ENU : x=EST y=NORD
    std::vector<double> coord_x = {0.0,30.0,-100.0,-100.0,100.0,50.0 ,0.0};
    std::vector<double> coord_y = {0.0,10.0, 30.0 , 80.0 ,150.0,150.0,0.0};

    for (int i = 0; i < coord_x.size(); ++i) {
        wp.pose.position.x = coord_x[i];
        wp.pose.position.y = coord_y[i];
        path.poses.push_back(wp);
    }

    pub.publish(path);

    return 0;
}
