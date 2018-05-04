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
    double x_origin = 398898;
    double y_origin = 5354422;
    std::vector<double> coord_x_loc = {0.0,30.0,-100.0,-100.0,100.0,50.0 ,0.0};
    std::vector<double> coord_y_loc = {0.0,10.0, 30.0 , 80.0 ,150.0,150.0,0.0};

    std::vector<double> coord_x;
    std::vector<double> coord_y;
    for(auto coord : coord_x_loc){
        coord_x.push_back(coord + x_origin);
    }
    for(auto coord : coord_y_loc){
        coord_y.push_back(coord + y_origin);
    }

    for (int i = 0; i < coord_x.size(); ++i) {
        wp.pose.position.x = coord_x[i];
        wp.pose.position.y = coord_y[i];
        path.poses.push_back(wp);
    }

    pub.publish(path);

    return 0;
}
