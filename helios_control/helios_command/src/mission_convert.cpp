//
// Created by simon on 12/06/18.
//

#include "ros/ros.h"
#include "geodesy/utility.h"
#include "nav_msgs/Path.h"

class MissionConverter
{
public:
    MissionConverter(){
        sub_mission = node.subscribe("mission", 1, &MissionConverter::updateMission, this);
        pub_path = node.advertise<nav_msgs::Path>("path", 1);
    }

    void updateMission(const nav_msgs::Path::ConstPtr& msg){
        nav_msgs::Path path;
        path.header = msg->header;
        for(auto pose:msg->poses){
            Coordinates coord = latlon2meters(pose.pose.position.y, pose.pose.position.x);
            pose.pose.position.y = coord.y;
            pose.pose.position.x = coord.x;
            path.poses.push_back(pose);
        }
        pub_path.publish(path);
    }

    void spin(){
        ros::spin();
    }
private:
    // Node
    ros::NodeHandle node;

    // Sub
    ros::Subscriber sub_mission;

    // Pub
    ros::Publisher pub_path;
};

int main(int argc, char **argv)
{
    // Node initialization
    std::cout << "Node initialization " << std::endl;
    ros::init(argc, argv, "missionControl");

    MissionConverter missionConverter;

    missionConverter.spin();
    return 0;
}
