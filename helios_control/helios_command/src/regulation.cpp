#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geodesy/utility.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Path.h"
//#include <iostream>
//#include <math.h>

enum State{
    manual = 0,
    linefollowing = 1,
    stationKeeping = 2
};

class Regulation
{
public:
    Regulation(){
        // Subscribers
        sub_pose = node.subscribe("pose_est", 1, &Regulation::updatePoseReal, this);
        sub_twist = node.subscribe("twist_est", 1, &Regulation::updateTwistReal, this);
        sub_line = node.subscribe("line", 1, &Regulation::updateFollowedLine, this);
        sub_state = node.subscribe("cmd_state", 1, &Regulation::updateCmdState, this);

        // Publishers
        cmdVel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        // Internal variables
        cmd_state = manual;

        cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        cmd_vel.angular.z = 0;
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.linear.z = 0;

        pose_real.position.z = 0;
        pose_real.position.x = 0;
        pose_real.position.y = 0;

        followedLine_prevwp_x = 0; //x en metres
        followedLine_prevwp_y = 0; //y en metres
        followedLine_prevwp_z = 0;
        followedLine_nextwp_x = 0; //x en metres
        followedLine_nextwp_y = 0; //y en metres
        followedLine_nextwp_z = 0;

        const tf::Quaternion q = tf::createQuaternionFromYaw(0);
        pose_real.orientation.z = q.z();
        pose_real.orientation.w = q.w();
        pose_real.orientation.x = q.x();
        pose_real.orientation.y = q.y();
    }

    void updatePoseReal(const geometry_msgs::Pose::ConstPtr& msg){
        pose_real.orientation = msg->orientation;
        pose_real.position = msg->position;
        ROS_DEBUG("I received an estimated position: ([%f], [%f], [%f])", pose_real.position.x, pose_real.position.y, pose_real.position.z);
    }

    void updateTwistReal(const geometry_msgs::Twist::ConstPtr& msg){
        twist_real.angular = msg->angular;
        twist_real.linear = msg->linear;
    }

    void updateFollowedLine(const nav_msgs::Path::ConstPtr& msg){
        followedLine_prevwp_x = msg->poses[0].pose.position.x;
        followedLine_prevwp_y = msg->poses[0].pose.position.y;
        followedLine_nextwp_x = msg->poses[1].pose.position.x;
        followedLine_nextwp_y = msg->poses[1].pose.position.y;
    }

    void updateCmdState(const std_msgs::Int8::ConstPtr& msg){
        cmd_state = msg->data;
    }

    void lineFollow(){

        double ax = followedLine_prevwp_x;
        printf("ax = [%f]\n",ax);
        double ay = followedLine_prevwp_y;
        printf("ay = [%f]\n",ay);
        double bx = followedLine_nextwp_x;
        printf("bx = [%f]\n",bx);
        double by = followedLine_nextwp_y;
        printf("by = [%f]\n",by);
        const double x = pose_real.position.x;
        printf("x = [%f]\n",x);
        const double y = pose_real.position.y;
        printf("y = [%f]\n",y);

        double head = tf::getYaw(pose_real.orientation);
        printf("head = [%f]\n",head);

        double headLine = atan2(by-ay,bx-ax); // ENU convention, angle from east
        printf("headLine = [%f]\n",headLine);

        // Si le bateau dépasse la ligne
        const double angleNextWp2Boat = atan2(y-by,x-bx); // ENU convention, angle from east
        printf("angleNextWp2Boat = [%f]\n",angleNextWp2Boat);
        printf("angle_rad(angleNextWp2Boat,-headLine) = [%f]\n",angle_rad(angleNextWp2Boat,-headLine));
        if(fabs(angle_rad(angleNextWp2Boat,-headLine))<M_PI/2.0){

            printf("===================== Inverting WayPoints\n");

            const double tmpx = ax;
            const double tmpy = ay;
            ax = bx;
            ay = by;
            bx = tmpx;
            by = tmpy;

            headLine = atan2(by-ay,bx-ax); // ENU convention, angle from east
        }

        double dist2Line;
        if( sqrt( pow(bx-ax,2) + pow(by-ay,2)) != 0){
            dist2Line = ((bx-ax)*(y-ay) - (by-ay)*(x-ax)) / sqrt( pow(bx-ax,2) + pow(bx-ax,2));
        } else{ dist2Line = 100; }

        printf("dist2Line = [%f]\n",dist2Line);

        const double error = atan(dist2Line);
        printf("headLine = [%f]\n", headLine);
        printf("error = [%f]\n", error);
        printf("headLine - error = [%f]\n", headLine - error);

        const double wantedHead = angle_rad(headLine, -error);
//        const double wantedHead = atan(tan(headLine - atan(dist2Line)));
        printf("wantedHead = [%f]\n", wantedHead); // Le cap voulu par rapport au repere global

        // Commande finale
        // TODO implémenter un PID
        const double twist = angle_rad( wantedHead,- head)/5.0;
	    const double tauTwist = 2.0;
        cmd_vel.angular.z = twist;
	    //cmd_vel.angular.z = (exp(tauTwist*twist)-1.0)/10.0;
        printf("twist = [%f]\n",twist);

//        cmd_vel.linear.x = atan(1/brakeDist*dist2Obj); // Le 1/1* c'est pour que le bateau ralentisse à 1m
        const double cmdLin = fabs(angle_rad(headLine,- head)); // Le bateau ralenti si il n'est pas en face de la ligne
        const double tauLin = 5.0;
        cmd_vel.linear.x = exp(-tauLin/M_PI*cmdLin)-exp(-tauLin);
        printf("lin vel = [%f]\n",cmd_vel.linear.x);

    }

    void stationKeep(){

    }

    void updateCommand(){
        printf(" --== Updating command ==-- \n");

        if(cmd_state==linefollowing){
            lineFollow();
        } else if(cmd_state==stationKeeping){
            stationKeep();
        }

    }

    void spin(){

        ros::Rate loop(10);

        while (ros::ok()){

            // call all waiting callbacks
            ros::spinOnce();

            if(cmd_state!=manual){

                updateCommand();

                // publish the command
                cmdVel_pub.publish(cmd_vel);
            }

            loop.sleep();

        }
    }

private:
    // Node
    ros::NodeHandle node;

    // Subscriber
    ros::Subscriber sub_pose;
    ros::Subscriber sub_twist;
    ros::Subscriber sub_line;
    ros::Subscriber sub_state;

    // Publishers
    ros::Publisher cmdVel_pub;

    // Internal variables
    int cmd_state;
    geometry_msgs::Twist pose;
    geometry_msgs::Twist twist_real;
    geometry_msgs::Pose pose_real;
    geometry_msgs::Twist cmd_vel;
    double followedLine_prevwp_x;
    double followedLine_prevwp_y;
    double followedLine_prevwp_z;
    double followedLine_nextwp_x;
    double followedLine_nextwp_y;
    double followedLine_nextwp_z;
};


int main(int argc, char **argv)
{
    // Node initialization
    std::cout << "Node initialization " << std::endl;
    ros::init(argc, argv, "regulation");

    Regulation regulation;

    regulation.spin();
    return 0;
}
