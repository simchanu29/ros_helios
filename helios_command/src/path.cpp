#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "bubble_msgs/Line.h"
#include "geometry_msgs/Pose.h"
#include "geodesy/utility.h"
#include "tf/transform_datatypes.h"
//#include <iostream>
//#include <math.h>

enum State{
    manual = 0,
    blackBoxResearch = 1,
    stationKeeping = 2
};

class Path
{
public:
    Path(){
        // Subscribers
        poseReal_sub = node.subscribe("pose_est", 1, &Path::updatePoseReal, this);
        anglePing_sub = node.subscribe("angle_ping", 1, &Path::updateAnglePing, this);
        cmdState_sub = node.subscribe("cmd_state", 1, &Path::updateCmdState, this);

        // Publishers
        line_pub = node.advertise<bubble_msgs::Line>("line", 1);

        // Internal variables
        pose_real.position.z = 0;
        pose_real.position.x = 0;
        pose_real.position.y = 0;

        line.prevWaypoint.x = 0; line.prevWaypoint.y = 0;
        line.nextWaypoint.x = 0; line.nextWaypoint.y = 0;

        angle_ping = -3; // -180 = non trouvé

        const tf::Quaternion q = tf::createQuaternionFromYaw(0);
        pose_real.orientation.z = q.z();
        pose_real.orientation.w = q.w();
        pose_real.orientation.x = q.x();
        pose_real.orientation.y = q.y();

        //researchPath.xVec.push_back(2.0);  researchPath.yVec.push_back(2.0);
        //researchPath.xVec.push_back(-2.0); researchPath.yVec.push_back(-2.0);
	    //researchPath.step = 0;
	    researchPath = generateResPath("square",0,0,20,20);

	std::cout << "Initialization done" << std::endl;
    }

    void updatePoseReal(const geometry_msgs::Pose::ConstPtr& msg){
        pose_real.orientation = msg->orientation;
        pose_real.position = msg->position;
//	    printf("received : ([%f], [%f], [%f])", pose_real.position.x, pose_real.position.y, pose_real.position.z);
        ROS_DEBUG("I received an estimated position: ([%f], [%f], [%f])", pose_real.position.x, pose_real.position.y, pose_real.position.z);
    }

    void updateCmdState(const std_msgs::Int8::ConstPtr& msg){
        cmd_state = msg->data;
    }

    void updateAnglePing(const std_msgs::Float64::ConstPtr& msg){
        angle_ping = msg->data;
    }

    void updateCommand(){
        // Si on a atteint le prochain waypoint, on regarde si on a besoin de changer de cap

//	    std::cout << "Assign xWp and yWp " << std::endl;
        const double xWp = line.nextWaypoint.x ;
        const double yWp = line.nextWaypoint.y;
//	    std::cout << "Assign x and y " << std::endl;
	    const double x = pose_real.position.x;
        const double y = pose_real.position.y;
	
//	    std::cout << "Computing distance to waypoint " << std::endl;
        const double dist2Wp = sqrt( pow(xWp-x,2) + pow(yWp-y,2) );

        if( dist2Wp < 2 ){ //m

            std::cout << "Waypoint harvested :" << std::endl;

            line.prevWaypoint = line.nextWaypoint;

            printf("Angle_ping = %f deg \n",angle_ping/M_PI*180.0);
            if(fabs(angle_ping)>M_PI/2.0){
                printf("Wrong detection \n");
                // La boite noire n'a pas encore été localisée donc on continue le chemin habituel
                researchPath.step++;

                line.nextWaypoint.x = researchPath.xVec[researchPath.step];
                line.nextWaypoint.y = researchPath.yVec[researchPath.step];

            } else{

                printf("Good detection \n");

                // L'angle donné par le système audio est bon donc on se dirige vers celui-ci

                double dist = 10; //m
                // Point à 5m dans la direction détectée. L'IMU donne des angles en convention NED, getYaw donne des radians
                // Passage en convention ENU
                const double cap = tf::getYaw(pose_real.orientation);
                line.nextWaypoint.x = pose_real.position.x + dist*cos( angle_rad(angle_ping,+ cap) );
                line.nextWaypoint.y = pose_real.position.y + dist*sin( angle_rad(angle_ping,+ cap) );

            }
        }
//	std::cout << "Command updated " << std::endl;
    }

    void spin(){

        ros::Rate loop(2);

        while (ros::ok()){

            // call all waiting callbacks
          
//            std::cout << "Spin once " << std::endl;
	          ros::spinOnce();

            // publish the command
            if(cmd_state!=manual){
//		            std::cout << "Updating command " << std::endl;
                updateCommand();
//		            std::cout << "Publishing line " << std::endl;

                line_pub.publish(line);
//		            std::cout << "Line published " << std::endl;
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
    ros::Subscriber poseReal_sub;

    // Publishers
    ros::Publisher line_pub;

    // Research path
    struct ResearchPath{
        std::vector<float> xVec;
        std::vector<float> yVec;
        int step;
    };

    // Internal variables
    int cmd_state;
    double angle_ping;
    geometry_msgs::Pose pose_real;
    bubble_msgs::Line line;
    ResearchPath researchPath;

    ResearchPath generateResPath(std::string shape, float xIni, float yIni, float xLen, float yLen){

        //Init
        ResearchPath resPath = Path::ResearchPath();
        const int squareLength = 4;
        int pathXLen = (int) floor(xLen / squareLength);
        int pathYLen = (int) floor(yLen / squareLength);


        //Square
        if(shape=="square"){
            for (int i = 0; i < pathYLen; ++i) {
                int yCoord = squareLength*i;

                if(fmod(i, 2) == 0){
                    for (int j = 0; j < pathXLen; ++j) {
                        int xCoord = squareLength*j;

                        resPath.xVec.push_back(xIni+xCoord);
                        resPath.yVec.push_back(yIni+yCoord);
                    }
                }
                else{
                    for (int j = pathXLen; j > 0; --j) {
                        int xCoord = squareLength*j;

                        resPath.xVec.push_back(xIni+xCoord);
                        resPath.yVec.push_back(yIni+yCoord);
                    }
                }



            }
        }
        resPath.step = 0;
        return resPath;
    }
};


int main(int argc, char **argv)
{
    // Node initialization
    std::cout << "Node initialization " << std::endl;
    ros::init(argc, argv, "path");

    Path path;

    std::cout << "Node spinning " << std::endl;
    path.spin();
    return 0;
}
