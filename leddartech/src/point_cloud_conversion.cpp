#include <ros/ros.h>

#include "leddartech/PointCloudConversion.h"
#include "sensor_msgs/point_cloud_conversion.h"


leddartech::PointCloudConversion srv;


bool point_cloud_conversion(leddartech::PointCloudConversion::Request &req,
                           leddartech::PointCloudConversion::Response &res)
{
  sensor_msgs::PointCloud cloud_in = req.cloud;
  sensor_msgs::PointCloud2 cloud_out;
  sensor_msgs::convertPointCloudToPointCloud2(cloud_in, cloud_out);
  res.cloud2 = cloud_out;
  ROS_INFO("request");
  ROS_INFO("sending back response");
  return true;
}

void callback(const sensor_msgs::PointCloud& msg)
{
  srv.request.cloud = msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_conversion");
  ros::NodeHandle n;
  ros::ServiceClient client;
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2> ("converted_cloud", 1);
  ros::Subscriber sub = n.subscribe("assembled_cloud", 100, callback);
  
  

  ros::ServiceServer service = n.advertiseService("pcl_converter", point_cloud_conversion);
  ROS_INFO("Ready to convert.");


  
  client = n.serviceClient<leddartech::PointCloudConversion>("client_converter"); 


  while (ros::ok())  
  {
    // Make the service call
    if (client.call(srv))
    {
      ROS_INFO("Published Cloud2") ;
      pub.publish(srv.response);
    }
    else
    {
      ROS_ERROR("Error making service call for conversion\n") ;
    }
  }

  

  return 0;
}


