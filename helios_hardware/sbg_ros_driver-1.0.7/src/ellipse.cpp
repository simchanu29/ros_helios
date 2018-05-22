#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <sbgEComLib.h>
#include <sbgEComIds.h>
#include <cmath>

sensor_msgs::Imu imu_msg;
sensor_msgs::NavSatFix nav_msg;
sensor_msgs::NavSatFix gps_msg;
geometry_msgs::Vector3Stamped attitude_msg;
sensor_msgs::MagneticField mag_msg;
geometry_msgs::Vector3Stamped gps_altitude_msg;
geometry_msgs::Vector3Stamped imu_altitude_msg;
geometry_msgs::TwistStamped nav_vel_msg;

bool new_imu_msg;
bool new_nav_msg;
bool new_attitude_msg;
bool new_gps_msg;
bool new_mag_msg;
bool new_nav_vel;

unsigned int mask_rtk_status = 0x0FC0; // 32 bits

// unsigned int createMask(unsigned int a, unsigned int b) // (6,11)
// {
//    unsigned int r = 0;
//    for (unsigned int i=a; i<=b; i++)
//        r |= 1 << i;
//    return r;
// }

/*!
 *  Callback definition called each time a new log is received.
 *  \param[in]  pHandle                 Valid handle on the sbgECom instance that has called this callback.
 *  \param[in]  msgClass                Class of the message we have received
 *  \param[in]  msg                   Message ID of the log received.
 *  \param[in]  pLogData                Contains the received log data as an union.
 *  \param[in]  pUserArg                Optional user supplied argument.
 *  \return                       SBG_NO_ERROR if the received log has been used successfully.
 */
SbgErrorCode onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg)
{
  // float time_of_week;
  switch (msg){
    case SBG_ECOM_LOG_EKF_QUAT:
      imu_msg.orientation.x = pLogData->ekfQuatData.quaternion[1];
      imu_msg.orientation.y = pLogData->ekfQuatData.quaternion[2];
      imu_msg.orientation.z = pLogData->ekfQuatData.quaternion[3];
      imu_msg.orientation.w = pLogData->ekfQuatData.quaternion[0];
      new_imu_msg = true;
      break;

    case SBG_ECOM_LOG_EKF_EULER:
    	attitude_msg.vector.x = pLogData->ekfEulerData.euler[0];
    	attitude_msg.vector.y = pLogData->ekfEulerData.euler[1];
    	attitude_msg.vector.z = pLogData->ekfEulerData.euler[2];
		new_attitude_msg = true;
    	break;

    case SBG_ECOM_LOG_EKF_NAV:
      nav_msg.latitude  = pLogData->ekfNavData.position[0];
      nav_msg.longitude = pLogData->ekfNavData.position[1];
      nav_msg.altitude  = pLogData->ekfNavData.position[2];
	  nav_msg.position_covariance[0] = pLogData->ekfNavData.positionStdDev[0];
	  nav_msg.position_covariance[4] = pLogData->ekfNavData.positionStdDev[1];
	  nav_msg.position_covariance[8] = pLogData->ekfNavData.positionStdDev[2];
	  nav_msg.position_covariance_type = nav_msg.COVARIANCE_TYPE_DIAGONAL_KNOWN;

	  imu_altitude_msg.vector.x = pLogData->ekfNavData.position[2];
	  imu_altitude_msg.vector.y = pLogData->ekfNavData.undulation;

      new_nav_msg = true;

	  nav_vel_msg.twist.linear.x = pLogData->ekfNavData.velocity[0]*cos(attitude_msg.vector.z);
	  nav_vel_msg.twist.linear.y = pLogData->ekfNavData.velocity[1]*sin(attitude_msg.vector.z);
	  nav_vel_msg.twist.linear.z = pLogData->ekfNavData.velocity[2];
	  new_nav_vel = true;

      break;

    case SBG_ECOM_LOG_IMU_DATA:
      imu_msg.linear_acceleration.x = pLogData->imuData.accelerometers[0];
      imu_msg.linear_acceleration.y = pLogData->imuData.accelerometers[1];
      imu_msg.linear_acceleration.z = pLogData->imuData.accelerometers[2];

      imu_msg.angular_velocity.x = pLogData->imuData.gyroscopes[0];
      imu_msg.angular_velocity.y = pLogData->imuData.gyroscopes[1];
      imu_msg.angular_velocity.z = pLogData->imuData.gyroscopes[2];
      new_imu_msg = true;
      break;

    case SBG_ECOM_LOG_MAG:
    	mag_msg.magnetic_field.x = pLogData->magData.magnetometers[0];
    	mag_msg.magnetic_field.y = pLogData->magData.magnetometers[1];
    	mag_msg.magnetic_field.z = pLogData->magData.magnetometers[2];
    	new_mag_msg = true;
    	break;

    // case SBG_ECOM_LOG_UTC_TIME:
    //   pInsSBG->new_time = true;
    //   pInsSBG->year          = pLogData->utcData.year;
    //   pInsSBG->month         = pLogData->utcData.month;
    //   pInsSBG->day           = pLogData->utcData.day;
    //   pInsSBG->hour          = pLogData->utcData.hour;
    //   pInsSBG->minute        = pLogData->utcData.minute;
    //   pInsSBG->second        = pLogData->utcData.second;
    //   pInsSBG->nanoSecond    = pLogData->utcData.nanoSecond;
    //   pInsSBG->gpsTimeOfWeek = pLogData->utcData.gpsTimeOfWeek;
      // break;
    case SBG_ECOM_LOG_GPS1_POS:
    	gps_msg.latitude  = pLogData->gpsPosData.latitude;
      	gps_msg.longitude = pLogData->gpsPosData.longitude;
      	gps_msg.altitude  = pLogData->gpsPosData.altitude;
		gps_msg.position_covariance[0] = pLogData->gpsPosData.latitudeAccuracy;
		gps_msg.position_covariance[4] = pLogData->gpsPosData.longitudeAccuracy;
		gps_msg.position_covariance[8] = pLogData->gpsPosData.altitudeAccuracy;
		gps_msg.position_covariance_type = gps_msg.COVARIANCE_TYPE_DIAGONAL_KNOWN;

		gps_altitude_msg.vector.x = pLogData->gpsPosData.altitude;
		gps_altitude_msg.vector.y = pLogData->gpsPosData.undulation;

		// STATUS gpsPosData[6-11]
		// 0 SBG_ECOM_POS_NO_SOLUTION No valid solution available.
		// 1 SBG_ECOM_POS_UNKNOWN_TYPE An unknown solution type has been computed.
		// 2 SBG_ECOM_POS_SINGLE Single point solution position.
		// 3 SBG_ECOM_POS_PSRDIFF Standard Pseudorange Differential Solution (DGPS).
		// 4 SBG_ECOM_POS_SBAS SBAS satellite used for differential corrections.
		// 5 SBG_ECOM_POS_OMNISTAR Omnistar VBS Position (L1 sub-meter).
		// 6 SBG_ECOM_POS_RTK_FLOAT Floating RTK ambiguity solution (20 cms RTK).
		// 7 SBG_ECOM_POS_RTK_INT Integer RTK ambiguity solution (2 cms RTK).
		// 8 SBG_ECOM_POS_PPP_FLOAT Precise Point Positioning with float ambiguities
		// 9 SBG_ECOM_POS_PPP_INT Precise Point Positioning with fixed ambiguities
		// 10 SBG_ECOM_POS_FIXED Fixed location solution position
      	gps_msg.status.status = (pLogData->gpsPosData.status & mask_rtk_status) >> 6;

    	new_gps_msg = true;
    	break;
    	
    default:
    	// ROS_INFO("message unknown");
      break;
  }
  return SBG_NO_ERROR;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbg_ellipse");

  ros::NodeHandle n;
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 10);
  ros::Publisher nav_pub = n.advertise<sensor_msgs::NavSatFix>("nav", 10);
  ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("fix", 10);
  ros::Publisher attitude_pub = n.advertise<geometry_msgs::Vector3Stamped>("imu_attitude", 10);
  ros::Publisher nav_vel_pub = n.advertise<geometry_msgs::TwistStamped>("nav_vel", 10);
  ros::Publisher mag_pub = n.advertise<sensor_msgs::MagneticField>("mag", 10);

  ros::Publisher imu_altitude_pub = n.advertise<geometry_msgs::Vector3Stamped>("nav_altitude", 10); 
  ros::Publisher gps_altitude_pub = n.advertise<geometry_msgs::Vector3Stamped>("gps_altitude", 10);

  std::string uart_port;
  int uart_baud_rate;

  n.param<std::string>("uart_port", uart_port, "/dev/sbg");
  n.param<int>("uart_baud_rate", uart_baud_rate, 115200);

    // ********************* Initialize the SBG  *********************
  SbgEComHandle       comHandle;
  SbgInterface        sbgInterface;
  SbgEComDeviceInfo   deviceInfo;
  SbgErrorCode        errorCode;

  errorCode = sbgInterfaceSerialCreate(&sbgInterface, uart_port.c_str(), uart_baud_rate);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgInterfaceSerialCreate Error");}

  errorCode = sbgEComInit(&comHandle, &sbgInterface); // Init the SBG
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComInit Error");}

  errorCode = sbgEComCmdGetInfo(&comHandle, &deviceInfo); // Get device info
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdGetInfo Error");}

  ROS_INFO("CONNEXTION SET-UP");

  // ****************************** SBG Config ******************************
  // ToDo: improve configuration capabilities

  // Enable getting EKF quaternion
  errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT, SBG_ECOM_OUTPUT_MODE_DIV_8);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdOutputSetConf SBG_ECOM_LOG_EKF_QUAT Error");}

  errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_EULER, SBG_ECOM_OUTPUT_MODE_DIV_8);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdOutputSetConf SBG_ECOM_LOG_EKF_EULER Error");}

	// Enable getting EKF position and velocity in the NED coordinates
  errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV, SBG_ECOM_OUTPUT_MODE_DIV_8);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdOutputSetConf SBG_ECOM_LOG_EKF_NAV Error");}

	// Enable getting IMU acceleration and angular velocity values
  errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA, SBG_ECOM_OUTPUT_MODE_DIV_8);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdOutputSetConf SBG_ECOM_LOG_IMU_DATA Error");}

  // // Enable getting velocity
  // errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_SHIP_MOTION, SBG_ECOM_OUTPUT_MODE_DIV_8);
  // if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdOutputSetConf SBG_ECOM_LOG_SHIP_MOTION Error");}

	// Enable getting magnetometer data
  errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG, SBG_ECOM_OUTPUT_MODE_DIV_8); // 8-> 25Hz, 20->10Hz
  if (errorCode != SBG_NO_ERROR) {ROS_WARN("SBG_ECOM_LOG_MAG");}

  // Enable getting GPS position
	errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_POS, SBG_ECOM_OUTPUT_MODE_NEW_DATA);
	if (errorCode != SBG_NO_ERROR) {ROS_WARN("SBG_ECOM_LOG_GPS1_POS");}

	// Enable getting GPS velocity
	errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_VEL, SBG_ECOM_OUTPUT_MODE_NEW_DATA);
	if (errorCode != SBG_NO_ERROR) {ROS_WARN("SBG_ECOM_LOG_GPS1_VEL");}

	// Enable getting GPS heading for a dual antenna system
	errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_HDT, SBG_ECOM_OUTPUT_MODE_NEW_DATA);
	if (errorCode != SBG_NO_ERROR) {ROS_WARN("SBG_ECOM_LOG_GPS1_HDT");}

	// GNSS : Initial position
  SbgEComInitConditionConf sbgEComInitConditionConf;
  sbgEComInitConditionConf.latitude  = 48.19;
  sbgEComInitConditionConf.longitude = -3.0;
  sbgEComInitConditionConf.altitude  = 130.0;
  sbgEComInitConditionConf.year      = 2018;
  sbgEComInitConditionConf.month     = 2;
  sbgEComInitConditionConf.day       = 13;
  errorCode = sbgEComCmdSensorSetInitCondition(&comHandle, &sbgEComInitConditionConf);
  if (errorCode != SBG_NO_ERROR) {ROS_WARN("sbgEComInitConditionConf");}

  // GNSS : Port config
  SbgEComAidingAssignConf sbgEComAidingAssignConf;
  sbgEComAidingAssignConf.gps1Port         = SBG_ECOM_MODULE_PORT_C;
  sbgEComAidingAssignConf.gps1Sync         = SBG_ECOM_MODULE_SYNC_DISABLED;
  sbgEComAidingAssignConf.rtcmPort         = SBG_ECOM_MODULE_DISABLED;
  sbgEComAidingAssignConf.odometerPinsConf = SBG_ECOM_MODULE_ODO_DISABLED;
  errorCode = sbgEComCmdSensorSetAidingAssignment(&comHandle, &sbgEComAidingAssignConf);
  if (errorCode != SBG_NO_ERROR) {ROS_WARN("sbgEComAidingAssignConf");}

  // SAVE AND REBOOT
  errorCode = sbgEComCmdSettingsAction(&comHandle, SBG_ECOM_SAVE_SETTINGS);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdSettingsAction Error");}

  ROS_INFO("CONFIGURATION DONE");

  // ************************** SBG Callback for data ************************
  bool test = false;
  sbgEComSetReceiveLogCallback(&comHandle, onLogReceived, NULL);

  ROS_INFO("START RECEIVING DATA");

  imu_msg.header.frame_id = "map";
  nav_msg.header.frame_id = "map";
  attitude_msg.header.frame_id = "map";
  gps_msg.header.frame_id = "map";
  mag_msg.header.frame_id = "map";
  gps_altitude_msg.header.frame_id = "map";
  imu_altitude_msg.header.frame_id = "map";

  ros::Rate loop_rate(25);
  while (ros::ok())
  {
    int errorCode = sbgEComHandle(&comHandle);

    if(new_nav_msg){
      nav_msg.header.stamp = ros::Time::now();
      nav_pub.publish(nav_msg);  

      imu_altitude_msg.header.stamp = ros::Time::now();
      imu_altitude_pub.publish(imu_altitude_msg);
      new_nav_msg = false;
    }

    if(new_gps_msg){
    	gps_msg.header.stamp = ros::Time::now();
    	gps_pub.publish(gps_msg);

    	gps_altitude_msg.header.stamp = ros::Time::now();
        gps_altitude_pub.publish(gps_altitude_msg);

    	new_gps_msg = false;
    }

    if(new_imu_msg){
      imu_msg.header.stamp = ros::Time::now();
      imu_pub.publish(imu_msg);
      new_imu_msg = false;
    }

    if(new_attitude_msg){
    	attitude_msg.header.stamp = ros::Time::now();
    	attitude_pub.publish(attitude_msg);
    	new_attitude_msg = false;
    }

    if(new_mag_msg){
    	mag_msg.header.stamp = ros::Time::now();
    	mag_pub.publish(mag_msg);
    	new_mag_msg = false;
    }

    if(new_nav_vel){
    	nav_vel_msg.header.stamp = ros::Time::now();
		nav_vel_pub.publish(nav_vel_msg);
    	new_nav_vel = false;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}