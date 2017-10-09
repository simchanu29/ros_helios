//
// Created by tag on 28/02/17.
//

#include <cmath>

#define CONVERSION_FACTOR_GPS 1852.0 // in meters/min

static double angle_deg(double a1,double a2){
    return fmod( a1 + a2 + 3*180, 360 ) - 180;
}

static double angle_rad(double a1,double a2){
    return (fmod( a1 + a2 + 3*M_PI, 2*M_PI ) - M_PI);
}

//static double enu2ned_yaw_rad(){
//
//}

static double ned2enu_yaw_rad(double yaw){
    return angle_rad(M_PI/2,- yaw);
}

static double ned2enu_yaw_deg(double yaw){
    return angle_deg(90,- yaw);
}

/**
 * N'est plus valable pour de sdistances supérieures à 500m
 * @param latPos
 * @param latOrigin
 * @return
 */
static double latDeg2meters(double latPos,double latOrigin){
    return (latPos-latOrigin)*CONVERSION_FACTOR_GPS*60.0;
}

/**
 * N'est plus valable pour de sdistances supérieures à 500m
 * @param longPos
 * @param latOrigin
 * @param longOrigin
 * @return
 */
static double longDeg2meters(double longPos,double latOrigin, double longOrigin){
    return (longPos-longOrigin)*CONVERSION_FACTOR_GPS*60.0*cos(latOrigin/180*M_PI);
}

static double distance(double x1, double y1, double x2, double y2){
    return sqrt( pow(x2-x1,2) + pow(y2-y1,2) );
}