//
// Created by tag on 28/02/17.
//
#pragma once

#include <cmath>
#include <proj_api.h>

#define CONVERSION_FACTOR_GPS 1852.0 // in meters/min
#define RAD_2_DEG 1.0/M_PI*180.0
#define DEG_2_RAD M_PI/180.0

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


struct Coordinates{
    double x;
    double y;
};

Coordinates latlon2meters(double lat, double lon){

    Coordinates coord;
    projPJ pj_lambert, pj_latlong, pj_utm30N;

    if (!(pj_lambert = pj_init_plus("+proj=lcc +lat_1=49 +lat_2=44 +lat_0=46.5 +lon_0=3 +x_0=700000 +y_0=6600000 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs ")))
    {
        printf("error lambert \n");
        exit(1);
    }

    if (!(pj_utm30N = pj_init_plus("+proj=utm +zone=30 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")))
    {
        printf("error utm30N \n");
        exit(1);
    }

    if (!(pj_latlong = pj_init_plus("+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs")))
    {
        printf("error latlong \n");
        exit(1);
    }

    lat *= DEG_TO_RAD;
    lon *= DEG_TO_RAD;

    pj_transform(pj_latlong, pj_utm30N, 1, 1, &lat, &lon, nullptr);

    //printf("X: %lf \nY: %lf\n", x, y);
    coord.x = lat;
    coord.y = lon;
    return coord;
}