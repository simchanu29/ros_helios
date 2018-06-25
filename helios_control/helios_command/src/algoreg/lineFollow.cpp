//
// Created by simon on 19/06/18.
//

#include "lineFollow.h"

LineFollow::LineFollow(){

};

double LineFollow::run(std::vector<double> wp_a, std::vector<double> wp_b, std::vector<double> state_vec){

    double ax = wp_a[0];
    printf("ax = [%f]\n",ax);
    double ay = wp_a[1];
    printf("ay = [%f]\n",ay);
    double bx = wp_b[0];
    printf("bx = [%f]\n",bx);
    double by = wp_b[1];
    printf("by = [%f]\n",by);
    const double x = state_vec[0];
    printf("x = [%f]\n",x);
    const double y = state_vec[1];
    printf("y = [%f]\n",y);

    double headLine = atan2(by-ay,bx-ax); // ENU convention, angle from east
    printf("headLine = [%f]\n",headLine);

    // Si le bateau dépasse la ligne
    const double angleNextWp2Boat = atan2(y-by,x-bx); // ENU convention, angle from east
    printf("angleNextWp2Boat = [%f]\n",angleNextWp2Boat);
    printf("angle_rad(angleNextWp2Boat,-headLine) = [%f]\n",angle_rad(angleNextWp2Boat,-headLine));
    if(fabs(angle_rad(angleNextWp2Boat,-headLine))<M_PI/2.0){
        // If boat has crossed the line
        printf("===================== Inverting WayPoints\n");

        const double tmpx = ax;
        const double tmpy = ay;
        ax = bx;
        ay = by;
        bx = tmpx;
        by = tmpy;

        headLine = atan2(by-ay,bx-ax); // ENU convention, angle from east
    }

    // Calcul de la distance à la ligne, si c'est un point alors une valeur est définie
    double dist2Line;
    if( sqrt( pow(bx-ax,2) + pow(by-ay,2)) != 0){
        dist2Line = ((bx-ax)*(y-ay) - (by-ay)*(x-ax)) / sqrt( pow(bx-ax,2) + pow(by-ay,2));
    } else{ dist2Line = 100; }
    printf("dist2Line = [%f]\n",dist2Line);

    // Calcul de l'erreur
    const double distAtanPt = 3;
    const double error = atan(dist2Line / distAtanPt);
    printf("headLine = [%f]\n", headLine);
    printf("error = [%f]\n", error);
    printf("headLine - error = [%f]\n", headLine - error);

    // Bornage entre -pi et pi
    const double wantedHead = angle_rad(headLine,- error);
    printf("wantedHead = [%f]\n", wantedHead); // Le cap voulu par rapport au repere global

    if(debug){
        debugMap["wantedHead"] = wantedHead;
        debugMap["headLine"] = headLine;
        debugMap["angleNextWp2Boat"] = angleNextWp2Boat;
        debugMap["error"] = error;
        debugMap["dist2Line"] = dist2Line;
        debugMap["ax"] = ax;
        debugMap["ay"] = ay;
        debugMap["bx"] = bx;
        debugMap["by"] = by;
    }

    return wantedHead;
}