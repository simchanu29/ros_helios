//
// Created by simon on 05/10/17.
//
#include "maestro.h"

int main(){
    int fd = maestroConnect("/dev/ttyACM0");
    maestroSetTarget(fd, 0, 5000);
    sleep(2);
    maestroSetTarget(fd, 0, 7000);
}
