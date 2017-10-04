# ros_helios_hardware
Hardware package for the catamaran HELIOS

### Installation

##### Configuration du workspace
```
cd src
catkin_init_workspace
```

##### Installation des packages ROS
```
sudo apt-get install nmea_navsat_driver hokuyo_node
```

### Sensors

##### Camera
Camera branché en USB

##### GPS RTK
GPS RTK branchée en USB. 

 __Nécessite :__ `nmea_navsat_driver`

##### Hokuyo

 __Nécessite :__ `hokuyo_node`

##### IMU (SBG)

 __Nécessite :__ `sbg_driver` version 1.0.7

### Actuators

##### Pololu Maestro pwm board
Carte pololu pour contrôler 2 moteurs T200 de BlueRobotics
