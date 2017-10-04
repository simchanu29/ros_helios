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
sudo apt-get install ros-kinetic-nmea-navsat-driver ros-kinetic-hokuyo-node ros-kinetic-usb-cam
```

### Interfaces
##### Entrées
##### Sorties

### Appareils
Le schéma d'architecture physique est disponible en ouvrant le fichier `cataArchiPhy.xml` avec la webapp draw.io

##### Camera
Camera branché en USB

__Nécessite :__ `usb_cam`

##### GPS RTK
GPS RTK branchée en USB. 

__Nécessite :__ `nmea_navsat_driver`

##### Hokuyo

__Nécessite :__ `hokuyo_node`

##### IMU (SBG)

__Nécessite :__ `sbg_driver` version 1.0.7

##### Pololu Maestro pwm board
Carte pololu pour contrôler 2 moteurs T200 de BlueRobotics

 - Channel 0 : propulseur gauche
 - Channel 1 : propulseur droit 
