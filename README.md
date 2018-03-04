# ros_helios_hardware
Hardware package for the catamaran HELIOS

![Status](https://img.shields.io/badge/Status-In%20Development-red.svg)
![ROS](https://img.shields.io/badge/ROS-Kinetic--Kame-green.svg)

### Installation
On part du principe que les packages de base de ROS sont installés.

##### Configuration du workspace
Dans le dossier où vous souhaitez créer votre workspace.
```
mkdir -p $nomdevotrechoix/src
cd $nomdevotrechoix/src
catkin_init_workspace
```

##### Installation des packages ROS

Installez les paquets suivants
```
sudo apt-get install ros-kinetic-nmea-navsat-driver ros-kinetic-hokuyo-node ros-kinetic-usb-cam ros-kinetic-rviz-imu-plugin ros-kinetic-laser-assembler
```

Lancez `install.sh` pour configurer les entrées du PC.

Pour l'installation du leddar, voir le README dans le dossier correspondant

##### Configuration des règles udev
Ces règles servent à identifier les appareils se connectant en USB. Elles sont situées dans le dossier `install_files`. Il faut le déplacer dans `/etc/udev/rules.d/`.

##### Outils de debug externes
Habituellement je les met dans un dossier `Logiciels`
```
cd && mkdir Logiciels && cd Logiciels
```

 - Pololu maestro control center :
```
wget https://www.pololu.com/file/download/maestro-linux-150116.tar.gz?file_id=0J315
mv maestro-linux-150116.tar.gz?file_id=0J315 maestro-linux-150116.tar.gz
tar -xvf maestro-linux-150116.tar.gz
```
Puis suivez les instructions du README.txt.

 - SBG sdk
ouvrez un navigateur et allez sur `https://files.sbg-systems.com/s/xg0mpvooQVqBgZR/authenticate`
Mettez les identifiants SBG et téléchargez le SDK. Pour calibrer la SBG, il faut passer sou Windows et télécharger le sbgControl center avec ces même identifiants.

### Interfaces
##### Entrées
##### Sorties

### Appareils
Le schéma d'architecture physique est disponible en ouvrant le fichier `cataArchiPhy.xml` avec la webapp draw.io

##### Polulu Maestro
Carte d'inteface capteur et actionneurs

__Nécessite :__ `usb_cam`

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

__Tests :__

 1. Lancer `roslaunch sbg_driver sbg_ellipse.launch` et `rviz`.
 2. Dans Rviz ajouter une imu et spécifier le topic `/imu`

##### Pololu Maestro pwm board
Carte pololu pour contrôler 2 moteurs T200 de BlueRobotics

 - Channel 0 : propulseur gauche
 - Channel 1 : propulseur droit

### Notes de developpement
ros_helios_config ne devrait pas être un repository séparé de ros_helios_hardware. Il faudrait l'enlever en tant que submodule et ajouter directement les fichiers. C'est tout simplement parce-que on utilisera pas la config d'Helios en dehors de ros_helios_hardware.
