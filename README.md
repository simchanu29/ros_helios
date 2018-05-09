# ros_helios_hardware
Hardware metapackage for the catamaran HELIOS

![Status](https://img.shields.io/badge/Status-In%20Development-red.svg)
![ROS](https://img.shields.io/badge/ROS-Kinetic--Kame-green.svg)

### Installation
On part du principe que les packages de base de ROS sont installés.

1. Clonez ce stack dans un workspace ROS

2. Installez les paquets suivants nécessaires : 
```
sudo apt install ros-kinetic-nmea-navsat-driver ros-kinetic-usb-cam
```

3. Installez si vous le souhaitez ces paquets recommandés (pour le debug) : 
 ```
 sudo apt install ros-kinetic-hokuyo-node ros-kinetic-rviz-imu-plugin ros-kinetic-laser-assembler
 ```

4. Configuration des règles udev
Copiez les règles udev du dossier install_files vers l'endoita approrié sinon lancez `install.sh` pour configurer automatiquement. WARNING : ne fonctionne que sous Ubuntu 16+. Ce script déplace des règles udev, initialise le workspace

##### Outils de debug externes
Habituellement je les met dans un dossier `Logiciels` dans `/home/$USER`
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

### Utilisation

Tout les fichiers de configuration sont contenu dans le package `ros_helios_config`. Plusieurs modes sont possibles : 
 - `helios_simu.launch` pour lancer la simulation avec téléoperation

### Appareils supportés
Le schéma d'architecture physique est disponible en ouvrant le fichier `cataArchiPhy.xml` avec la webapp draw.io

 - Polulu Maestro
Carte d'inteface capteur et actionneurs

__Nécessite :__ `ros_maestro`

 - Attopilot
capteur courant et tension.

__Nécessite :__ `ros_maestro`

 - Camera
Camera branché en USB

__Nécessite :__ `usb_cam`

 - GPS
GPS branché en USB.

__Nécessite :__ `nmea_navsat_driver`

 - IMU (SBG)

__Nécessite :__ `sbg_driver` version 1.0.7

__Tests :__

 1. Lancer `roslaunch sbg_driver sbg_ellipse.launch` et `rviz`.
 2. Dans Rviz ajouter une imu et spécifier le topic `/imu`

### Troubleshooting
