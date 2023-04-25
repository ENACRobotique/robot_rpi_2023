# robot_rpi_2023

## To Install
Install eCAL & Python eCAL bindings from [here](https://eclipse-ecal.github.io/ecal/getting_started/setup.html).
Install requirements.txt (if virtual env) or environnment.yml (if anaconda).
Last tested protolib version : 3.11.3
Last tested eCAL version : 5.10.1
Python Version : 3.10.9

## To use  : 
* Lancer le driver de lidar : ld06_driver/ecal_ld06_driver.py
    * Réglage du port serial dans ld06_driver.py

* Traitement du lidar (localisation + évitement): loca_lidar/launch_loca_ecal.py

* Pour sauvegarder des logs & les afficher tous sur la même console : common/ecal_debug_printer.py

* Pour enregistrer automatiquement 110s de données après match_start : common/launch_autorecorder.py

## Pour info sur les topics
* Nom des topics à utiliser : 
    * Position du lidar "brute" : lidar_pos
    * Consigne d'arret détection obstacle en mouvement : stop_cons
    * Position moyennée lorsque le lidar est fixe : smooth_pos

# common
# ld06_driver

Localize the lidar/robot position on its environement with knowing fixed points positions for eurobot.
Takes eCAL topic driver as input and output positions on eCAL too.
### launch_tests
using pytest, unit tests for all functionalities (amalgame discovery, pattern finder, ...)

### launch_vizualisation
using eCAL + matplotlib, plot data (more using debug mode)

### config.py
allow to set parameters (debug mode, ...)

## Contribution
Check regularly using launch_tests.py if nothing is broken
### Details 
Lidar Frame -> Polar coordinates, origin is the lidar, distance in meters, angle in 360°
Table Frame -> Cartesian Coordinates, origin is bottom left, distance in float meters, angles are radians


## Relevant files
```
src
│   README.md
│   file001.txt    
│
└───ld06_driver
│   │   ld06_driver.py - Connect to Serial to get lidar data
│   │   ecal_ld06_driver.py - Stream lidar on 'lidar_data' eCAL topic
│   │   ecal_fake_lidar.py - fixed fake lidar data to test 'lidar_data' streaming in eCAL 
│   │   
└─── loca_lidar
│   │   launch_loca_ecal.py - Takes lidar_data -> Publish cone_detection (stop_cons), lidar position
│   │   launch_tests.py - unit test for loca_lidar part
│   │   launch_vizualisation.py - Matplotlib Lidar, fixed_beacon positions, obstacle_cone visualization
└─── position_fusion

proto - protobuf definition folder & compiled for python
```