# loca lidar

## Introduction

Localize the lidar/robot position on its environement with knowing fixed points positions for eurobot.
Takes eCAL topic driver as input and output positions on eCAL too.

### launch_ecal
file to launch collision warning + localization using eCAL for communication  
### launch_tests
using pytest, unit tests for all functionalities (amalgame discovery, pattern finder, ...)

### launch_vizualisation
using eCAL + matplotlib, plot data (more using debug mode)

### config.py
allow to set parameters (debug mode, ...)

## Contribution
Check regularly using launch_tests.py if nothing is broken
## Details 
Lidar Frame -> Polar coordinates, origin is the lidar, distance in meters, angle in 360°
Table Frame -> Cartesian Coordinates, origin is bottom left, distance in float meters, angles are radians

## TODO 
check if numpy arrays are useful or not, and type hinting in PointsDatStruct
