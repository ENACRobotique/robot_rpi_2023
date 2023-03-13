# robot_rpi_2023

## To use/Install
Install eCAL & Python eCAL bindings from [here](https://eclipse-ecal.github.io/ecal/getting_started/setup.html).
Install requirements.txt (if virtual env) or environnment.yml (if anaconda).
Last tested protolib version : 3.11.3
Last tested eCAL version : 5.10.1
Python Version : 3.10.9
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
│   │   launch_ecal.py - Takes lidar_data -> Publish cone_detection (stop_cons), lidar position
│   │   launch_tests.py - unit test for loca_lidar part
│   │   launch_vizualisation.py - Matplotlib Lidar, fixed_beacon positions, obstacle_cone visualization
│   │   
proto - protobuf definition folder & compiled for python
```