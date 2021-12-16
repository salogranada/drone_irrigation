# drone_irrigation

### Prerequisites

1. The simulation requires ROS (Robot Operating System) and was mainly developed and tested with:

  * ROS Melodic Morenia on Ubuntu 18.04

Other distributions might not work properly so it is recommended to use one of this setup.

2. CoppeliaSim V4.2.0 rev5 for Ubuntu 18.04.

  This software is free and you can download it from: https://coppeliarobotics.com/downloads

#### Building


1. Clone the repository (recommended location is ~) downloaded file is a ROS package.

  ```bash
  cd ~
  ```

  ```bash
  git clone https://github.com/salogranada/drone_irrigation.git
  ```

2. Move repository to the source of the workspace (for this example tesis_ws).

  ```bash
  cd ~/tesis_ws/src/
  ```
3. Move to the root of the workspace.

  ```bash
  cd ~/tesis_ws/
  ```

4. Build the workspace.

  ```bash
  catkin build
  source devel/setup.bash
  ```
### Launching

  Although we will be running a .launch file (which starts roscore behind) we need to run this roscore so that Coppeliasim can start its ROS plugin.
  
1. Run roscore.

  ```bash
  roscore
  ```
2. On another terminal window, open the simulator Coppeliasim. Change current directory to your downloaded folder and run.

  ```bash
  cd Downloads/CoppeliaSim_Edu_V4_2_0_Ubuntu18_04
  ./coppeliaSim.sh
  ```
  
Open tracking_controller.ttt scene (this file is inside this repository, in Scene folder).  
  
3. Run the drone controller, on another terminal run (Make sure you source the devel space on each terminal session you want to use the simulation on):

  ```bash
  cd ~/tesis_ws/
  source devel/setup.bash
  roslaunch drones_pkg simulation.launch
  ```
  
A black box should show up in the simulation and the drone should go towards this waypoint. The drone would complete a path with various points defined in a txt file. When there is no more waypoints in the path you can stop the simulation and report files will be generated with flight data and error log.

## ROS API

#### Subscribed Topics
* **`/simulationTime`** ([Float32])
* **`/realTime`** ([Float32])
* **`/currentMass`** ([Float32])
* **`/drone_pose`** ([Float32MultiArray])
* **`/drone_orientation`** ([Float32MultiArray])
* **`/target_pose`** ([Float32MultiArray])

#### Published Topics
* **`/drone_nextPose`** ([Float32MultiArray])
* **`/drone_nextEuler`** ([Float32MultiArray])
* **`/PE/Drone/tank_volume`** ([String])
* **`/PE/Drone/drone_status`** ([Float32MultiArray])
* **`PE/Drone/controller_time`** ([Float32MultiArray])
* **`/PE/Drone/restart`** ([Bool])
