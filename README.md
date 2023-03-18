# Robot-Control-Planning-Navigation
[![ROS Build](https://github.com/mukundbala/Robot-Control-Planning-Navigation/actions/workflows/main.yml/badge.svg?branch=main)](https://github.com/mukundbala/Robot-Control-Planning-Navigation/actions/workflows/main.yml)

This repository contains from scratch implementation of planning and control techniques for differential drive robots (part 1) and quadrotors (part 2)
Tested on Ubuntu 20.04 for ROS Noetic and compiled with C++17.

## How to build

```bash
##but first, some dependencies

sudo apt install python3-catkin-tools
sudo apt install ros-noetic-plotjuggler-ros. 

#ensure that you have ros-noetic installed: http://wiki.ros.org/noetic/Installation/Ubuntu
```
Clone workspace
```bash
cd ~
mkdir -p catkin_ws/src ##note that you dont have to use "catkin_ws"
cd ~catkin_ws/src
git clone git@github.com:mukundbala/Robot-Control-Planning-Navigation.git
```
Build workspace using catkin build
```bash
catkin build
```
Always remember to source the workspace before running anything
```bash
source ~/catkin_ws/devel/setup.bash
```

## Part 1: Differential Drive Robot

### Objectives:

- [X] Motion Filter using odometry and joint states
- [X] Logs Odd Bayes Filter for Occupancy Grids
- [X] Mission Planner
- [X] Global Planner: Any Angle A*
- [ ] Global Planner: Theta*
- [X] Global Planner: Djikstra
- [X] Commander: Cubic Spline
- [X] Commander: Quintic Spline
- [X] Commander: PID Controller with Bidirectional Motion
- [X] Full code refactor
- [X] Improve param loading sequence. Specifically, load goals using XmlRpcTypeArray instead of parsing string
- [X] Time profiler for measuring loop execution speed. Used to optimize ros rates 

### Package Description:
#### **(1) turtle_bringup**
Bringup package contains all the necessary TurtleBot3 burger description files, world files, rviz config files and a tf broadcaster.

#### **(2) loco_mapping**
A localization-mapping package. Localization is achieved by fusing IMU data and wheel encoder data using a weighted average filter. Weights for wheel encoder and IMU can be found in motion_filter.yaml. Mapping is done using a logs-odd bayes filter. A map with inflation zones can be seen on Rviz

#### **(3) mission_planner**
The mission_planner package constantly check the distance between the robot and the current goal. It serves the global planner with new goals once the robot has reached within a goal_radius to the goal. It also accepts goal updates from the global planner in case the goal provided lies on an inaccessible area. For a particular WORLD, goals can be set in turtle_bringup/worlds/WORLD/WORLD_goals.yaml

#### **(4) global_planner**
The global_planner package uses Any-Angle A* as its main planning algorithm to plan a path from the robot to the goal. It is also equipped with a Djikstra backup planner to find the nearest free cell if the robot or goal is on a non-free cell.

#### **(5) commander**
The commander package comprises of a local_planner (trajectory generator) and pid controller. Trajectory planner can utilise Linear , Cubic or Quintic Hermite trajectories to generate targets. This can be set in trajectory.yaml. PID gains can be set in pid.yaml.

#### **(6) tmsgs**
The tmsgs package contains purely message and service files for communication between nodes.

**================================**
## Part 2: Quadrotor

### Instructions (Please Read)

- With the Quadrotor, it is possible to run solo flights AND co-op missions with the turtlebot
- To set Solo Flight MODE:
    - Go to params.sh and set TASK to "soloflight" and WORLD to world0
    - Go to turtle_bringup package/worlds/world0 and set the waypoints you want. Node that each waypoint is a TUPLE of [x,y,z]
    - Got to drone_commander/config/drone_commander and set co-op param to false
    - Good to go!
- To set Co-Op MODE:
    - Go to params.sh and set TASK to "proj2" and WORLD to world20/world21
    - Got to drone_commander/config/drone_commander and set co-op param to true
    - Good to go!