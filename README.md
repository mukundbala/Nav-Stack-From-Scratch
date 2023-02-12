# Robot-Control-Planning-Navigation
[![ROS Build](https://github.com/mukundbala/Robot-Control-Planning-Navigation/actions/workflows/main.yml/badge.svg?branch=main)](https://github.com/mukundbala/Robot-Control-Planning-Navigation/actions/workflows/main.yml)

This repository contains from scratch implementation of planning and control techniques for differential drive robots (part 1) and quadrotors (part 2)
Tested on Ubuntu 20.04 for ROS Noetic.
(Note for teammates: Please copy hector into your source directory. When you push, hector will not be pushed to git)
## How to build

```bash
##but first, some dependencies

sudo apt install python3-catkin-tools

#ensure that you have ros-noetic installed: http://wiki.ros.org/noetic/Installation/Ubuntu
```
Install dependencies
```bash
cd ~
mkdir -p catkin_ws/src ##note that you dont have to use "catkin_ws"
cd ~catkin_ws/src
git clone git@github.com:mukundbala/Robot-Control-Planning-Navigation.git
```
Build workspace using catkin build
```bash
##build. Should take a couple of seconds to 2min
catkin build
```
Always remember to source the workspace before running anything
```bash
source ~/catkin_ws/devel/setup.bash
roscd ##this will bring you to the devel folder in your workspace
```

## Part 1: Differential Drive Robot

- [X] Motion Filter using odometry and joint states
- [ ] Logs Odd Bayes Filter for Occupancy Grids
- [ ] Mission Planner
- [ ] Global Planner base class
- [ ] Global Planner: Theta*
- [ ] Global Planner: Djikstra
- [ ] Global Planner: BFS
- [ ] Commander: Cubic Spline
- [ ] Commander: PID Controller with Bidirectional Motion
- [ ] Full code refactor
- [ ] Improve param loading sequence using expeditious use of launch files
- [ ] Do away with all .sh files and use launch files instead
- [X] Time profiler for measuring loop execution speed. Used to optimize ros rates 
## Part 2: Quadrotor


