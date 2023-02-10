# Robot-Control-Planning-Navigation

This repository contains from scratch implementation of planning and control techniques for differential drive robots (part 1) and quadrotors (part 2)

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

- [ ] Motion Filter using odometry and joint states
- [ ] Logs Odd Bayes Filter for Occupancy Grids
- [ ] Mission Planner
- [ ] Global Planner: Theta*
- [ ] Global Planner: Djikstra
- [ ] Global Planner: BFS
- [ ] Commander: Cubic Spline
- [ ] Commander: PID Controller with Bidirectional Motion
- [ ] Full code refactor
- [ ] Improve param loading sequence using expeditious use of launch files

## Part 2: Quadrotor


