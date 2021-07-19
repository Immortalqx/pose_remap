# Pose_Remap
This is a ros package used to transform a pose into the one robot needs.

## Prerequisites

- ubuntu 18.04
- ROS melodic
- Mavros
- Gflags

## Build Pose_Remap

Clone the repository and catkin_make:

```bash
    cd ~/catkin_ws/src
    git clone git@github.com:Immortalqx/pose_remap.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## Run Pose_Remap

```bash
roslaunch pose_remap remap_default.launch
```

## Dynamic Reconfig

```bash
rosrun rqt_reconfigure rqt_reconfigure
```
