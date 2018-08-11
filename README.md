# learning_moveit

## 1. Introduction

This ros metapackage contains some packages that I wrote it myself according to the tutorials provided by the MoveIt! official website. The MoveIt! Tutorials website is: http://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html.


### 1.1 Basic Components

- **panda_bringup**: The panda_bringup package provides roslaunch for starting panda_arm_hand motion planning and simulation test.
- **panda_description**: The panda_description package provides a complete 3D model of the panda_arm_hand for simulation and visualization.

### 1.2 Motion Simulation

- **panda_gazebo**: The panda_gazebo package can launch panda_arm_hand in gazebo simulator.
- **panda_gazebo_controller_config**: The panda_gazebo_controller_config package provides controller configuration files for simulating panda_arm_hand in gazebo.

### 1.3 Motion Control

- **panda_teleop**: The panda_teleop package can control panda_arm_hand by using keyboard.

### 1.4 Motion Planning

- **panda_moveit_config**: The panda_moveit_config package provides configuration and launch files for using the panda_arm_hand with the MoveIt! Motion Planning Framework.
- **panda_moveit_control**: The panda_moveit_control package provides motion planning and control for panda_arm_hand by using kinematics.

## 2. Installation

```sh
$> roscd
$> cd ../src
$> git clone https://github.com/myyerrol/learning_moveit.git
$> cd ..
$> rosdep install -y --from-paths src --ignore-src --rosdistro kinetic
$> catkin_make
```

## 3. Usage
