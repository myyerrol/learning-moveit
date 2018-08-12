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

- Installing ROS packages.
```sh
$> roscd
$> cd ../src
$> git clone https://github.com/myyerrol/learning_moveit.git
$> cd ..
$> rosdep install -y --from-paths src --ignore-src --rosdistro kinetic
$> cd src/learning_moveit
$> ./setup.sh
```

- Building ROS packages.
```sh
$> roscd
$> cd ..
$> catkin_make
```

## 3. Usage

### 3.1 Motion control

```sh
$> roslaunch panda_bringup panda_bringup_gazebo.launch
$> rosrun panda_teleop panda_teleop_arm_hand_keyboard
```
![panda_control_rviz](.images/panda_control_rviz.png)
![panda_control_gazebo](.images/panda_control_gazebo.png)


### 3.2 Motion Planning

```sh
$> roslaunch panda_moveit_config demo.launch
```
![panda_planning_demo](.images/panda_planning_demo.png)

---

```sh
$> roslaunch panda_bringup panda_bringup_gazebo_and_moveit.launch
```

![panda_planning_rviz](.images/panda_planning_rviz.png)
![panda_planning_gazebo](.images/panda_planning_gazebo.png)

## 4. Note

- **Because **.

