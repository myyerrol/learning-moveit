#!/usr/bin/env bash

set -e

PATH_HOME=${HOME}

installROSPackages()
{
    sudo apt-get update
    sudo apt-get install ros-kinetic-effort-controllers \
                         ros-kinetic-robot-state-publisher \
                         ros-kinetic-joint-state-controller \
                         ros-kinetic-joint-state-publisher \
                         ros-kinetic-joint-trajectory-action \
                         ros-kinetic-joint-trajectory-controller \
                         ros-kinetic-gazebo-plugins \
                         ros-kinetic-gazebo-ros \
                         ros-kinetic-gazebo-ros-control \
                         ros-kinetic-moveit-ros-perception \
                         ros-kinetic-moveit-simple-controller-manager \
                         ros-kinetic-moveit-visual-tools \
                         ros-kinetic-position-controllers \
                         ros-kinetic-velocity-controllers
}

setGazeboModels()
{
    if [ ! -d ${PATH_HOME}/.gazebo/models ]; then
        mkdir -p ${PATH_HOME}/.gazebo/models
        echo "Creating the directory of Gazebo models successfully!"
    else
        echo "The directory of Gazebo models has been created successfully!"
    fi

    cp -pruv panda_gazebo/objects/object_cube ${PATH_HOME}/.gazebo/models
    cp -pruv panda_gazebo/objects/object_cuboid ${PATH_HOME}/.gazebo/models
    cp -pruv panda_gazebo/objects/object_cylinder ${PATH_HOME}/.gazebo/models

    echo "Setting Gazebo models successfully!"
}

main()
{
    installROSPackages
    setGazeboModels
}

RUNNING=$(basename $0)
[ "$RUNNING" = "setup.sh" ] && main
