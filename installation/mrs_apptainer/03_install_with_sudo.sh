#!/bin/bash 

# ## | ------------------- install under sudo ------------------- |
#
# Install acados and its dependencies
sudo pip install casadi==3.6.3
sudo pip install --use-pep517 ~/git/acados/interfaces/acados_template

# Install useful packages
sudo apt install ros-$ROS_DISTRO-plotjuggler-ros

