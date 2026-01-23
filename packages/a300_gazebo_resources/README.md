a300 gazebo resources
===========================

This package includes all necessary requisities for our small fast and agile drone, which was build on the a300 frame.
This package is compatible with the new MRS system.

# Instalation

There is no need to install anything. 
You can just build this package.

# How to use

If you are just normal user build this package and go into the `tmux` (template) folder to run ```start.sh``` script.
It will run tmux session with correct drone.

# Package structure

* `config` directory contains configuration of the UAV and its physical properties.
* `models` directory includes UAV's model and definition for the gazebo simulation.
* `tmux` directory includes predefined start script for running a300 drone in simulation.
* `worlds` directory includes definition of the worlds for the gazebo simulation.
* `CMakeLists.txt` is cmake file that defines build properties.
* `package.xml` defines properties of the package, such as package name and dependencies. 
* `README.md` is this file that you are currently reading and the file contains description of this package.

# NOTES what was necessary to modify to make it run
* modified mrs_simulation/config/spawner_params.yaml 
* modified mrs_simulation/scripts/mrs_drone_spawner.py
* added file to mrs_simulation/ROMFS/px4fmu_common/init.d/airframes/4001_a300
