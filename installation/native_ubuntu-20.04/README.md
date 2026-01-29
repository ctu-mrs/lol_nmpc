Installation instructions (ubuntu 20.04 native)
==============================================

To install the whole project (all necessary packages are going to be cloned automagically using gitman), clone this repository somewhere outside of your ROS workspace. 
It is recommended to put in in ~/git folder. (It is possible that some of the path are hardcoded in the install script).

1. install gitman 
  ```bash
  pip install gitman
  ```
2. clone fast_nmpc repository
  ```bash
  cd ~/git
  git clone git@mrs.fel.cvut.cz:agile-flight/fast_nmpc.git
  cd fast_nmpc
  gitman install --force
  ```
3. install dependencies
  - run install script from this folder
    ```bash 
    ./install.sh
    ```
4. Create workspace
  ```bash
  mkdir -p ~/nmpc_workspace/src && cd ~/nmpc_workspace
  ```
  - configure workspace
  ```bash 
  catkin init
  catkin config --profile reldeb --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON 
  catkin config --extend /opt/ros/$ROS_DISTRO
  catkin profile set reldeb
  ```
5. Link fast_nmpc package to workspace
  ```bash
  cd ~/nmpc_workspace/src
  ln -s ~/git/fast_nmpc 
  ```
6. Build workspace
  ```bash
  cd ~/nmpc_workspace
  catkin build -c
  ```
 - you need to correct sourcing of the new workspace in .bashrc
 - add to the end of the file `~/.bashrc` the line
    ```bash
    source ~/nmpc_workspace/devel/setup.bash
    ```

# Known issues
The install script should build the entire project (even the nmpc-for-quadcopter python build is run from the cmake).
PS: `when you compile it for the first time the python build requires terarender`. 
If the build runs to long (more than 5 min) run the 
`python main_build.py`
in fast_nmpc/packages/controller_module/src/nmpc-for-quadcopter manually.

