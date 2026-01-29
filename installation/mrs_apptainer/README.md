Installation instructions for NMPC in Apptainer:
================================================

The instructions below expect that the repositories are cloned to folder **~/git**. If you need to choose another location, the commands below need to be changed accordingly. 

1. install apptainer on your system
  - https://github.com/ctu-mrs/mrs_apptainer
2. install gitman - which we use for managing git submodules
  - ubuntu 24.04 
    ```
    sudo apt install pipx
    pipx install gitman
    ```


How to install NMPC inside apptainer:
1. create_overlay.sh 
    go to mrs_apptainer/scripts/create_overlay.sh
    - change the OVERLAY_SIZE=5000 # MB (this should be enough)
    - do not forgot to set OVERLAY=true in wrapper.sh
2. do git clone, git checkout and gitman update outside of the wrapper (even acados should be updated outside of the wrapper) (not sure if this is needed if you source .ssh and install gitman inside the wrapper) this is done by the following script:
  ```
  01_clone_outside_apptainer.sh
  ```
3. mount your folders to be visible in apptainer
- you need to modify the mrs_apptainer/wrapper.sh script by including following lines:
  ```
  "type=bind" "/home/<your_user_name>/.ssh" "/home/$USER/.ssh"
  "type=bind" "/home/<your_user_name>/bag_files" "/home/$USER/bag_files"
  "type=bind" "/home/<your_user_name>/git" "/home/$USER/git"
  "type=bind" "/home/<your_user_name>/nmpc_workspace" "/home/$USER/nmpc_workspace"
  ```
  plese note that <your_user_name> is needed for some installation in sudo mode 
4. build acados inside the wrapper
  - go to apptainer and run the following commands:
  ```
  02_install_without_sudo.sh
  ```
5. install acados python template used for the nmpc:
  - run apptainer with sudo (sudo ./wrapper) and run the following command:
  ``` 
  03_install_with_sudo.sh
  ```
6. setup bashrc in apptainer by running the following command:
  - run apptainer without sudo (./wrapper) and run the following command:
  ```
  04_setup_apptainer.sh
  ```
  - you need to include sourcing of the new workspace in .bashrc, this can be done when the apptainer is running or eventually you can modify mrs_apptainer/mount/ apptainer_bashrc.sh directly
7. build the nmpc_workspace
  - run apptainer without sudo (./wrapper) and build the workspace:
    ```
    cd /home/$USER/nmpc_workspace
    catkin build
    ```
8. source the workspace in apptainer
  - you can do this by adding the following line to your .bashrc inside apptainer:
    ```
    source ~/nmpc_workspace/devel/setup.bash
    ```
  - you can edit this file in the apptainer or you can modify mrs_apptainer/mount/apptainer_bashrc.sh directly


Please note that these guidlines were made after some time and therefore it is possible that I missed something or something was done little bit differently. 
Please do not hesitate to ask if you have any questions or if something is not clear. 
If you will find that something is not working please let me know and I will try to help you. <prochon4@fel.cvut.cz>

Troubleshooting
---------------
If you encounter issues during the installation or setup process, consider the following steps:

- acados_template can not be found == installation of acados_template was not successful
  * make sure you have run the 03_install_with_sudo.sh script inside the apptainer with sudo
  * if the problem persists try to run the following command inside the apptainer with sudo:
    ```
    cd ~/git/acados/interfaces/acados_template
    pip install -e .
    ```

- Your catkin build is stuck == build of controller_module runs more then 10 minutes
  * go to the controller_module/src/nmpc-for-quadrocopter directory and run the following command:
    ```
    python main_build.py
    ```
  * confirm installation of tera_renderer 
