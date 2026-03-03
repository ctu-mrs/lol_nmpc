# LoL-NMPC in Docker Environment

This folder contains the Docker configuration and scripts to set up a fully containerized environment for the LoL-NMPC workspace. 

The environment includes all necessary dependencies, including Acados, which is automatically installed during the image build process.

## File Overview

* **`Dockerfile`**: Defines the system environment, installs ROS/system dependencies, and configures the workspace.
* **`install_acados.sh`**: A helper script executed internally by the `Dockerfile` to install and configure Acados and its dependencies.
* **`build_docker.sh`**: Script to build the Docker image.
* **`start_docker.sh`**: Script to run the container and mount your local workspace into the Docker environment.

---

## Quick Start Guide

### 1. Build the Docker Image
First, build the Docker image. This will download the base image, install all dependencies, and run the Acados installation script.

```bash
./build_docker.sh
``` 

### 2. Start the Container
Once the image is built, start the interactive Docker container:

```bash 
./start_docker.sh
```

### 3. Run the simulation
Inside the container, you need to navigate to the packgage with the simulation launch file and run it:

```bash 
roscd a300_gazebo_resources/tmux/gazebo_sim/
./start.sh
```


