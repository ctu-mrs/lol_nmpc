# Use the MRS UAV System the base image
FROM ctumrs/mrs_uav_system:latest

# Set workspace name as an argument with a default value
ARG WORKSPACE_NAME=lol_nmpc_ws
ENV WORKSPACE_NAME=${WORKSPACE_NAME}

# Install additional packages into the image
RUN apt-get -y update && apt-get install -y git curl python3-pip python-is-python3

RUN pip install gitman

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Create required directories
RUN mkdir -p /root/git

################################################################################
# Installing Acados

# Set Rust env so it's available to scripts
ENV PATH="/root/.cargo/bin:$PATH"

# Copy your script into the image
COPY install_acados.sh /root/install_acados.sh

# Make the script executable
RUN chmod +x /root/install_acados.sh

# Run the setup script
RUN /root/install_acados.sh

# Set environment variables for future shells
RUN echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/root/git/acados/lib' >> /root/.bashrc && \
    echo 'export ACADOS_SOURCE_DIR=$HOME/git/acados' >> /root/.bashrc

RUN echo source /opt/ros/noetic/setup.bash >> ~/.bashrc
ENV LD_LIBRARY_PATH=/root/git/acados/lib
ENV ACADOS_SOURCE_DIR=/root/git/acados

##################################################################################
# Install Python dependencies for px4_firmware

RUN python3 -m pip install --no-cache-dir \
    argcomplete \
    "argparse>=1.2" \
    cerberus \
    coverage \
    "empy>=3.3" \
    future \
    # "jinja2>=2.8" \
    jsonschema \
    kconfiglib \
    lxml \
    # "matplotlib>=3.0" \
    "numpy>=1.13" \
    nunavut>=1.1.0 \
    packaging \
    # "pandas>=0.21" \
    pkgconfig \
    psutil \
    # pygments \
    "wheel>=0.31.1" \
    # pymavlink \
    # pyros-genmsg \
    # pyserial \
    # "pyulog>=0.5.0" \
    # pyyaml \
    # requests \
    # "setuptools>=39.2.0" \
    # "six>=1.12.0" \
    "toml>=0.9"

RUN git config --global --add safe.directory '*'

#################################################################################
# Clone the repositories for the NMPC wrapper and the PX4 firmware
RUN mkdir -p /home/${WORKSPACE_NAME}/src
WORKDIR /home/${WORKSPACE_NAME}/src
RUN git clone https://github.com/ctu-mrs/lol_nmpc.git && \
    git clone https://github.com/ctu-mrs/px4_firmware.git && \
    cd px4_firmware && \
    git checkout integral_publishing

# WORKDIR /home/${WORKSPACE_NAME}/src/px4_firmware
# RUN git checkout integral_publishing

# Use Acados to build the NMPC wrapper 
WORKDIR /home/${WORKSPACE_NAME}/src/lol_nmpc/packages/controller_module/src/nmpc_acados
RUN yes | python3 main_build.py 

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - |  apt-key add -
# RUN apt-get install python3-catkin-tools -y

################################################################################

# Build the ROS workspace
WORKDIR /home/${WORKSPACE_NAME}/
RUN DEBIAN_FRONTEND=noninteractive rosdep install --from-paths /home/${WORKSPACE_NAME}/src --ignore-src -r -y
RUN catkin config -DCMAKE_BUILD_TYPE=Release

RUN . /opt/ros/noetic/setup.sh && catkin build

################################################################################
# Set environment variables for future shells
RUN echo "source /home/${WORKSPACE_NAME}/devel/setup.bash" >> /root/.bashrc

# Specify the default command to run
CMD ["/ros_entrypoint.sh"]
