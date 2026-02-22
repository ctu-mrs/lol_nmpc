#!/bin/bash
workspace_dir=$HOME/nmpc_workspace
sudo docker run -it \
  --name nmpc-container \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -v $workspace_dir:/root/nmpc_workspace \
  --privileged \
  lol_nmpc_docker:latest \
  bash
sudo docker rm nmpc-container
