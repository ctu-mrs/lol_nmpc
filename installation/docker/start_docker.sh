#!/bin/bash
sudo docker run -it \
  --name lol-nmpc-container \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --privileged \
  lol_nmpc_docker:latest \
  bash
sudo docker rm lol-nmpc-container
