#!/bin/bash

# Generating Large-Scale Trajectories Efficiently using Double Descriptions of Polynomials

set -e

Yellow='\033[0;33m'
LBlue='\e[1;34m'
NC='\033[0m' # No Color

echo "$0: Installing trajectory generator"

cd ~/git
if [ ! -d nmpc_workplace ]; then
  git clone --recursive git@mrs.felk.cvut.cz:hrebeja1/nmpc_workplace.git
fi
cd nmpc_workplace
git fetch
git pull
git submodule update


# build the project
echo -e "${Yellow}Compile trajectory generator${NC}"
cd ~/git/nmpc_workplace

if [ ! -d build_objs ]; then
  mkdir build_objs
fi

if [ ! -d build_objs_local ]; then
  mkdir build_objs_local
fi

cd nmpc-for-quadcopters
python main_build.py
cd ..
make generator
