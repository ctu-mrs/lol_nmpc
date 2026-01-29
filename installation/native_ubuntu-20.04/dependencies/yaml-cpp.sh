#!/bin/bash

set -e

Yellow='\033[0;33m'
LBlue='\e[1;34m'
NC='\033[0m' # No Color

echo "$0: Installing Yaml-cpp"

cd ~/git
if [ ! -d yaml-cpp ]; then
  git clone https://github.com/jbeder/yaml-cpp.git
fi

# build the project
echo -e "${Yellow}Compile yaml-cpp${NC}"
cd ~/git/yaml-cpp

if [ ! -d build ]; then
  mkdir build
fi

cd build
cmake ..
make 
sudo make install

pip install pyyaml
