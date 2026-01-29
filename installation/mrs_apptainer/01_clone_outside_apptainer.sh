#!/bin/bash

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

arch=`uname -i`

# define custom colors
Yellow='\033[0;33m'
LBlue='\e[1;34m'
NC='\033[0m' # No Color

## | ---------------------- clone acados ---------------------- |

echo "$0: Clone acados"

# create git folder if it does not exist already
if [ ! -d ~/git ]; then
  mkdir -p ~/git 
  cd ~/git
fi

cd ~/git
if [ ! -d acados ]; then
  git clone https://github.com/acados/acados.git
fi
cd acados
git fetch
git checkout v0.3.3 # checkout on exact version
git submodule update --recursive --init # there is many submodules 

## | ------------------ clone fast_nmpc ------------------ |

echo "$0: Clone fast_nmpc"

cd ~/git
if [ ! -d fast_nmpc ]; then
  git clone git@mrs.fel.cvut.cz:agile-flight/fast_nmpc.git
fi

# update submodules
cd ~/git/fast_nmpc
gitman install --force

## | ------------------- prepare workspace ------------------- |

echo "$0: Prepare workspace"

cd ~/

# create workspace if not exists already
if [ ! -d nmpc_workspace ]; then
  mkdir -p ~/nmpc_workspace/src && cd ~/nmpc_workspace
  cd ~/nmpc_workspace/src
  ln -s $(realpath ~/git/fast_nmpc)
fi

