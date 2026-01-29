#!/bin/bash

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

# define custom colors
Yellow='\033[0;33m'
LBlue='\e[1;34m'
NC='\033[0m' # No Color

set -e

## | ---------------------- clone acados ---------------------- |

echo "$0: Clone acados"

cd ~/git
if [ ! -d acados ]; then
  git clone https://github.com/acados/acados.git
fi
cd acados
git fetch
git checkout v0.3.3 # checkout on exact version
git submodule update --recursive --init # there is many submodules 


## | ---------------- build and install acados ---------------- |

echo -e "${Yellow}Build and install acados${NC}"
cd ~/git/acados

if [ ! -d build ]; then
  mkdir -p build
fi

cd build
cmake -DACADOS_WITH_QPOASES=ON ..
# add more optional arguments e.g. -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_OSQP=OFF/ON -DACADOS_INSTALL_DIR=<path_to_acados_installation_folder> above
make install -j4

## | ------------- install casadi specific version ------------ |

pip install casadi==3.6.3

## | ------------ install python templates package ------------ |

pip install -e /home/$USER/git/acados/interfaces/acados_template

## | ------------- add acados sourcing to .bashrc ------------- |

line='export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$HOME/git/acados/lib"'
num=`cat ~/.bashrc | grep "$line" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "Adding '$line' to your .bashrc"

  # set exports to .bashrc
  echo "
$line" >> ~/.bashrc
fi

line='export ACADOS_SOURCE_DIR="$HOME/git/acados"'
num=`cat ~/.bashrc | grep "$line" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "Adding '$line' to your .bashrc"

  # set exports to .bashrc
  echo "
$line" >> ~/.bashrc
fi

## | ------------- add acados sourcing to .zshrc  ------------- |

line='export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$HOME/git/acados/lib"'
num=`cat ~/.zshrc | grep "$line" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "Adding '$line' to your .zshrc"

  # set exports to .bashrc
  echo "
$line" >> ~/.zshrc
fi

line='export ACADOS_SOURCE_DIR="$HOME/git/acados"'
num=`cat ~/.zshrc | grep "$line" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "Adding '$line' to your .zshrc"

  # set exports to .bashrc
  echo "
$line" >> ~/.zshrc
fi
