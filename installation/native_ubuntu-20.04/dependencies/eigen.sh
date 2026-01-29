#!/bin/bash

set -e

Yellow='\033[0;33m'
LBlue='\e[1;34m'
NC='\033[0m' # No Color

echo "$0: Installing eigen lib"

cd ~/git
if [ ! -d eigen ]; then
  git clone git@gitlab.com:libeigen/eigen.git
fi

cd eigen

if [ ! -d build ]; then
  mkdir build
fi

cd build
cmake ..\
sudo make install
# The "make install" step may require administrator privileges.

