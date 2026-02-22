#!/bin/bash


set -e  # exit on error

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

arch=`uname -i`

echo "$0: Clone acados"

cd ~/git
if [ ! -d acados ]; then
  git clone https://github.com/acados/acados.git
fi
cd acados
git fetch
git checkout v0.3.3 # checkout on exact version
git submodule update --recursive --init # there is many submodules 

cd ~/git/acados

if [ ! -d build ]; then
  mkdir -p build
fi

cd build
cmake -DACADOS_WITH_QPOASES=ON ..
# add more optional arguments e.g. -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_OSQP=OFF/ON -DACADOS_INSTALL_DIR=<path_to_acados_installation_folder> above
make install -j4

sudo pip install casadi==3.6.3
sudo pip install -e ~/git/acados/interfaces/acados_template

# Install Rust (non-interactive)
echo "Installing Rust..."
curl https://sh.rustup.rs -sSf | sh -s -- -y
source $HOME/.cargo/env

# Clone and build tera_renderer
echo "Cloning and building tera_renderer..."
mkdir -p ~/git
cd ~/git
if [ ! -d tera_renderer ]; then
  git clone https://github.com/acados/tera_renderer.git
fi

cd tera_renderer
cargo build --release

# Symlink t_renderer into acados/bin
ACADOS_BIN_DIR=$HOME/git/acados/bin
mkdir -p "$ACADOS_BIN_DIR"
ln -sf "$HOME/git/tera_renderer/target/release/t_renderer" "$ACADOS_BIN_DIR/t_renderer"
