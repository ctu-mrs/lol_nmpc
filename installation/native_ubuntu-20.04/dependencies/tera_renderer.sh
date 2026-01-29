#!/bin/bash

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

# define custom colors
Yellow='\033[0;33m'
LBlue='\e[1;34m'
NC='\033[0m' # No Color

set -e

## | ---------------------- install rust ---------------------- |

curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

## | ------------------- clone tera_renderer ------------------ |

echo "$0: Clone acados"

cd ~/git
if [ ! -d tera_renderer ]; then
  git clone https://github.com/acados/tera_renderer.git
fi

cd tera_renderer 
source $HOME/.cargo/env
cargo build --verbose --release

ACADOS_BIN_DIR_DIR=/home/$USER/git/acados/bin/
if [ -d "$ACADOS_BIN_DIR_DIR" ];
then
    echo "$ACADOS_BIN_DIR_DIR directory exists."
    rm -f /home/$USER/git/acados/bin/t_renderer
    ln -s /home/$USER/git/tera_renderer/target/release/t_renderer /home/$USER/git/acados/bin/t_renderer
else
	echo "$ACADOS_BIN_DIR_DIR directory does not exist."
  exit 1
fi
