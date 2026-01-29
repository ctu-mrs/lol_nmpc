#!/bin/bash

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

arch=`uname -i`

# define custom colors
Yellow='\033[0;33m'
LBlue='\e[1;34m'
NC='\033[0m' # No Color

## | --------- change to the directory of this script --------- |

cd "$MY_PATH"

## | ------------------------- acados ------------------------- |

bash $MY_PATH/dependencies/acados.sh

## | ------------------------- tera_renderer ------------------------- |

if [ "$arch" != "x86_64" ]; then
  bash $MY_PATH/dependencies/tera_renderer.sh
fi

## | -------------------------- eigen ------------------------- |

sudo apt install -y libeigen3-dev

## | ------------------------ yaml-cpp ------------------------ |

sudo apt-get install libyaml-cpp-dev

echo -e "${Yellow}Do not forget to modify the .bashrc by sourcing only the new workspace.${NC}"
