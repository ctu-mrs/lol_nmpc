#!/bin/bash


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
