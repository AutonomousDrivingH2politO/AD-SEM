#!/bin/bash -e

clear

echo 'ament_uncrustify --reformat src'
ament_uncrustify --reformat src

echo 'colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)'
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)

echo './install/local_setup.bash'
source ./install/local_setup.bash
