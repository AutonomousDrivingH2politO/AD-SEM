#!/bin/bash -e

clear

echo 'colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=RelWithDebInfo'
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=RelWithDebInfo --parallel-workers $(nproc)

echo './install/local_setup.bash'
source ./install/local_setup.bash
