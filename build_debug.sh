#!/bin/bash -e

echo 'colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Debug'
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Debug --allow-overriding camera_info_manager image_transport --parallel-workers $(nproc)

echo './install/local_setup.bash'
source ./install/local_setup.bash
