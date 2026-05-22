#!/bin/bash -e

if [ $# -eq 0 ]; then
  echo 'colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)'
  colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
else
  echo "colcon build --symlink-install --packages-select $@ --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)"
  colcon build --symlink-install --packages-select $@ --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
fi

echo './install/local_setup.bash'
source ./install/local_setup.bash
