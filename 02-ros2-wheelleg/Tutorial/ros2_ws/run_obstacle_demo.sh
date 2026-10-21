#!/usr/bin/env bash
set -e
assignment_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$HOME/miniforge3/etc/profile.d/conda.sh"
conda activate rm_ros2
export PYTHONNOUSERSITE=1
source /opt/ros/jazzy/setup.bash
if [ ! -f "$assignment_ws/install/setup.bash" ]; then
  echo "请先把 src、obstacle_demo 和本脚本放入教程工作区，完成 colcon build。"
  exit 1
fi
source "$assignment_ws/install/setup.bash"
python "$assignment_ws/obstacle_demo/run_demo.py"
