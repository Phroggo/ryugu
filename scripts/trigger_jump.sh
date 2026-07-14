#!/bin/bash
TARGET=${1:-5.0}
echo "Commanding scout_1 to jump with target distance: $TARGET meters"
ros2 topic pub --once /scout_1/jump_target_distance std_msgs/msg/Float64 "{data: $TARGET}"
