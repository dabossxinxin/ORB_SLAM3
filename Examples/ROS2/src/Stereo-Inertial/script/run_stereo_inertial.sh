#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
source "$PKG_DIR/install/setup.bash"

#ros2 run --prefix 'gdb -ex run -ex bt --args' stereo_inertial stereo_inertial_node  \
#    "$PKG_DIR/../../Vocabulary/ORBvoc.txt" \
#    "$PKG_DIR/src/Stereo-Inertial/config/RealSense_D435i.yaml" \
#    "$PKG_DIR/src/Stereo-Inertial/log/"

ros2 run stereo_inertial stereo_inertial_node  \
    "$PKG_DIR/../../Vocabulary/ORBvoc.txt" \
    "$PKG_DIR/src/Stereo-Inertial/config/RealSense_D435i.yaml" \
    "$PKG_DIR/src/Stereo-Inertial/log/"
