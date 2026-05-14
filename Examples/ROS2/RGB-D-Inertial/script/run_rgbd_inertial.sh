#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"

source "$PKG_DIR/install/setup.bash"

ros2 run orbslam3 rgbd_inertial_ros2 \
    "$PKG_DIR/../../../Vocabulary/ORBvoc.txt" \
    "$PKG_DIR/src/config/RealSense_D435i.yaml" \
    "$PKG_DIR/src/log/"