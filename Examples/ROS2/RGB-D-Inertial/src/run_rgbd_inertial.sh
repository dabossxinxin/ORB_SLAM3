
source ../install/setup.bash &&
ros2 run orbslam3 rgbd_inertial_ros2 \
    ../../../../Vocabulary/ORBvoc.txt ./config/RealSense_D435i.yaml \
    /home/xinxin/Data/Realsense-D435i/unitree_legged_robotic_datasets-002/