echo "Building ORB_SLAM3 Library"

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF
make -j8

cd ..
echo "Building ROS2 nodes"

cd Examples/ROS2
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
