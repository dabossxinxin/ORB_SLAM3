echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8

cd ../../g2o
echo "Configuring and building Thirdparty/g2o ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8

cd ../../Sophus
echo "Configuring and building Thirdparty/Sophus ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8

cd ../../../
echo "Uncompress vocabulary ..."

cd Vocabulary
if [ ! -f ORBvoc.txt ]; then
    tar -xf ORBvoc.txt.tar.gz
    echo "Vocabulary file extracted"
else
    echo "Vocabulary file already exists, skipping extraction"
fi

cd ..
echo "Configuring and building ORB_SLAM3 ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
