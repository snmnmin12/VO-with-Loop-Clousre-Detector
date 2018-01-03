cd ThirdParty/DBoW2
mkdir build
cd build
cmake ..
make
cd ../../DLib

#go to the next dependency
mkdir build
cd build
cmake ..
make
cd ../../..

cd resources
tar -xf ORBvoc.txt.tar.gz
cd ..

mkdir build

cd build
cmake ..
make
cd ..