#! /bin/sh
export  VCPKG_ROOT="/opt/vcpkg" #Change this to your directory where vcpkg is stored
export PATH=$VCPKG_ROOT:$PATH

echo "[+] Grabbing external header files"
cd ./external

if [ -d "eigen" ]
then
    echo "[~] Eigen3 library already detected. Continuing with other headers."
else
    EIGEN_VERSION="3.4.0"
    echo "[+] Getting Eigen 3.4.0 from https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-${EIGEN_VERSION}.zip"
    wget "https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-${EIGEN_VERSION}.zip" 
    unzip -q ./eigen-${EIGEN_VERSION}.zip
    mv ./eigen-${EIGEN_VERSION} ./eigen
    rm ./eigen-${EIGEN_VERSION}.zip
fi
cd ../

echo "[+] Making Build Directory"
if [ ! -d "build" ]
then 
 mkdir build

else
 echo "[~] Build Folder already exists! continuing.."
fi

chmod +x ./build
if [ $(echo $(cmake --version) |grep "version" | cut -d " " -f3) != "3.27.1" ]
then 
 echo "[~] Warning: Cmake Version is not correct.Will still try anyways."
fi

echo "[+] Building Cmake files"
cd ./build
cmake -S ../ -B .

cd ../

echo "[+] Success!" 
exit
