#! /bin/sh
export  VCPKG_ROOT="/opt/vcpkg" #Change this to your directory where vcpkg is stored
export PATH=$VCPKG_ROOT:$PATH

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

echo "[+] Success!" 
exit
