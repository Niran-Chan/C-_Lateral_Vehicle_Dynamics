#! /bin/sh

echo "[+] Building Project based on current system"
cd ./build
make VERBOSE=1
make install
echo "[+] Done"

