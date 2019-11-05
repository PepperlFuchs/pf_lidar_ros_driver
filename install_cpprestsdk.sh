#!/bin/bash

set -e
apt update
apt install -y git libwebsocketpp libssl-dev

rm -rf ~/cpprest
mkdir ~/cpprest
cd ~/cpprest/
git clone https://github.com/microsoft/cpprestsdk -b v2.10.2
cd cpprestsdk/
mkdir build
cd build
cmake ../Release/
make
make install
cd
rm -rf ~/cpprest