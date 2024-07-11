#!/bin/bash

# Install essential packages
sudo apt-get install wget unzip build-essential cmake

# Download torch cpp
export DIRNAME=$(dirname -- "$0")/cpp/glasshouse_mapping
URL="https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.3.1%2Bcpu.zip"

if [ ! -f "${DIRNAME}/libtorch/build-version" ]; then
  echo "Downloading Torch, it might take a while..."
  wget -c ${URL} -O ${DIRNAME}/libtorch.zip
  unzip ${DIRNAME}/libtorch.zip -d ${DIRNAME}
  rm ${DIRNAME}/libtorch.zip
  echo "Done"
else
  echo "Torch folder is already there, nothing to do"
fi

# Install other dependencies
sudo apt-get install libnanoflann-dev=1.4.2+ds-1
sudo apt-get install libeigen3-dev=3.4.0-2ubuntu2
sudo apt-get install libtbb-dev=2021.5.0-7ubuntu2
sudo apt-get install libopen3d-dev=0.14.1+dfsg-7build3
sudo apt-get install libopencv-dev=4.5.4+dfsg-9ubuntu4
sudo apt-get install libceres-dev=2.0.0+dfsg1-5
sudo apt-get install libjsoncpp-dev=1.9.5-3
sudo apt-get install libyaml-cpp-dev=0.7.0+dfsg-8build1
