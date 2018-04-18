#!/bin/bash 

source ./project.sh


sudo apt-get install git build-essential python libusb-1.0-0-dev freeglut3-dev openjdk-8-jdk

sudo apt-get install doxygen graphviz mono-complete


mkdir kinect

cd kinect

mypwd=$PWD

git clone https://github.com/OpenNI/OpenNI.git
cd OpenNI
git checkout Unstable-1.5.4.0
cd Platform/Linux/CreateRedist
chmod +x RedistMaker
./RedistMaker
cd ../Redist/OpenNI-Bin-Dev-Linux-x64-v1.5.4.0 

sudo ./install.sh


cd $mypwd

git clone https://github.com/avin2/SensorKinect
cd SensorKinect
cd Platform/Linux/CreateRedist
chmod +x RedistMaker
./RedistMaker
cd ../Redist/Sensor-Bin-Linux-

chmod +x install.sh
sudo ./install.sh

sudo apt-get install ros-kinetic-openni*

cd $mypwd
git clone https://github.com/arnaud-ramey/NITE-Bin-Dev-Linux-v1.5.2.23
cd NITE-Bin-Dev-Linux-v1.5.2.23/x64

sudo ./install.sh
