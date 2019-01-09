# Jetyak UAV Utilities

***Aiming to provide an interface for the interaction of UAV and jetyaks***

## Prerequisites
Here listed a few dependencies that need to be downloaded from the corresponding
repository.

* DJI Onboard SDK
```
git clone https://github.com/dji-sdk/Onboard-SDK.git
cd Onboard-SDK
mkdir build
cd build
cmake ..
make
sudo make install
```

* DJI Onboard SDK ROS from the catkin_ws/src directory
```
git clone https://github.com/dji-sdk/Onboard-SDK-ROS
catkin_make -C ..
```

* ar\_track\_alvar
```
sudo apt install ros-<VERSION>-ar-track-alvar
```
