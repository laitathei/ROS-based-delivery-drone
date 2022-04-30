### Gazebo simulation

### 1. Software Prerequisite
### 1.1 PX4 ROS Gazebo environment
```
pip3 install --user empy
pip3 install --user toml
pip3 install --user numpy
pip3 install --user pyros-genmsg
pip3 install kconfiglib
pip3 install --user packaging
pip3 install --user jinja2
pip3 install --user jsonschema
sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras     ** for melodic
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras     ** for noetic
sudo apt install gcc-arm-none-eabi
sudo apt install gperf
sudo apt-get install python-dev python3-dev libxml2-dev libxslt1-dev zlib1g-dev
sudo apt upgrade libignition-math2          **for gazebo error which cause the gazebo cannot launch

gedit ~/.bashrc
#put this statement in .bashrc
export OPENBLAS_CORETYPE=ARMV8 python3

cd ~/Desktop
wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
chmod +x ubuntu_sim_ros_melodic.sh
sudo bash install_geographiclib_datasets.sh
source ubuntu_sim_ros_melodic.sh
cd

git clone https://github.com/PX4/PX4-Autopilot.git
cd ~/PX4-Autopilot
git checkout v1.12.3
bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx
git submodule update --init --recursive

make px4_fmu-v3_default **refer to https://docs.px4.io/master/en/dev_setup/building_px4.html to check version which only for hardware setup

cd ~/PX4-Autopilot
make px4_sitl_default gazebo

## Type the following code into .bashrc
source ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/sitl_gazebo

roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"   **only sitl with gazebo
roslaunch px4 mavros_posix_sitl.launch        **SITL and MAVROS
```

### 1.2 Realsense Gazebo plugin
```
cd ~/catkin_ws/src
git clone https://github.com/nilseuropa/realsense_ros_gazebo.git
cd ~/catkin_ws
catkin_make
```

### 1.3 jsk_pcl_ros installation
```
# Ubuntu 18.04
sudo apt-get install ros-melodic-jsk-pcl-ros
sudo apt-get install ros-melodic-jsk-rviz-plugins
sudo apt-get install ros-melodic-ros-numpy

# Ubuntu 20.04
sudo apt-get install ros-noetic-jsk-pcl-ros
sudo apt-get install ros-noetic-jsk-rviz-plugins
sudo apt-get install ros-noetic-ros-numpy
```

### 1.4 QGC installation
```
#### build from source
sudo apt-get install speech-dispatcher libudev-dev libsdl2-dev
cd
git clone --recursive -j8 https://github.com/mavlink/qgroundcontrol.git
git submodule update --recursive
***create account for QT***
***download the online installer from https://www.qt.io/download-qt-installer?hsCtaTracking=99d9dd4f-5681-48d2-b096-470725510d34%7C074ddad0-fdef-4e53-8aa8-5e8a876d6ab4***
chmod +x qt-unified-linux-x64-4.1.1-online.run
./qt-unified-linux-x64-4.1.1-online.run
***follow https://dev.qgroundcontrol.com/master/en/getting_started/index.html QT part 2 step to install the correct version***
sudo apt install libsdl2-dev
_____________________________________________________________________________________________________________________________________________________________

#### follow the official tutorial (https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
***Download QGroundControl.AppImage***
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage
```

### 1.5 ROS Octomap installation
```
# Ubuntu 18.04
sudo apt-get install ros-melodic-octomap-ros
sudo apt-get install ros-melodic-octomap-msgs 
sudo apt-get install ros-melodic-octomap-server
sudo apt-get install ros-melodic-octomap-rviz-plugins
sudo apt-get install ros-melodic-octomap-mapping
sudo apt-get install ros-melodic-octomap

# Ubuntu 20.04
sudo apt-get install ros-noetic-octomap-ros
sudo apt-get install ros-noetic-octomap-msgs 
sudo apt-get install ros-noetic-octomap-server
sudo apt-get install ros-noetic-octomap-rviz-plugins
sudo apt-get install ros-noetic-octomap-mapping
sudo apt-get install ros-noetic-octomap
```

### 1.6 Python3 ROS melodic installation (if required)
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/install/setup.bash --extend" >> ~/.bashrc
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep

sudo apt-get install python-pip python-yaml 
sudo apt-get install python3-pip python3-yaml 
sudo pip3 install rospkg catkin_pkg
sudo apt-get install python-catkin-tools 
sudo apt-get install python3-catkin-tools 
sudo apt-get install python3-dev python3-numpy
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
```

### 1.7 ceres-solver installation (if required)
```
***get the latest stable version of ceres-solver from http://ceres-solver.org/installation.html***
sudo apt-get install cmake
sudo apt-get install libgoogle-glog-dev libgflags-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libeigen3-dev
sudo apt-get install libsuitesparse-dev
tar zxf ceres-solver-2.0.0.tar.gz
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-2.0.0
make -j3
make test
sudo make install
```

### 1.8 Vins-Fusion (if required)
```
sudo apt-get install cmake
sudo apt-get install libgoogle-glog-dev libgflags-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libeigen3-dev
sudo apt-get install libsuitesparse-dev

tar zxf ceres-solver-2.0.0.tar.gz
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-2.0.0
make -j3
make test
make install

cd ~/catkin_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```
