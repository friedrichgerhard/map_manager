# map_manager

This is a ROS-package allowing a robot create multiple maps automatically by detecting a doorway passing and choose an appropriate SLAM configuration for the new map. This code belongs to a conference paper submitted for review. More detailed informations are coming soon.

### Prerequisites
The robot the package was developed with is running Ubuntu 16.04 with ROS kinetic.
Therefore, to use the package the following prerequisites must be met:

* [Ubuntu](http://releases.ubuntu.com/16.04/) - A working install of Ubuntu 16.04 - Xenial Xerus
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) - ROS kinetic installation
* [Configured ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) - ROS workspace created

In addition, the ros package pose_cov_ops is required. For installation run:
```
sudo apt-get install ros-kinetic-pose-cov-ops
```
Furthermore, the library g2o has to be installed:
```
git clone https://github.com/RainerKuemmerle/g2o.git
mkdir g2o-build; cd g2o-build
```
To install dependencies run:
```
sudo apt-get install build-essential cmake freeglut3 freeglut3-dev freeglut3-dbg qt4-qmake libqglviewer2 libqglviewer-dev libqglviewer-doc libeigen3-dev libeigen3-doc
```
Then
```
cmake ../g2o
make
sudo make install
```
