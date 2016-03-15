# Socially Normative Robot Navigation Behaviors

This meta-package contains a set of packages that implement different
socially normative mobile robot navigation behaviors particularly geared 
at service robots operating in crowded scenes.

## Depencies
* C++11 compiler
* ROS setup (including `move_base`, rviz)
* Spencer related messages and robot control

## Usage
```
# checkout into a catkin workspace
catkin build socially_normative_navigation
roslaunch snn_launchers lobby.launch
```

## Authors
Billy Okal

## Note
This is research code used actively in the SPENCER project. If you use
this code, please cite any the following papers;

```
@InProceedings{okalRSSLfd15,
   author = {Okal, Billy and Arras, Kai O.},
    title = {Learning Socially Normative Robot Navigation Behaviors
using Bayesian Inverse Reinforcement Learning},
    booktitle={IEEE Int. Conference on Robotics and Automation
(ICRA)},
    address = {Stockholm, Sweeden},
    year={2016},
}
```
