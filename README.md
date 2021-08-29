# Pull Test for a climbing
pull_test is a Software designed to compute required adhesive force threshold for a climbing robot 

This software framework is developed by Jee-Eun Lee (https://github.com/jeeeunlee/ros-pnc.git)


# Pull Test for Magento
Pull Test for Magento is now available. 

## Run the Code
```
$ source install/setup.bash
$ rosrun pull_test run_pull_test
```

### Install Required Dependancies
dart is used to calculate some kinematics of a robot

- run ```source install_sim.sh``` for [Dart 6.9.0](https://dartsim.github.io/install_dart_on_mac.html) and [pybullet](https://pybullet.org/wordpress/)

### Compile the Code
```
$ catkin build
```
