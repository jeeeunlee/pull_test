# Pull Test for a climbing
pull_test is a Software designed to compute required adhesive force threshold for a climbing robot 

This software framework is developed by Jee-Eun Lee (https://github.com/jeeeunlee/pull_test.git)


# Pull Test for Magento
Pull Test for Magento is now available. 



## Install Required Dependancies
dart is used to calculate some kinematics of a robot

- run ```source install_sim.sh``` for [Dart 6.9.0](https://dartsim.github.io/install_dart_on_mac.html) 

## Download the Code
```
$ cd (your_catkin_ws)/src
$ git clone https://github.com/jeeeunlee/pull_test.git
```

## Compile the Code
```
$ catkin build pull_test
```

## Run the Code
```
$ source install/setup.bash
$ rosrun pull_test run_pull_test
```