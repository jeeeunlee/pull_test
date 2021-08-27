cmake_minimum_required(VERSION 3.0.2)
project (my_cdd)

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_INCLUDE_DIR}
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
catkin_package(
 LIBRARIES my_cdd
)

FILE(GLOB_RECURSE headers "*.h" "*.hpp")
file(GLOB_RECURSE sources *.c)

add_library (my_cdd SHARED ${headers} ${sources})
target_include_directories(my_cdd PUBLIC   
                      ${PROJECT_INCLUDE_DIR})


message("[my_cdd]INSTALL_INCLUDE_DIR=")
message("${CATKIN_PACKAGE_INCLUDE_DESTINATION}")
message("[my_cdd]INSTALL_LIB_DIR=")
message("${CATKIN_PACKAGE_LIB_DESTINATION}")
install(TARGETS my_cdd DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
install(FILES ${headers} DESTINATION
   "${CATKIN_PACKAGE_INCLUDE_DESTINATION}")
