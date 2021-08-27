cmake_minimum_required(VERSION 3.0.2)
project (my_yaml)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
catkin_package(
 LIBRARIES my_yaml
)

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_INCLUDE_DIR}
)



file(GLOB_RECURSE sources "*.cpp")
file(GLOB yaml_headers "include/my_yaml/*.h")
file(GLOB yaml_contrib_headers "include/my_yaml/contrib/*.h")
file(GLOB yaml_node_headers "include/my_yaml/node/*.h")
file(GLOB yaml_node_detail_headers "include/my_yaml/node/detail/*.h")

add_library(my_yaml SHARED ${sources}
                          ${yaml_headers}
                          ${yaml_contrib_headers}
                          ${yaml_node_headers}
                          ${yaml_node_detail_headers})

message("[my_yaml]INSTALL_INCLUDE_DIR=")
message("${CATKIN_PACKAGE_INCLUDE_DESTINATION}")
message("[my_yaml]INSTALL_LIB_DIR=")
message("${CATKIN_PACKAGE_LIB_DESTINATION}")
install(TARGETS my_yaml DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
install(FILES ${yaml_headers} DESTINATION
    "${CATKIN_PACKAGE_INCLUDE_DESTINATION}")
install(FILES ${yaml_contrib_headers} DESTINATION
    "${CATKIN_PACKAGE_INCLUDE_DESTINATION}/contrib")
install(FILES ${yaml_node_headers} DESTINATION
    "${CATKIN_PACKAGE_INCLUDE_DESTINATION}/node")
install(FILES ${yaml_node_detail_headers} DESTINATION
    "${CATKIN_PACKAGE_INCLUDE_DESTINATION}/node/detail")
