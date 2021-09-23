cmake_minimum_required(VERSION 3.0.2)
project(pull_test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

add_subdirectory(third_party/my_yaml)
add_subdirectory(third_party/my_cdd)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
set(CMAKE_MODULE_PATH 
      ${CMAKE_MODULE_PATH} 
      ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
find_package(DART 6.9 REQUIRED COMPONENTS utils-urdf gui-osg CONFIG)

# find_package(MOSEK)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 LIBRARIES pull_test
 INCLUDE_DIR include
)

###########
## Build ##
###########
set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(MYYAML_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/third_party/my_yaml/include)
set(my_cdd_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/third_party/my_cdd/src )                       

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${DART_INCLUDE_DIRS}
  ${MYYAML_INCLUDE_DIR}
  ${my_cdd_INCLUDE_DIR}
)

FILE(GLOB_RECURSE sources "*.cpp")

FILE(GLOB runfile "src/magneto/main.cpp")


FILE(GLOB io_headers "include/pull_test/my_utils/io/*.hpp")
FILE(GLOB math_headers "include/pull_test/my_utils/math/*.hpp")

FILE(GLOB robot_system_headers "include/pull_test/my_robot_system/*.hpp")
FILE(GLOB geometry_headers "include/pull_test/my_geometry/polytope/*.hpp" "*.h")
FILE(GLOB magento_headers "include/pull_test/magneto/*.hpp")
FILE(GLOB pull_test_headers "include/pull_test/*.hpp")

## Declare a C++ library
#message("${sources}")
#message("${headers}")
message("=================================")
message("${DART_LIBRARIES}")
message("${DART_INCLUDE_DIRS}")
add_library(pull_test SHARED  ${sources}
                            ${headers})

target_include_directories(pull_test PUBLIC   
                    ${PROJECT_INCLUDE_DIR}
                    ${MYYAML_INCLUDE_DIR}
                    ${my_cdd_INCLUDE_DIR}
                    ${DART_INCLUDE_DIRS})

target_link_libraries(pull_test ${DART_LIBRARIES}
                                ${EIGEN_LIBRARIES}
                                my_yaml
                                my_cdd)

add_executable(run_pull_test ${runfile})
target_link_libraries(run_pull_test pull_test)

#############
## Install ##
#############


message("[pull_test]INSTALL_LIB_DIR=")
message("${CATKIN_PACKAGE_LIB_DESTINATION}")
message("[pull_test]INSTALL_INCLUDE_DIR=")
message("${CATKIN_PACKAGE_INCLUDE_DESTINATION}")
message("[pull_test]CATKIN_PACKAGE_BIN_DESTINATION=")
message("${CATKIN_PACKAGE_BIN_DESTINATION}")

install(TARGETS run_pull_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS pull_test DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")

install(DIRECTORY robot_description DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES ${io_headers} DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}/pull_test/my_utils/io")
install(FILES ${math_headers} DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}/pull_test/my_utils/math")
install(FILES ${robot_system_headers} DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}/pull_test/my_robot_system")
install(FILES ${geometry_headers} DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}/pull_test/my_geometry/polytope")
install(FILES ${magento_headers} DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}/pull_test/magneto")
install(FILES ${pull_test_headers} DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}/pull_test")


## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pull_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

SET (THIS_COM ${CMAKE_SOURCE_DIR}/ )
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/config.h.cmake ${PROJECT_SOURCE_DIR}/Configuration.h)
install(FILES ${PROJECT_SOURCE_DIR}/Configuration.h DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}")