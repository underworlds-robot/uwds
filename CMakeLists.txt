cmake_minimum_required(VERSION 2.8.3)
project(uwds)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nodelet
  pluginlib
  rostime
  urdf
  uwds_msgs
  geometry_msgs
  sensor_msgs
  jsk_recognition_msgs
  visualization_msgs
  ar_track_alvar_msgs
  tf
  tf2_ros
  pose_cov_ops
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(Assimp REQUIRED COMPONENTS system)
find_package(octomap REQUIRED)

catkin_python_setup()

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
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp nodelet pluginlib rostime tf tf2_ros pose_cov_ops
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/tools/model_loader.cpp
   src/uwds.cpp
   src/server.cpp
   src/uwds_client_nodelet.cpp
   src/reconfigurable_client.cpp
   src/uwds_provider.cpp
   src/env_provider.cpp
   src/scene_viewer.cpp
   src/world_merger.cpp
   src/ar_objects_provider.cpp
   src/simple_object_provider.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} assimp)

#add_dependencies(${PROJECT_NAME})

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_LIBRARIES})
endif()

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/test.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
