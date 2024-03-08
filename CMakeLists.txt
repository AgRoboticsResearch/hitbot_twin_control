cmake_minimum_required(VERSION 3.0.2)
project(hitbot_twin_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  control_msgs
  actionlib_msgs
  actionlib
)


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

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
  CATKIN_DEPENDS 
    controller_manager 
    hardware_interface 
    roscpp 
    control_msgs 
    actionlib_msgs
)

###########
## Build ##
###########
find_library(hitbot_interface NAMES hitbot_interface 
             PATHS ${PROJECT_SOURCE_DIR}/extern NO_DEFAULT_PATH)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/extern/include
  ${PROJECT_SOURCE_DIR}/include
)

## Declare a C++ library
set(hitbot_interface ${PROJECT_SOURCE_DIR}/extern/libsmall_scara_interface.so)

add_executable(${PROJECT_NAME}_node src/hitbot_twin_encoder.cpp)           
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} 
                                           ${hitbot_interface}) 
