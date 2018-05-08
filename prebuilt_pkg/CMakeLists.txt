cmake_minimum_required(VERSION 2.8.3)
project(prebuilt_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mylib
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )


# used to create mylib shared library
# add_library(mylib
#   include/mylib.h
#   src/mylib.cpp)

find_library(MYLIB NAMES "mylib" PATHS "lib")
set(LIB_DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})


# add imported library
# add imported library
add_library(mylib SHARED IMPORTED)
set_property(TARGET mylib PROPERTY IMPORTED_LOCATION ${MYLIB})

# manually copy to devel space
file(COPY ${MYLIB}
  DESTINATION ${LIB_DESTINATION}
  )


# Mark header & libraries for installation
install( DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install( FILES ${MYLIB}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
