cmake_minimum_required(VERSION 2.8.3)
project(kdl_parser_example)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  roscpp
  rospy
)

# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kdl_parser_example
#  CATKIN_DEPENDS kdl_parser roscpp rospy
#  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(my_kdl_parser src/main.cpp)
target_link_libraries(my_kdl_parser
  ${catkin_LIBRARIES}
)