cmake_minimum_required(VERSION 3.0.2)
project(ambs_core)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    ambs_msgs
    std_msgs
    geometry_msgs
    roslint
    tf
    tf_conversions
    angles
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ambs_msgs
  DEPENDS std_msgs geometry_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()
