cmake_minimum_required(VERSION 3.0.2)
project(ambs_components)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ambs_msgs
  ambs_core
  roslint
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ambs_msgs ambs_core
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright,-whitespace/braces")
roslint_cpp()
