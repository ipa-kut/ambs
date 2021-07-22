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
  LIBRARIES ambs_components
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ambs_components
   src/interpreters/command_interpreters/velocity_interpreter.cpp
   src/interpreters/location_interpreters/odom_interpreter.cpp
 )
add_dependencies(ambs_components ${catkin_EXPORTED_TARGETS})
target_link_libraries(ambs_components ${catkin_LIBRARIES})

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright,-whitespace/braces")
roslint_cpp()
