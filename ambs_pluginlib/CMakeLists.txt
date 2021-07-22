cmake_minimum_required(VERSION 3.0.2)
project(ambs_pluginlib)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  ambs_core
  ambs_msgs
  ambs_components
  roslint
)

catkin_package(
   LIBRARIES ambs_calculators ambs_runners
   CATKIN_DEPENDS nodelet roscpp ambs_msgs ambs_core ambs_components
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(ambs_calculators
   src/calculators/comparators/comp_float_param_nodelet.cpp
   src/calculators/differentiators/diff_float_temporal_nodelet.cpp
   src/calculators/differentiators/diff_pose_temporal_nodelet.cpp
   src/calculators/differentiators/stopwatch_nodelet.cpp
 )
add_dependencies(ambs_calculators ${catkin_EXPORTED_TARGETS})
target_link_libraries(ambs_calculators ${catkin_LIBRARIES})

add_library(ambs_runners
   src/runners/test1_braking_nodelet.cpp
 )
add_dependencies(ambs_runners ${catkin_EXPORTED_TARGETS})
target_link_libraries(ambs_runners ${catkin_LIBRARIES})

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()
