cmake_minimum_required(VERSION 3.0.2)
project(ambs_pluginlib)

find_package(catkin REQUIRED COMPONENTS
  ambs_components
  ambs_core
  ambs_msgs
  nodelet
  roscpp
  roslint
)

catkin_package(
   LIBRARIES ambs_calculators ambs_runners
   CATKIN_DEPENDS
    ambs_components
    ambs_core
    ambs_msgs
    nodelet
    roscpp
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
   src/runners/empty_runner_nodelet.cpp
   src/runners/test1_braking_nodelet.cpp
 )
add_dependencies(ambs_runners ${catkin_EXPORTED_TARGETS})
target_link_libraries(ambs_runners ${catkin_LIBRARIES})

add_library(ambs_interpreters
   src/interpreters/command_interpreters/velocity_interpreter_nodelet.cpp
   src/interpreters/location_interpreters/odom_interpreter_nodelet.cpp
 )
add_dependencies(ambs_interpreters ${catkin_EXPORTED_TARGETS})
target_link_libraries(ambs_interpreters ${catkin_LIBRARIES})

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()

install(TARGETS ambs_calculators
                ambs_interpreters
                ambs_runners
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
