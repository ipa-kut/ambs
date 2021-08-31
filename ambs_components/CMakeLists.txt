cmake_minimum_required(VERSION 3.0.2)
project(ambs_components)

find_package(catkin REQUIRED COMPONENTS
    ambs_core
    ambs_msgs
    diagnostic_msgs
    geometry_msgs
    nav_msgs
    roscpp
    roslint
    rospy
    std_msgs
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    ambs_core
    ambs_msgs
    diagnostic_msgs
    geometry_msgs
    nav_msgs
    roscpp
    rospy
    std_msgs
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   scripts/rosbagger_node
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(${PROJECT_NAME}
   src/interpreters/command_interpreters/velocity_interpreter.cpp
   src/interpreters/location_interpreters/odom_interpreter.cpp
   src/loggers/debug_logger.cpp
 )
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright,-whitespace/braces, -build/c++11")
roslint_cpp()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
