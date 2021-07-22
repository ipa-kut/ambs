cmake_minimum_required(VERSION 3.0.2)
project(ambs_components)

find_package(catkin REQUIRED COMPONENTS
    ambs_core
    ambs_msgs
    geometry_msgs
    nav_msgs
    roscpp
    roslint
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    ambs_core
    ambs_msgs
    geometry_msgs
    nav_msgs
    roscpp
    std_msgs
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/interpreters/command_interpreters/velocity_interpreter.cpp
   src/interpreters/location_interpreters/odom_interpreter.cpp
 )
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright,-whitespace/braces")
roslint_cpp()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
