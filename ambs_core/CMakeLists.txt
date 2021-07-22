cmake_minimum_required(VERSION 3.0.2)
project(ambs_core)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    ambs_msgs
    angles
    geometry_msgs
    roscpp
    roslint
    std_msgs
    tf
    tf_conversions
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    ambs_msgs
    geometry_msgs
    roscpp
    std_msgs
    tf
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
