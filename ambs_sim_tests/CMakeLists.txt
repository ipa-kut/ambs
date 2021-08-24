cmake_minimum_required(VERSION 2.8.3)
project(ambs_sim_tests)

find_package(catkin REQUIRED COMPONENTS
    roslint
)

catkin_package(
   CATKIN_DEPENDS
)

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
