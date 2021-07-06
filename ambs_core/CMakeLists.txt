cmake_minimum_required(VERSION 3.0.2)
project(ambs_core)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    ambs_msgs
    std_msgs
    roslint
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ambs_msgs
  DEPENDS std_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(test_base src/test_base.cpp
                         src/ambs_base/ambs_base.cpp
               )
target_link_libraries(test_base ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(test_base ${catkin_EXPORTED_TARGETS})

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()
