cmake_minimum_required(VERSION 3.0.2)
project(ambs_tests)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  ambs_components
  ambs_core
  ambs_msgs
  ambs_pluginlib
  geometry_msgs
  roscpp
  roslint
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS
    ambs_components
    ambs_core
    ambs_msgs
    ambs_pluginlib
    geometry_msgs
    roscpp
    roslint
    std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    add_rostest_gtest(test_comp_float_param
        test/unit_tests/calculators/comparators/test_comp_float_param.test
        test/unit_tests/calculators/comparators/test_comp_float_param.cpp)
    target_link_libraries(test_comp_float_param ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
    add_dependencies(test_comp_float_param ${catkin_EXPORTED_TARGETS})

    add_rostest_gtest(test_diff_pose_temporal
        test/unit_tests/calculators/differentiators/test_diff_pose_temporal.test
        test/unit_tests/calculators/differentiators/test_diff_pose_temporal.cpp)
    target_link_libraries(test_diff_pose_temporal ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
    add_dependencies(test_diff_pose_temporal ${catkin_EXPORTED_TARGETS})

    add_rostest_gtest(test_stopwatch
        test/unit_tests/calculators/differentiators/test_stopwatch.test
        test/unit_tests/calculators/differentiators/test_stopwatch.cpp)
    target_link_libraries(test_stopwatch ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
    add_dependencies(test_stopwatch ${catkin_EXPORTED_TARGETS})

    add_rostest_gtest(test_diff_float_temporal
        test/unit_tests/calculators/differentiators/test_diff_float_temporal.test
        test/unit_tests/calculators/differentiators/test_diff_float_temporal.cpp)
    target_link_libraries(test_diff_float_temporal ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
    add_dependencies(test_diff_float_temporal ${catkin_EXPORTED_TARGETS})

    add_rostest_gtest(test_edge_detector
        test/unit_tests/calculators/comparators/test_edge_detector.test
        test/unit_tests/calculators/comparators/test_edge_detector.cpp)
    target_link_libraries(test_edge_detector ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
    add_dependencies(test_edge_detector ${catkin_EXPORTED_TARGETS})

    add_rostest_gtest(test_helper
        test/unit_tests/core/helper/test_helper.test
        test/unit_tests/core/helper/test_helper.cpp)
    target_link_libraries(test_helper ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
    add_dependencies(test_helper ${catkin_EXPORTED_TARGETS})

    add_rostest_gtest(test_signal_repeater
        test/unit_tests/calculators/utils/test_signal_repeater.test
        test/unit_tests/calculators/utils/test_signal_repeater.cpp)
    target_link_libraries(test_signal_repeater ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
    add_dependencies(test_signal_repeater ${catkin_EXPORTED_TARGETS})

endif()


set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
