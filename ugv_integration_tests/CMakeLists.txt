cmake_minimum_required(VERSION 3.0.2)
project(ugv_integration_tests)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(catkin REQUIRED OPTIONAL_COMPONENTS ugv_reset_safety ugv_reset_client)
catkin_package()
# ROS-level tests that run packages of this repository against each other under
# the isolated rostest master: the station's Reset coordinator with the vehicle
# controller nodes, or the module entity, and a synthetic plant. Nothing here is installed or shipped.
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # The closed-loop scenario harness: the real coordinator and executors on a physical plant, no ROS.
  catkin_add_gtest(reset_scenario_harness_test test/reset_scenario_harness_test.cpp)
  if(TARGET reset_scenario_harness_test)
    target_include_directories(reset_scenario_harness_test PRIVATE ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
    target_compile_definitions(reset_scenario_harness_test PRIVATE
      RESET_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../ugv_reset_safety/test/data")
    target_link_libraries(reset_scenario_harness_test ugv_reset_safety_math gtest_main)
  endif()
  add_rostest(test/reset_coordinator.test)
  add_rostest(test/reset_coordinator_scout.test)
  add_rostest(test/reset_coordinator_open.test)
  add_rostest(test/reset_two_vehicles.test)

  # The tests of the module entity run xgc2-module-host: $XGC2_MODULE_HOST, else the one on the PATH.
  # Without a host binary they are not registered, and this says so.
  if(DEFINED ENV{XGC2_MODULE_HOST} AND EXISTS "$ENV{XGC2_MODULE_HOST}")
    set(XGC2_MODULE_HOST_BINARY "$ENV{XGC2_MODULE_HOST}")
  else()
    find_program(XGC2_MODULE_HOST_BINARY xgc2-module-host)
  endif()
  if(XGC2_MODULE_HOST_BINARY)
    message(STATUS "ugv_integration_tests: module host ${XGC2_MODULE_HOST_BINARY}")
    add_rostest(test/reset_coordinator_scout_module.test ARGS host:=${XGC2_MODULE_HOST_BINARY})
    add_rostest(test/scout_module_chain.test ARGS host:=${XGC2_MODULE_HOST_BINARY})
    add_rostest(test/scout_module_chain_sim.test ARGS host:=${XGC2_MODULE_HOST_BINARY})
  else()
    message(STATUS "ugv_integration_tests: xgc2-module-host not found (set XGC2_MODULE_HOST): "
                   "the tests of the module entity are not registered")
  endif()
endif()
