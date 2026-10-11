cmake_minimum_required(VERSION 3.0.2)
project(ugv_integration_tests)
find_package(catkin REQUIRED)
catkin_package()
# ROS-level tests that run packages of this repository against each other under
# the isolated rostest master: the station's Reset coordinator with the vehicle
# controller nodes, or the module entity, and a synthetic plant. Nothing here is installed or shipped.
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
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
