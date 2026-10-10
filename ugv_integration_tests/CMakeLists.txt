cmake_minimum_required(VERSION 3.0.2)
project(ugv_integration_tests)
find_package(catkin REQUIRED)
catkin_package()
# ROS-level tests that run packages of this repository against each other under
# the isolated rostest master: the station's Reset coordinator with the vehicle
# controller nodes and a synthetic plant. Nothing here is installed or shipped.
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/reset_coordinator.test)
  add_rostest(test/reset_coordinator_scout.test)
  add_rostest(test/reset_coordinator_open.test)
  add_rostest(test/reset_two_vehicles.test)
endif()
