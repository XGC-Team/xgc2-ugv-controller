cmake_minimum_required(VERSION 3.0.2)
project(mecanum_ugv_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp std_msgs ugv_reset_safety)
find_package(xgc2_state_machine REQUIRED CONFIG)
find_package(xgc2_math REQUIRED CONFIG)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mecanum_ugv_controller_core
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs ugv_reset_safety
)

include_directories(include ${catkin_INCLUDE_DIRS})

# ROS-free controller core: the controller, its state machine states and the
# core log. It takes plain types and explicit time; ROS stays in the node.
#
# The directory-level include path (catkin, which puts the ROS headers on it)
# is reset to the core's real dependencies: this package's headers and the
# header-only ugv_reset_safety session. It links no ROS library and is linked
# with -Wl,--no-undefined, so a ROS symbol reaching the core fails the link.
find_path(MECANUM_RESET_SAFETY_CORE_INCLUDE_DIR ugv_reset_safety/reset_session.h
  PATHS ${catkin_INCLUDE_DIRS} NO_DEFAULT_PATH)
if(NOT MECANUM_RESET_SAFETY_CORE_INCLUDE_DIR)
  message(FATAL_ERROR "ugv_reset_safety/reset_session.h not found")
endif()
add_library(mecanum_ugv_controller_core
  src/common_types.cpp
  src/core_log.cpp
  src/mecanum_ugv_controller.cpp
  src/state_machine/states.cpp)
set_property(TARGET mecanum_ugv_controller_core PROPERTY INCLUDE_DIRECTORIES "")
target_include_directories(mecanum_ugv_controller_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${MECANUM_RESET_SAFETY_CORE_INCLUDE_DIR})
target_link_libraries(mecanum_ugv_controller_core
  xgc2_state_machine::state_machine
  xgc2_math::algebra
  pthread)
set_property(TARGET mecanum_ugv_controller_core APPEND_STRING PROPERTY LINK_FLAGS " -Wl,--no-undefined")

add_executable(${PROJECT_NAME}_node
  src/mecanum_ugv_ros_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  mecanum_ugv_controller_core
  ${catkin_LIBRARIES}
  pthread)

install(TARGETS mecanum_ugv_controller_core ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY tools
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(mecanum_ugv_controller_runtime_test
    test/mecanum_ugv_controller_runtime_test.cpp)
  if(TARGET mecanum_ugv_controller_runtime_test)
    target_link_libraries(mecanum_ugv_controller_runtime_test
      mecanum_ugv_controller_core
      ${catkin_LIBRARIES}
      gtest_main)
  endif()
  # Deterministic closed-loop replay of the controller core (test/replay/): the
  # refactor gate. Two builds are compared with `cmp` on its output. It has no
  # ROS on its include path or link line, so it also keeps the core usable
  # without ROS.
  add_executable(mecanum_ugv_controller_replay test/replay/mecanum_replay.cpp)
  set_property(TARGET mecanum_ugv_controller_replay PROPERTY INCLUDE_DIRECTORIES "")
  target_link_libraries(mecanum_ugv_controller_replay mecanum_ugv_controller_core)
  set_property(TARGET mecanum_ugv_controller_replay APPEND_STRING PROPERTY LINK_FLAGS " -Wl,--no-undefined")
  find_package(rostest QUIET)
  if(rostest_FOUND)
    add_rostest_gtest(mecanum_ugv_ros_sandbox_test
      test/mecanum_ugv_ros_sandbox.test
      test/mecanum_ugv_ros_sandbox_test.cpp)
    if(TARGET mecanum_ugv_ros_sandbox_test)
      target_link_libraries(mecanum_ugv_ros_sandbox_test ${catkin_LIBRARIES})
      add_dependencies(mecanum_ugv_ros_sandbox_test ${PROJECT_NAME}_node)
    endif()
    add_rostest_gtest(mecanum_ugv_public_command_reset_test
      test/mecanum_ugv_public_command_reset.test
      test/mecanum_ugv_public_command_reset_test.cpp)
    if(TARGET mecanum_ugv_public_command_reset_test)
      target_link_libraries(mecanum_ugv_public_command_reset_test ${catkin_LIBRARIES})
      add_dependencies(mecanum_ugv_public_command_reset_test ${PROJECT_NAME}_node)
    endif()
  endif()
  find_program(PYTHON3_EXECUTABLE python3)
  if(PYTHON3_EXECUTABLE)
    add_test(NAME holonomic_tracker_python
      COMMAND ${PYTHON3_EXECUTABLE} -m unittest discover -s
              ${CMAKE_CURRENT_SOURCE_DIR}/tools/holonomic_tracker/tests -p "test_*.py")
    set_tests_properties(holonomic_tracker_python PROPERTIES
      ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/tools")
  endif()
endif()
