cmake_minimum_required(VERSION 3.0.2)
project(unicycle_reference_trajectory)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  unicycle_reference_trajectory_msgs
)
find_package(xgc2_state_machine REQUIRED CONFIG)
find_package(xgc2_math 0.5.12 REQUIRED CONFIG)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES unicycle_reference_trajectory_core
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs unicycle_reference_trajectory_msgs
  CFG_EXTRAS unicycle_reference_trajectory-dependencies.cmake
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## The ROS-free runtime: the reference trajectory state machine, its trajectory
## models, the plain request/status types and the driver that accepts requests
## into the runtime. The library gets no ROS include directory and links no
## ROS library; -Wl,--no-undefined makes any ROS symbol it used fail the
## link. The node (the ROS edge), the module, the tests and the replay link it.
add_library(unicycle_reference_trajectory_core
  src/unicycle_reference_trajectory_runtime.cpp
  src/reference_driver.cpp
  src/default_analytic.cpp
  src/state_machine/self_check_state.cpp
  src/state_machine/ready_state.cpp
  src/state_machine/planning_state.cpp
  src/state_machine/active_state.cpp
)
set_target_properties(unicycle_reference_trajectory_core PROPERTIES INCLUDE_DIRECTORIES "")
target_include_directories(unicycle_reference_trajectory_core BEFORE PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(unicycle_reference_trajectory_core
  xgc2_math::trajectory
  xgc2_state_machine::state_machine
  pthread
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set_property(TARGET unicycle_reference_trajectory_core APPEND_STRING PROPERTY LINK_FLAGS " -Wl,--no-undefined")
endif()

add_executable(unicycle_reference_trajectory_node
  src/unicycle_reference_trajectory_main.cpp
  src/unicycle_reference_trajectory_node.cpp
  src/input/reference_input_producer.cpp
  src/output/reference_output_consumer.cpp
)
add_dependencies(unicycle_reference_trajectory_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                      ${catkin_EXPORTED_TARGETS})
target_link_libraries(unicycle_reference_trajectory_node
  unicycle_reference_trajectory_core
  xgc2_math::trajectory
  xgc2_state_machine::state_machine
  ${catkin_LIBRARIES}
  pthread
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS unicycle_reference_trajectory_core unicycle_reference_trajectory_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(unicycle_reference_trajectory_unit_test
    test/unicycle_reference_trajectory_unit_test.cpp
  )
  target_link_libraries(unicycle_reference_trajectory_unit_test
    xgc2_math::trajectory
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(unicycle_reference_trajectory_smoke_test
    test/unicycle_reference_trajectory_smoke_test.cpp
  )
  target_link_libraries(unicycle_reference_trajectory_smoke_test
    xgc2_math::trajectory
    ${catkin_LIBRARIES}
  )

  # The runtime and the driver through the ROS-free core only.
  catkin_add_gtest(unicycle_reference_trajectory_runtime_test
    test/unicycle_reference_trajectory_runtime_test.cpp
  )
  target_link_libraries(unicycle_reference_trajectory_runtime_test
    unicycle_reference_trajectory_core
    xgc2_math::trajectory
    xgc2_state_machine::state_machine
    pthread
  )

  # The ROS conversions are bit-exact copies of the messages and back.
  catkin_add_gtest(unicycle_reference_trajectory_conversion_test
    test/unicycle_reference_trajectory_conversion_test.cpp
  )
  add_dependencies(unicycle_reference_trajectory_conversion_test
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(unicycle_reference_trajectory_conversion_test
    unicycle_reference_trajectory_core
    ${catkin_LIBRARIES}
  )

  # The node over the core, on an isolated master.
  find_package(rostest QUIET)
  if(rostest_FOUND)
    add_rostest_gtest(unicycle_reference_trajectory_node_test
      test/unicycle_reference_trajectory_node.test
      test/unicycle_reference_trajectory_node_test.cpp)
    if(TARGET unicycle_reference_trajectory_node_test)
      target_link_libraries(unicycle_reference_trajectory_node_test ${catkin_LIBRARIES})
      add_dependencies(unicycle_reference_trajectory_node_test unicycle_reference_trajectory_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    endif()
  endif()

  # Deterministic replay of the reference runtime (test/replay): the trace is
  # compared with the recorded hash of the original ROS runtime. The replay has
  # no ROS on its include path or link line, so it also keeps the core usable
  # without ROS. The program writes the trace for `cmp`; the gtest checks it.
  add_executable(unicycle_reference_trajectory_replay test/replay/reference_replay.cpp)
  set_property(TARGET unicycle_reference_trajectory_replay PROPERTY INCLUDE_DIRECTORIES "")
  target_include_directories(unicycle_reference_trajectory_replay PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test/replay)
  target_link_libraries(unicycle_reference_trajectory_replay unicycle_reference_trajectory_core)
  set_property(TARGET unicycle_reference_trajectory_replay APPEND_STRING PROPERTY LINK_FLAGS " -Wl,--no-undefined")

  catkin_add_gtest(unicycle_reference_trajectory_replay_test
    test/unicycle_reference_trajectory_replay_test.cpp
  )
  target_include_directories(unicycle_reference_trajectory_replay_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test ${CMAKE_CURRENT_SOURCE_DIR}/test/replay)
  target_compile_definitions(unicycle_reference_trajectory_replay_test PRIVATE
    REPLAY_SHA256_FILE="${CMAKE_CURRENT_SOURCE_DIR}/test/replay/reference_replay.sha256")
  target_link_libraries(unicycle_reference_trajectory_replay_test
    unicycle_reference_trajectory_core
  )
endif()
