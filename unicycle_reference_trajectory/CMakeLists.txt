cmake_minimum_required(VERSION 3.0.2)
project(unicycle_reference_trajectory)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)
find_package(xgc2_state_machine REQUIRED CONFIG)
find_package(xgc2_math REQUIRED CONFIG)

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  ActivePolynomialReference.msg
  AnalyticReference.msg
  PlanarReferencePoint.msg
  ReferenceStatus.msg
  SampledReference.msg
  WaypointReferenceRequest.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs message_runtime roscpp rospy std_msgs
  DEPENDS EIGEN3 xgc2_math
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(unicycle_reference_trajectory_node
  src/unicycle_reference_trajectory_main.cpp
  src/unicycle_reference_trajectory_node.cpp
  src/unicycle_reference_trajectory_runtime.cpp
  src/input/reference_input_producer.cpp
  src/output/reference_output_consumer.cpp
  src/state_machine/self_check_state.cpp
  src/state_machine/ready_state.cpp
  src/state_machine/planning_state.cpp
  src/state_machine/active_state.cpp
  src/state_machine/fault_state.cpp
)
add_dependencies(unicycle_reference_trajectory_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                      ${catkin_EXPORTED_TARGETS})
target_link_libraries(unicycle_reference_trajectory_node
  xgc2_math::trajectory
  xgc2_state_machine::state_machine
  ${catkin_LIBRARIES}
  pthread
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS unicycle_reference_trajectory_node
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
endif()
