cmake_minimum_required(VERSION 3.0.2)
project(ugv_modules)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rigid_state_estimator_msgs
  rosgraph_msgs
  roscpp
  std_msgs
  ugv_reset_client
  ugv_reset_safety
  unicycle_reference_trajectory
  unicycle_reference_trajectory_msgs
  unicycle_ugv_controller
)
# The module ABI header (xgc2/module.h) of the xgc2-module SDK.
find_package(Xgc2Module REQUIRED CONFIG)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
find_package(Eigen3 REQUIRED)
find_package(xgc2_state_machine REQUIRED CONFIG)
find_package(xgc2_math 0.5.12 REQUIRED CONFIG)

# The payload header is the public vocabulary of the modules.
catkin_package(INCLUDE_DIRS include)

# The two ROS-free modules link the cores by file and nothing else of the catkin dependencies:
# a ROS library on their link line would only be dragged in by the catkin variables.
find_library(UGV_REFERENCE_CORE unicycle_reference_trajectory_core
  PATHS ${unicycle_reference_trajectory_LIBRARY_DIRS} NO_DEFAULT_PATH)
find_library(UGV_CONTROLLER_CORE unicycle_ugv_controller_core
  PATHS ${unicycle_ugv_controller_LIBRARY_DIRS} NO_DEFAULT_PATH)
if(NOT UGV_REFERENCE_CORE OR NOT UGV_CONTROLLER_CORE)
  message(FATAL_ERROR "The reference and controller core libraries were not found")
endif()

# A module is a plug-in of the host: it exports xgc2_module_entry and nothing else, and every
# symbol it uses resolves at link time.
function(ugv_add_module target)
  add_library(${target} MODULE ${ARGN})
  set_target_properties(${target} PROPERTIES
    PREFIX "lib"
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
  target_compile_definitions(${target} PRIVATE UGV_MODULES_VERSION="${${PROJECT_NAME}_VERSION}")
  target_compile_options(${target} PRIVATE -Wall -Wextra)
  target_include_directories(${target} BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${JSONCPP_INCLUDE_DIRS})
  target_link_libraries(${target} PRIVATE Xgc2Module::SDK ${JSONCPP_LIBRARIES})
  set_property(TARGET ${target} APPEND_STRING PROPERTY LINK_FLAGS
    " -Wl,--no-undefined -Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/exports.map")
  install(TARGETS ${target} LIBRARY DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endfunction()

ugv_add_module(ugv_unicycle_reference src/ugv_unicycle_reference.cpp)
target_include_directories(ugv_unicycle_reference BEFORE PRIVATE
  ${unicycle_reference_trajectory_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(ugv_unicycle_reference PRIVATE
  ${UGV_REFERENCE_CORE}
  xgc2_math::trajectory
  xgc2_state_machine::state_machine
  pthread)

ugv_add_module(ugv_unicycle_controller src/ugv_unicycle_controller.cpp)
target_include_directories(ugv_unicycle_controller BEFORE PRIVATE
  ${unicycle_ugv_controller_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(ugv_unicycle_controller PRIVATE
  ${UGV_CONTROLLER_CORE}
  xgc2_math::control
  xgc2_math::trajectory
  xgc2_state_machine::state_machine
  pthread)

# The ROS edge is the one module that links ROS.
ugv_add_module(ugv_ros_edge src/ugv_ros_edge.cpp)
add_dependencies(ugv_ros_edge ${catkin_EXPORTED_TARGETS})
target_include_directories(ugv_ros_edge BEFORE PRIVATE
  ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(ugv_ros_edge PRIVATE ${catkin_LIBRARIES} pthread)
# roscpp cannot be restarted in a process and ends at process exit (RosShutdown in the module), so
# the library is never unmapped: unloading it, or replacing it by a new version, leaves ROS running.
set_property(TARGET ugv_ros_edge APPEND_STRING PROPERTY LINK_FLAGS " -Wl,-z,nodelete")

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  # A module test loads the module as a shared library through an in-test host.
  function(ugv_add_module_test target source module)
    catkin_add_gtest(${target} ${source})
    if(TARGET ${target})
      add_dependencies(${target} ${module})
      # further arguments: more modules the test loads, then its compile definitions
      set(_definitions)
      foreach(_argument ${ARGN})
        if(TARGET ${_argument})
          add_dependencies(${target} ${_argument})
        else()
          list(APPEND _definitions ${_argument})
        endif()
      endforeach()
      target_compile_definitions(${target} PRIVATE
        ${_definitions})
      target_include_directories(${target} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test)
      target_link_libraries(${target} Xgc2Module::SDK gtest_main dl pthread)
    endif()
  endfunction()

  ugv_add_module_test(reference_module_test test/reference_module_test.cpp
    ugv_unicycle_reference
    REFERENCE_MODULE_PATH="$<TARGET_FILE:ugv_unicycle_reference>")
  ugv_add_module_test(controller_module_test test/controller_module_test.cpp
    ugv_unicycle_controller
    CONTROLLER_MODULE_PATH="$<TARGET_FILE:ugv_unicycle_controller>")

  # The module chain against the chain of the two ROS nodes, on the replay scenario of the generator.
  ugv_add_module_test(chain_parity_test test/chain_parity_test.cpp
    ugv_unicycle_reference ugv_unicycle_controller
    REFERENCE_MODULE_PATH="$<TARGET_FILE:ugv_unicycle_reference>"
    CONTROLLER_MODULE_PATH="$<TARGET_FILE:ugv_unicycle_controller>")
  if(TARGET chain_parity_test)
    target_include_directories(chain_parity_test PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/../unicycle_reference_trajectory/test/replay
      ${unicycle_reference_trajectory_INCLUDE_DIRS} ${unicycle_ugv_controller_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(chain_parity_test
      ${UGV_REFERENCE_CORE} ${UGV_CONTROLLER_CORE}
      xgc2_math::control xgc2_math::trajectory xgc2_state_machine::state_machine)
  endif()

  # The ROS edge against the ROS master that rostest starts for the test (never the default master),
  # and one without a master.
  find_package(rostest REQUIRED)
  function(ugv_add_edge_rostest target launch source)
    add_rostest_gtest(${target} ${launch} ${source})
    if(TARGET ${target})
      add_dependencies(${target} ugv_ros_edge ${catkin_EXPORTED_TARGETS})
      target_compile_definitions(${target} PRIVATE EDGE_MODULE_PATH="$<TARGET_FILE:ugv_ros_edge>")
      target_include_directories(${target} PRIVATE
        ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/test ${EIGEN3_INCLUDE_DIRS})
      target_link_libraries(${target} Xgc2Module::SDK ${catkin_LIBRARIES} dl pthread)
    endif()
  endfunction()
  ugv_add_edge_rostest(ros_edge_test test/ros_edge.test test/ros_edge_test.cpp)
  ugv_add_edge_rostest(ros_edge_sim_test test/ros_edge_sim.test test/ros_edge_sim_test.cpp)
  ugv_add_module_test(ros_edge_no_master_test test/ros_edge_no_master_test.cpp
    ugv_ros_edge
    EDGE_MODULE_PATH="$<TARGET_FILE:ugv_ros_edge>")
endif()
