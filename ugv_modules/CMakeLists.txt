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

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  # A module test loads the module as a shared library through an in-test host.
  function(ugv_add_module_test target source module)
    catkin_add_gtest(${target} ${source})
    if(TARGET ${target})
      add_dependencies(${target} ${module})
      target_compile_definitions(${target} PRIVATE
        ${ARGN})
      target_include_directories(${target} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test)
      target_link_libraries(${target} Xgc2Module::SDK gtest_main dl pthread)
    endif()
  endfunction()

  ugv_add_module_test(reference_module_test test/reference_module_test.cpp
    ugv_unicycle_reference
    REFERENCE_MODULE_PATH="$<TARGET_FILE:ugv_unicycle_reference>")
endif()
