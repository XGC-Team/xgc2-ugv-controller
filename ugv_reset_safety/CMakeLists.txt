cmake_minimum_required(VERSION 3.0.2)
project(ugv_reset_safety)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(catkin REQUIRED COMPONENTS geometry_msgs std_msgs roscpp ugv_reset_client ugv_reset_msgs xgc2_geometry_msgs)
find_package(Eigen3 REQUIRED)
find_package(xgc2_math CONFIG REQUIRED)
# Eigen's config exports uppercase variables, while catkin follows the spelling
# in DEPENDS when exporting public includes to CMake and pkg-config consumers.
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
catkin_package(INCLUDE_DIRS include LIBRARIES ugv_reset_safety_math CATKIN_DEPENDS geometry_msgs std_msgs roscpp ugv_reset_client ugv_reset_msgs xgc2_geometry_msgs DEPENDS Eigen3)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
# The station's Reset coordinator and its ROS-free core: the reservation planner (exact swept-footprint
# verification), the visibility-graph route and the event-driven coordinator; scene_projection.h and
# the node are the ROS edge. The vehicle side of the protocol is ugv_reset_client; the shared message
# definitions are in ugv_reset_msgs.
# The math library's include path is reset from the catkin directory path (which
# puts the ROS headers on it) to this package's headers and Eigen; it links no
# ROS library and is linked with -Wl,--no-undefined, so a ROS symbol reaching it
# fails the link.
add_library(ugv_reset_safety_math src/reset_path.cpp src/reset_planner.cpp src/reset_coordinator.cpp)
set_property(TARGET ugv_reset_safety_math PROPERTY INCLUDE_DIRECTORIES "")
target_include_directories(ugv_reset_safety_math PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${ugv_reset_client_INCLUDE_DIRS})
target_link_libraries(ugv_reset_safety_math PUBLIC Eigen3::Eigen)
set_property(TARGET ugv_reset_safety_math APPEND_STRING PROPERTY LINK_FLAGS " -Wl,--no-undefined")
add_executable(ugv_reset_coordinator_node src/reset_coordinator_node.cpp)
add_dependencies(ugv_reset_coordinator_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ugv_reset_coordinator_node ugv_reset_safety_math xgc2_math::geometry ${catkin_LIBRARIES})
install(TARGETS ugv_reset_safety_math ugv_reset_coordinator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  foreach(case scene_projection reset_path reset_planner reset_coordinator)
    catkin_add_gtest(${case}_test test/${case}_test.cpp)
    if(TARGET ${case}_test)
      add_dependencies(${case}_test ${catkin_EXPORTED_TARGETS})
      target_link_libraries(${case}_test ugv_reset_safety_math ${catkin_LIBRARIES} gtest_main)
      target_compile_definitions(${case}_test PRIVATE RESET_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
      if(case STREQUAL "scene_projection")
        target_link_libraries(${case}_test xgc2_math::geometry)
      endif()
    endif()
  endforeach()
endif()
