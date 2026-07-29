include(CMakeFindDependencyMacro)

find_dependency(Eigen3 CONFIG)
find_dependency(xgc2_math CONFIG)
find_dependency(xgc2_state_machine CONFIG)

list(APPEND unicycle_reference_trajectory_LIBRARIES
  Eigen3::Eigen
  xgc2_math::trajectory
  xgc2_state_machine::state_machine
)
list(REMOVE_DUPLICATES unicycle_reference_trajectory_LIBRARIES)
