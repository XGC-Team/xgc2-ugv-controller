include(CMakeFindDependencyMacro)

find_dependency(Eigen3 CONFIG)
find_dependency(xgc2_math 0.5.12 CONFIG)
find_dependency(xgc2_state_machine 0.2.0 CONFIG)

list(APPEND unicycle_reference_trajectory_LIBRARIES
  Eigen3::Eigen
  xgc2_math::trajectory
  xgc2_state_machine::state_machine
)
list(REMOVE_DUPLICATES unicycle_reference_trajectory_LIBRARIES)
