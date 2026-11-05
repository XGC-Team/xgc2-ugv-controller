include(CMakeFindDependencyMacro)

find_dependency(Eigen3 CONFIG)
find_dependency(xgc2_acados CONFIG)
find_dependency(xgc2_math 0.5.12 CONFIG)
find_dependency(xgc2_state_machine 0.2.0 CONFIG COMPONENTS workers)

list(APPEND unicycle_ugv_controller_INCLUDE_DIRS
  ${XGC2_ACADOS_INCLUDE_DIRS}
)
list(APPEND unicycle_ugv_controller_LIBRARIES
  Eigen3::Eigen
  ${XGC2_ACADOS_LIBRARIES}
  xgc2_math::control
  xgc2_math::trajectory
  xgc2_state_machine::workers
)
list(REMOVE_DUPLICATES unicycle_ugv_controller_INCLUDE_DIRS)
list(REMOVE_DUPLICATES unicycle_ugv_controller_LIBRARIES)
