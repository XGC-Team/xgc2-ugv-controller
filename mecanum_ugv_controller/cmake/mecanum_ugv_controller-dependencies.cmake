include(CMakeFindDependencyMacro)

find_dependency(xgc2_state_machine 0.2.0 CONFIG)
find_dependency(xgc2_math 0.5.12 CONFIG)
list(APPEND mecanum_ugv_controller_LIBRARIES
  xgc2_state_machine::state_machine
  xgc2_math::algebra)
list(REMOVE_DUPLICATES mecanum_ugv_controller_LIBRARIES)
