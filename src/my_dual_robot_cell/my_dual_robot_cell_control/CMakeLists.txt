cmake_minimum_required(VERSION 3.8)
project(my_dual_robot_cell_control)

find_package(ament_cmake REQUIRED)

# Install launch/config/etc. (no need to install your python package dir into share/)
install(
  DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install each node as a real executable in lib/<pkg> (script file -> executable name).
# Scripts that are not present in this checkout are skipped, so a trimmed copy of the
# package still builds.
set(NODE_SCRIPTS
  joint_state_merger joint_state_merger
  joint_state_merger_new joint_state_merger_new
  dual_ik_commander dual_ik_commander
  dual_task_runner dual_task_runner
  ref_to_world_relay ref_to_world_relay
  dual_ik_teleop dual_ik_teleop
  dual_ik_teleop_v2 dual_ik_teleop_v2
  dual_ik_teleop_v3 dual_ik_teleop_v3
  dual_ik_teleop_v4 dual_ik_teleop_v4
  la_pap_task la_pap_task
  pap_task_v1 pap_task_v1
  pap_task_v2 pap_task_v2
  pap_task_v3 pap_task_v3
  bridge_hardware_sim bridge_hardware_sim
  hard_demo_record hard_demo_record
  simu_teleop simu_teleop
  hand_service_commander_left hand_service_commander_left
  hand_service_commander_right hand_service_commander_right
  hand_service_commander_left_low_effort hand_service_commander_left_low_effort
  hand_service_commander_right_low_effort hand_service_commander_right_low_effort
  new_driver_real new_driver_real
  home_arms home_arms
  ur_joint_states_republisher_left jointrepub_left
  ur_joint_states_republisher_right jointrepub_right
  test_hardware_connection test_hardware_connection
  test_right_hand test_right_hand
  test_left_hand test_left_hand
  test_grasp_goals test_grasp_goals
  dexgrasp_pap dexgrasp_pap
  thumb_gravity_test thumb_gravity_test
)
list(LENGTH NODE_SCRIPTS _n)
math(EXPR _last "${_n} - 1")
foreach(_i RANGE 0 ${_last} 2)
  math(EXPR _j "${_i} + 1")
  list(GET NODE_SCRIPTS ${_i} _script)
  list(GET NODE_SCRIPTS ${_j} _exe)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/${_script}.py)
    install(PROGRAMS scripts/${_script}.py DESTINATION lib/${PROJECT_NAME} RENAME ${_exe})
  endif()
endforeach()

ament_package()
