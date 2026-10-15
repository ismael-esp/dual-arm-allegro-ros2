cmake_minimum_required(VERSION 3.8)
project(allegro_hand_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ContactPointArray.msg"
  "msg/ContactPoints.msg"
  "msg/CurrentState.msg"
  "msg/Trajectory.msg"
  "srv/GetController.srv"
  "srv/GetFingersMovementStatus.srv"
  "srv/GetGravityVector.srv"
  "srv/GetHandConfiguration.srv"
  "srv/GetPoseFK.srv"
  "srv/GetTorque.srv"
  "srv/JointLimits.srv"
  "srv/CoupledClosing.srv"
  "srv/SetController.srv"
  "srv/SetDesPos.srv"
  "srv/SetDesPosReflexxes.srv"
  "srv/SetDesPosVel.srv"
  "srv/SetDesTorque.srv"
  "srv/SetGravityVector.srv"
  "srv/SetPoseIK.srv"
  "srv/StopHand.srv"
  "srv/PauseFinger.srv"
  "srv/ContinueFinger.srv"
  "srv/StopFinger.srv"
  "action/MoveConfig.action"
  "action/MoveConfigFinger.action"
  "action/MoveFingertipPose.action"
  "action/MoveFingertipPoses.action"
  "action/MoveTrajectory.action"
  "action/MoveTrajectoryVel.action"
  DEPENDENCIES geometry_msgs trajectory_msgs sensor_msgs # .msg file names have to start with uppercase in ros2
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
