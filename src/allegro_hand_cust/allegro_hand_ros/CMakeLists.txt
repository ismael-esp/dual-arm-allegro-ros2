cmake_minimum_required(VERSION 3.8)
project(allegro_hand_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED) #To find other packages (used for reading the xacro)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(allegro_hand_interfaces REQUIRED)
find_package(allegro_hand REQUIRED)
find_package(urdf REQUIRED)
find_package(control_msgs REQUIRED)

set(THIS_PACKAGE_DEPS allegro_hand)

include_directories(include)
# Define a library target:
add_library(allegro_ros_library SHARED ./src/allegro_hand_ros.cpp ./src/move_actions.cpp)
ament_target_dependencies(allegro_ros_library ament_index_cpp rclcpp rclcpp_action std_msgs std_srvs sensor_msgs geometry_msgs trajectory_msgs control_msgs tf2 tf2_ros tf2_kdl tf2_geometry_msgs allegro_hand_interfaces allegro_hand urdf rclcpp_action rclcpp_components)
ament_export_dependencies(ament_index_cpp rclcpp rclcpp_action std_msgs std_srvs sensor_msgs geometry_msgs trajectory_msgs control_msgs tf2 tf2_ros tf2_kdl tf2_geometry_msgs allegro_hand_interfaces allegro_hand urdf rclcpp_action rclcpp_components)
ament_export_targets(allegro_ros_library HAS_LIBRARY_TARGET) 
 
# add_executable(node_allegro src/node_allegro.cpp)
# ament_target_dependencies(node_allegro rclcpp std_msgs std_srvs allegro_hand_interfaces allegro_hand)
# add_executable(allegro_hand_ros2 src/allegro_hand_ros.cpp)
# ament_target_dependencies(allegro_hand_ros2 rclcpp std_msgs std_srvs allegro_hand_interfaces allegro_hand)

add_executable(allegro_node ./src/allegro_hand_ros.cpp ./src/node_allegro.cpp ./src/move_actions.cpp)
ament_target_dependencies(allegro_node rclcpp rclcpp_action std_msgs std_srvs sensor_msgs geometry_msgs trajectory_msgs control_msgs tf2 tf2_ros tf2_kdl tf2_geometry_msgs allegro_hand_interfaces allegro_hand urdf ament_index_cpp)

# Action clients
add_executable(move_config_client ./test/move_config_client.cpp)
ament_target_dependencies(move_config_client rclcpp rclcpp_action rclcpp_components std_msgs std_srvs sensor_msgs geometry_msgs trajectory_msgs tf2 tf2_ros tf2_kdl tf2_geometry_msgs allegro_hand_interfaces allegro_hand urdf)
add_executable(move_poses_client ./test/move_poses_client.cpp)
ament_target_dependencies(move_poses_client rclcpp rclcpp_action rclcpp_components std_msgs std_srvs sensor_msgs geometry_msgs trajectory_msgs tf2 tf2_ros tf2_kdl tf2_geometry_msgs allegro_hand_interfaces allegro_hand urdf)
add_executable(move_trajectory_client ./test/move_trajectory_client.cpp)
ament_target_dependencies(move_trajectory_client rclcpp rclcpp_action rclcpp_components std_msgs std_srvs sensor_msgs geometry_msgs trajectory_msgs tf2 tf2_ros tf2_kdl tf2_geometry_msgs allegro_hand_interfaces allegro_hand urdf)
add_executable(move_trajectory_vel_client ./test/move_trajectory_vel_client.cpp)
ament_target_dependencies(move_trajectory_vel_client rclcpp rclcpp_action rclcpp_components std_msgs std_srvs sensor_msgs geometry_msgs trajectory_msgs tf2 tf2_ros tf2_kdl tf2_geometry_msgs allegro_hand_interfaces allegro_hand urdf)
add_executable(move_config_finger_client ./test/move_config_finger_client.cpp)
ament_target_dependencies(move_config_finger_client rclcpp rclcpp_action rclcpp_components std_msgs std_srvs sensor_msgs geometry_msgs trajectory_msgs tf2 tf2_ros tf2_kdl tf2_geometry_msgs allegro_hand_interfaces allegro_hand urdf)
add_executable(move_one_pose_client ./test/move_pose_client.cpp)
ament_target_dependencies(move_one_pose_client rclcpp rclcpp_action rclcpp_components std_msgs std_srvs sensor_msgs geometry_msgs trajectory_msgs tf2 tf2_ros tf2_kdl tf2_geometry_msgs allegro_hand_interfaces allegro_hand urdf)
add_executable(follow_joint_trajectory_client ./test/follow_joint_trajectory_client.cpp)
ament_target_dependencies(follow_joint_trajectory_client rclcpp rclcpp_action rclcpp_components std_msgs std_srvs sensor_msgs geometry_msgs trajectory_msgs control_msgs tf2 tf2_ros tf2_kdl tf2_geometry_msgs allegro_hand_interfaces allegro_hand urdf)
# Service client
add_executable(service_client ./test/service_client.cpp)
ament_target_dependencies(service_client rclcpp rclcpp_action rclcpp_components std_msgs std_srvs sensor_msgs geometry_msgs trajectory_msgs tf2 tf2_ros tf2_kdl tf2_geometry_msgs allegro_hand_interfaces allegro_hand urdf)
# # Allegro test all
# add_executable(test_allegro ./test/allegro_test.cpp)
# ament_target_dependencies(test_allegro rclcpp rclcpp_action rclcpp_components std_msgs std_srvs sensor_msgs geometry_msgs trajectory_msgs tf2 tf2_ros tf2_kdl tf2_geometry_msgs allegro_hand_interfaces allegro_hand urdf)


install(TARGETS
  allegro_node
  move_config_client
  move_poses_client
  move_trajectory_client
  move_trajectory_vel_client
  move_config_finger_client
  move_one_pose_client
  follow_joint_trajectory_client
  service_client
  # test_allegro
  # node_allegro
  # allegro_hand_ros2
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

 install(
  TARGETS allegro_ros_library
  EXPORT allegro_ros_library
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
 )

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/glove_mapper.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/hand_toggle.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/dual_service_bridge.py
  DESTINATION lib/${PROJECT_NAME}
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
