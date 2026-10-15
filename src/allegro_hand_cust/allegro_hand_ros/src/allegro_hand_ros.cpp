#include "geometry_msgs/msg/twist.hpp"

#include "std_srvs/srv/set_bool.hpp"

#include "allegro_hand_ros/allegro_hand_ros.hpp"

/**
 * @brief Constructor Default constructor
 */
AllegroRos::AllegroRos(rclcpp::Node::SharedPtr _node, const std::string& _prefix)
: AllegroHand(),
  node_(_node), 
  prefix(_prefix),
  //rate(1000000) //1ms
  rate(1000)
  {
  
  // Parameters variables
  std::string pcan;
  std::string yaml_file;
  int channel;
  bool fake_hand;

  // Define and read slave parameter
  auto slave_desc = rcl_interfaces::msg::ParameterDescriptor{};
  slave_desc.description = "Slave parameter: true or false";
  node_->declare_parameter("slave", false, slave_desc);
  node_->get_parameter("slave", slave);

  // Name and namespace based on slave or not
  handed = node_->get_name();
  name_space_node = node_->get_namespace();
  RCLCPP_INFO_STREAM(node_->get_logger(), "Starting AllegroRos object initialization with handed:= " << handed);
  if(!slave)
  {
    name_space = "/" + handed + "/";
  }
  else
  {
    name_space = name_space_node +"/"+handed+"/";
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "with this " << name_space << " ***** " << std::endl);

  /* Handling parameters*/
  // COMMUNICATIONS PARAMETERS
  // Pcan parameter 
  auto pcan_desc = rcl_interfaces::msg::ParameterDescriptor{};
  pcan_desc.description = "Pcan path parameter";
  node_->declare_parameter("path_pcan", "/dev/pcan", pcan_desc);
  pcan = node_->get_parameter("path_pcan").as_string();
    
  // Channel parameter
  auto channel_desc = rcl_interfaces::msg::ParameterDescriptor{};
  channel_desc.description = "Communications channel parameter";
  node_->declare_parameter("channel", 0, channel_desc);
  channel = node_->get_parameter("channel").as_int();

  RCLCPP_INFO_STREAM(node_->get_logger(), "Pcan path: " << pcan << ", channel: " << channel << std::endl);

  // Allegro Hand configuration file: The yaml file parameter
  auto yaml_file_desc = rcl_interfaces::msg::ParameterDescriptor{};
  yaml_file_desc.description = "Path to the Allegro Hand yaml configuration file";
  node_->declare_parameter("yaml_file", "src/allegro_hand/conf/configuration_files/V4R284.yaml");
  yaml_file = node_->get_parameter("yaml_file").as_string();
  RCLCPP_INFO_STREAM(node_->get_logger(), "***** " << yaml_file << " ***** " << std::endl);
 
  // Fake hand parameter
  auto fake_hand_desc = rcl_interfaces::msg::ParameterDescriptor{};
  fake_hand_desc.description = "Fake hand parameter: true (simulated hand), false (real hand)";
  node_->declare_parameter("fake_hand", false, fake_hand_desc);
  node_->get_parameter("fake_hand", fake_hand);
  RCLCPP_INFO_STREAM(node_->get_logger(), "Fake hand: " << fake_hand);


  // Initial controller parameter
  auto initial_controller_desc = rcl_interfaces::msg::ParameterDescriptor{};
  initial_controller_desc.description = "Initial controller of the hand";
  node_->declare_parameter("initial_controller", "GravityCompensator", initial_controller_desc);
  std::string initial_controller = node_->get_parameter("initial_controller").as_string();
  RCLCPP_INFO_STREAM(node_->get_logger(), "Initial controller set to: " << initial_controller);

  // Parameters related to the Xacro
  
  // Joint prefix parameter
  auto joint_prefix_desc = rcl_interfaces::msg::ParameterDescriptor{};
  joint_prefix_desc.description = "joint_prefix parameter";
  node_->declare_parameter("joint_prefix", "", joint_prefix_desc);
  joint_prefix = node_->get_parameter("joint_prefix").as_string();
  RCLCPP_INFO_STREAM(node_->get_logger(), "joint_prefix: " << joint_prefix);
  // if(joint_prefix == "/"){joint_prefix = "";}
  // Link link_prefix parameter
  // auto link_prefix_desc = rcl_interfaces::msg::ParameterDescriptor{};
  // link_prefix_desc.description = "Link link_prefix parameter";
  // node_->declare_parameter("link_prefix", "", link_prefix_desc);
  // link_prefix = node_->get_parameter("link_prefix").as_string();
  // RCLCPP_INFO_STREAM(node_->get_logger(), "Link link_prefix: " << link_prefix);
  // // TODO: Separar link_prefix de prefix
  // if(link_prefix == "/"){link_prefix = "";}
  // END TODO: Borrar

  // if (prefix.empty()) {
  //   RCLCPP_WARN_STREAM(this->get_logger(), "No prefix provided");
  // }

  // Tip parameter
  auto tip_desc = rcl_interfaces::msg::ParameterDescriptor{};
  tip_desc.description = "Tip parameter";
  node_->declare_parameter("tip", "", tip_desc);
  tip = node_->get_parameter("tip").as_string();
  RCLCPP_INFO_STREAM(node_->get_logger(), "Tip: " << tip);


  /* End handling parameters*/
  // Convert xacro to urdf and store it as an string containing the urdf
  std::string urdf_string = xacroToString();

  // Initialize Allegro Hand class
  if(fake_hand){
    // Simulated hand
    setUp(yaml_file, urdf_string);
  }
  else{
    // Real hand
    setUp(yaml_file, pcan, channel, urdf_string);
  }

  // TODO: Why només si slave mirar (no recordo)?
  // Start up services if not slave 
  if(!slave){
    setUpServices();
    // Action to Move to Configuration at a desired speed
    move_config_action_server = rclcpp_action::create_server<MoveConfig>(node_, "move_conf",
        std::bind(&AllegroRos::handle_goal_move_config, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&AllegroRos::handle_cancel_move_config, this, std::placeholders::_1),
        std::bind(&AllegroRos::handle_accepted_move_config, this, std::placeholders::_1));
    // Action move to fingertip poses
    move_fingertip_poses_action_server = rclcpp_action::create_server<MoveFingertipPoses>(node_, "move_fingertip_poses",
        std::bind(&AllegroRos::handle_goal_move_fingertip_poses, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&AllegroRos::handle_cancel_move_fingertip_poses, this, std::placeholders::_1),
        std::bind(&AllegroRos::handle_accepted_move_fingertip_poses, this, std::placeholders::_1));

    // Action move along a trajectory defined by a set of hand configurations
    move_trajectory_action_server = rclcpp_action::create_server<MoveTrajectory>(node_, "move_trajectory",       
        std::bind(&AllegroRos::handle_goal_move_trajectory, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&AllegroRos::handle_cancel_move_trajectory, this, std::placeholders::_1),
        std::bind(&AllegroRos::handle_accepted_move_trajectory, this, std::placeholders::_1));

    // Action move along a trajectory defined by a set of hand configurations with a maximum velocity defined
    move_trajectory_vel_action_server = rclcpp_action::create_server<MoveTrajectoryMaxVel>(node_, "move_trajectory_vel",       
        std::bind(&AllegroRos::handle_goal_move_trajectory_vel, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&AllegroRos::handle_cancel_move_trajectory_vel, this, std::placeholders::_1),
        std::bind(&AllegroRos::handle_accepted_move_trajectory_vel, this, std::placeholders::_1));

  // Action move along a trajectory defined by a set of hand configurations
    follow_joint_trajectory_action_server = rclcpp_action::create_server<FollowJointTrajectory>(node_, "follow_joint_trajectory",       
        std::bind(&AllegroRos::handle_goal_follow_joint_trajectory, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&AllegroRos::handle_cancel_follow_joint_trajectory, this, std::placeholders::_1),
        std::bind(&AllegroRos::handle_accepted_follow_joint_trajectory, this, std::placeholders::_1));

    // Action to move a finger to a given configuration at a speed based on the desired percentage speed
    move_config_finger_action_server = rclcpp_action::create_server<MoveConfigFinger>(node_, "move_conf_finger",
        std::bind(&AllegroRos::handle_goal_move_config_finger, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&AllegroRos::handle_cancel_move_config_finger, this, std::placeholders::_1),
        std::bind(&AllegroRos::handle_accepted_move_config_finger, this, std::placeholders::_1));

    // Action move one fingertip to the desired pose
    move_fingertip_pose_action_server = rclcpp_action::create_server<MoveFingertipPose>(node_, "move_fingertip_pose",
        std::bind(&AllegroRos::handle_goal_move_fingertip_pose, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&AllegroRos::handle_cancel_move_fingertip_pose, this, std::placeholders::_1),
        std::bind(&AllegroRos::handle_accepted_move_fingertip_pose, this, std::placeholders::_1));
  //   // Action to Move to Configuration at a desired speed
  //   move_config_action_server = rclcpp_action::create_server<MoveConfig>(node_, this->prefix + "move_conf",
  //       std::bind(&AllegroRos::handle_goal_move_config, this, std::placeholders::_1, std::placeholders::_2),
  //       std::bind(&AllegroRos::handle_cancel_move_config, this, std::placeholders::_1),
  //       std::bind(&AllegroRos::handle_accepted_move_config, this, std::placeholders::_1));
  //   // Action move to fingertip poses
  //   move_fingertip_poses_action_server = rclcpp_action::create_server<MoveFingertipPoses>(node_, this->prefix + "move_fingertip_poses",
  //       std::bind(&AllegroRos::handle_goal_move_fingertip_poses, this, std::placeholders::_1, std::placeholders::_2),
  //       std::bind(&AllegroRos::handle_cancel_move_fingertip_poses, this, std::placeholders::_1),
  //       std::bind(&AllegroRos::handle_accepted_move_fingertip_poses, this, std::placeholders::_1));

  //   // Action move along a trajectory defined by a set of hand configurations
  //   move_trajectory_action_server = rclcpp_action::create_server<MoveTrajectory>(node_, this->prefix + "move_trajectory",       
  //       std::bind(&AllegroRos::handle_goal_move_trajectory, this, std::placeholders::_1, std::placeholders::_2),
  //       std::bind(&AllegroRos::handle_cancel_move_trajectory, this, std::placeholders::_1),
  //       std::bind(&AllegroRos::handle_accepted_move_trajectory, this, std::placeholders::_1));

  //   // Action move along a trajectory defined by a set of hand configurations with a maximum velocity defined
  //   move_trajectory_vel_action_server = rclcpp_action::create_server<MoveTrajectoryMaxVel>(node_, this->prefix + "move_trajectory_vel",       
  //       std::bind(&AllegroRos::handle_goal_move_trajectory_vel, this, std::placeholders::_1, std::placeholders::_2),
  //       std::bind(&AllegroRos::handle_cancel_move_trajectory_vel, this, std::placeholders::_1),
  //       std::bind(&AllegroRos::handle_accepted_move_trajectory_vel, this, std::placeholders::_1));

  // // Action move along a trajectory defined by a set of hand configurations
  //   follow_joint_trajectory_action_server = rclcpp_action::create_server<FollowJointTrajectory>(node_, this->prefix + "follow_joint_trajectory",       
  //       std::bind(&AllegroRos::handle_goal_follow_joint_trajectory, this, std::placeholders::_1, std::placeholders::_2),
  //       std::bind(&AllegroRos::handle_cancel_follow_joint_trajectory, this, std::placeholders::_1),
  //       std::bind(&AllegroRos::handle_accepted_follow_joint_trajectory, this, std::placeholders::_1));

  //   // Action to move a finger to a given configuration at a speed based on the desired percentage speed
  //   move_config_finger_action_server = rclcpp_action::create_server<MoveConfigFinger>(node_, this->prefix + "move_conf_finger",
  //       std::bind(&AllegroRos::handle_goal_move_config_finger, this, std::placeholders::_1, std::placeholders::_2),
  //       std::bind(&AllegroRos::handle_cancel_move_config_finger, this, std::placeholders::_1),
  //       std::bind(&AllegroRos::handle_accepted_move_config_finger, this, std::placeholders::_1));

  //   // Action move one fingertip to the desired pose
  //   move_fingertip_pose_action_server = rclcpp_action::create_server<MoveFingertipPose>(node_, this->prefix + "move_fingertip_pose",
  //       std::bind(&AllegroRos::handle_goal_move_fingertip_pose, this, std::placeholders::_1, std::placeholders::_2),
  //       std::bind(&AllegroRos::handle_cancel_move_fingertip_pose, this, std::placeholders::_1),
  //       std::bind(&AllegroRos::handle_accepted_move_fingertip_pose, this, std::placeholders::_1));

  }
  else{
    RCLCPP_INFO_STREAM(node_->get_logger(), "AllegroHandRos: Slave Mode" << std::endl);
  }

  // Set initial controller accordingly to the parameter initial_controller if the hand is real
  if(!fake_hand){
    // Real hand
    setController(initial_controller);
  }
  // else{
  //   // Simulated hand allows only the FakeController
  //   std::string new_current_controller = "Fake";
  //   setController(new_current_controller); // fake_controller is similar as GravityCompensator
  //   std::array<double,n_fingers * finger_n_dofs> init_pos{};
  //   init_pos[12] = 0.83; //set to be in range
  //   setDesiredConfiguration(init_pos);
  // }
  
  // Advertise node publishers 
  // Joint states
  RCLCPP_INFO_STREAM(node_->get_logger(), "Advertising node publishers" << std::endl);
  if(!slave){
    joints_pub = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 100); //1000
  }
  else{
    joints_pub = node_->create_publisher<sensor_msgs::msg::JointState>(name_space + "joint_states", 100); //1000
  }

  // Init wrench stuff
  // All the wrench will be w.r.t the tip_id of the finger
  tip_names[0] = std::string(this->prefix +link_prefix + "link_3_0_tip");
  tip_names[1] = std::string(this->prefix +link_prefix + "link_7_0_tip");
  tip_names[2] = std::string(this->prefix +link_prefix + "link_11_0_tip");
  tip_names[3] = std::string(this->prefix +link_prefix + "link_15_0_tip");

  for(size_t i = 0; i < 4; ++i)
  {
    wrench_stamped_array[i].header.frame_id = tip_names[i];
    contact_point_frame_stamped[i].header.frame_id = tip_names[i];
  }

  contact_point_frame_stamped[0].child_frame_id = this->prefix +link_prefix + "contact_frame_Index_ros";
  contact_point_frame_stamped[1].child_frame_id = this->prefix +link_prefix + "contact_frame_Middle_ros";
  contact_point_frame_stamped[2].child_frame_id = this->prefix +link_prefix + "contact_frame_Annular_ros";
  contact_point_frame_stamped[3].child_frame_id = this->prefix +link_prefix + "contact_frame_Thumb_ros";

  // wrench_pub[0] = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(this->prefix +"wrench_contact_frame_Index", 10);
  // wrench_pub[1] = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(this->prefix +"wrench_contact_frame_Middle", 10);
  // wrench_pub[2] = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(this->prefix +"wrench_contact_frame_Ring", 10);
  // wrench_pub[3] = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(this->prefix +"wrench_contact_frame_Thumb", 10);
  
  wrench_pub[0] = node_->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench_contact_frame_Index", 10);
  wrench_pub[1] = node_->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench_contact_frame_Middle", 10);
  wrench_pub[2] = node_->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench_contact_frame_Ring", 10);
  wrench_pub[3] = node_->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench_contact_frame_Thumb", 10);
  //Publisher and subscriber to commandHand
  if(!slave)
  {
    commandHand = node_->create_subscription<sensor_msgs::msg::JointState>("joint_state_commanded", 1000, std::bind(&AllegroRos::setCommandHand, this, std::placeholders::_1));
    joints_commanded = node_->create_publisher<sensor_msgs::msg::JointState>("joint_state_commanded", 1000);
    // commandHand = node_->create_subscription<sensor_msgs::msg::JointState>(this->prefix +"joint_state_commanded", 1000, std::bind(&AllegroRos::setCommandHand, this, std::placeholders::_1));
    // joints_commanded = node_->create_publisher<sensor_msgs::msg::JointState>(this->prefix +"joint_state_commanded", 1000);
  }
  // if it is slave, it has not to subscribe to any topic to command the hand
  /*else
  {
    commandHand = this->create_subscription<sensor_msgs::msg::JointState>(handed + "/joint_state_commanded", 1000, std::bind(&AllegroRos::setCommandHand, this, std::placeholders::_1));
    joints_commanded = this->create_publisher<sensor_msgs::msg::JointState>(handed +"/joint_state_commanded", 1000);
  }*/

  // Fill joint names in jointStates.name
  jointStatesMsg.name.reserve(AllegroHand::n_fingers* AllegroHand::finger_n_dofs);
  for (unsigned int i = 0; i < AllegroHand::n_fingers* AllegroHand::finger_n_dofs; ++i) {
    jointStatesMsg.name.push_back("joint_"+std::to_string(i)+"_0");
  }
  jointStatesMsg.position.resize(AllegroHand::n_fingers* AllegroHand::finger_n_dofs);
  jointStatesMsg.velocity.resize(AllegroHand::n_fingers* AllegroHand::finger_n_dofs);
  jointStatesMsg.effort.resize(AllegroHand::n_fingers* AllegroHand::finger_n_dofs);

  contact_frame.setOrigin(tf2::Vector3(0.,0.,0.));
  contact_frame.setRotation(tf2::Quaternion(0.,1.,0.,1.));

  // First init
  for(size_t i = 0; i < 4; ++i)
  {
    contact_point_frame_stamped[i].header.stamp = node_->get_clock()->now();
    contact_point_frame_stamped[i].transform = tf2::toMsg(contact_frame);

    // std::cout << "contact_point_frame_stamped[" << i << "]:" << std::endl;
    // std::cout << "Header:" << std::endl;
    // std::cout << "  Stamp: " << contact_point_frame_stamped[i].header.stamp.sec << "s "
    //                           << contact_point_frame_stamped[i].header.stamp.nanosec << "ns" << std::endl;
    // std::cout << "  Frame ID: " << contact_point_frame_stamped[i].header.frame_id << std::endl;
    // std::cout << "  Child ID: " << contact_point_frame_stamped[i].child_frame_id << std::endl;

    // std::cout << "Transform:" << std::endl;
    // std::cout << "  Translation: [" << contact_point_frame_stamped[i].transform.translation.x << ", "
    //                                   << contact_point_frame_stamped[i].transform.translation.y << ", "
    //                                   << contact_point_frame_stamped[i].transform.translation.z << "]" << std::endl;
    // std::cout << "  Rotation: [" << contact_point_frame_stamped[i].transform.rotation.x << ", "
    //                                << contact_point_frame_stamped[i].transform.rotation.y << ", "
    //                                << contact_point_frame_stamped[i].transform.rotation.z << ", "
    //                                << contact_point_frame_stamped[i].transform.rotation.w << "]" << std::endl;

    br_contact_frame = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    // need to be improved. For other programs we need someone to publish this tf
    // br_contact_frame->sendTransform(contact_point_frame_stamped[i]); /////////////HEREEEEE
  }

  cog_x = std::vector<double>(4,0.);
  cog_y = std::vector<double>(4,0.);
  force = std::vector<double>(4,0.);
  area  = std::vector<double>(4,0.);

  RCLCPP_INFO_STREAM(node_->get_logger(), "AllegroHandRos initialization completed." << std::endl);

// loop_function();

}


/**
* @brief Function to create the services
*/
void AllegroRos::setUpServices(){
  RCLCPP_INFO_STREAM(node_->get_logger(), "Advertising node services" << std::endl);
  get_controller_srv = node_->create_service<allegro_hand_interfaces::srv::GetController>("getController", std::bind(&AllegroRos::getControllerSrv, this, std::placeholders::_1, std::placeholders::_2));
  set_controller_srv = node_->create_service<allegro_hand_interfaces::srv::SetController>("setController", std::bind(&AllegroRos::setControllerSrv, this, std::placeholders::_1, std::placeholders::_2));
  position_srv = node_->create_service<allegro_hand_interfaces::srv::SetDesPos>("setDesiredConfiguration", std::bind(&AllegroRos::setDesiredConfigurationSrv, this, std::placeholders::_1, std::placeholders::_2));
  stop_srv = node_->create_service<allegro_hand_interfaces::srv::StopHand>("stopHand", std::bind(&AllegroRos::stopHandSrv, this, std::placeholders::_1, std::placeholders::_2));
  getLimits_srv = node_->create_service<allegro_hand_interfaces::srv::JointLimits>("getLimits", std::bind(&AllegroRos::getLimitsSrv, this, std::placeholders::_1, std::placeholders::_2));
  get_pose_fk_srv = node_->create_service<allegro_hand_interfaces::srv::GetPoseFK>("getPoseFK", std::bind(&AllegroRos::getPoseFKSrv, this, std::placeholders::_1, std::placeholders::_2));
  set_pose_ik_srv = node_->create_service<allegro_hand_interfaces::srv::SetPoseIK>("setPoseIK", std::bind(&AllegroRos::setPoseIKSrv, this, std::placeholders::_1, std::placeholders::_2));
  get_torque_srv = node_->create_service<allegro_hand_interfaces::srv::GetTorque>("getTorque", std::bind(&AllegroRos::getTorqueSrv, this, std::placeholders::_1, std::placeholders::_2));
  set_torque_srv = node_->create_service<allegro_hand_interfaces::srv::SetDesTorque>("setTorque", std::bind(&AllegroRos::setTorqueSrv, this, std::placeholders::_1, std::placeholders::_2));
  get_gravity_vector_srv = node_->create_service<allegro_hand_interfaces::srv::GetGravityVector>("getGravityVector", std::bind(&AllegroRos::getGravityVectorSrv, this, std::placeholders::_1, std::placeholders::_2));
  set_gravity_vector_srv = node_->create_service<allegro_hand_interfaces::srv::SetGravityVector>("setGravityVector", std::bind(&AllegroRos::setGravityVectorSrv, this, std::placeholders::_1, std::placeholders::_2));
  coupled_closing_srv = node_->create_service<allegro_hand_interfaces::srv::CoupledClosing>("coupledClosing", std::bind(&AllegroRos::coupledClosingSrv, this, std::placeholders::_1, std::placeholders::_2));
  get_hand_configuration_srv = node_->create_service<allegro_hand_interfaces::srv::GetHandConfiguration>("getHandConfiguration", std::bind(&AllegroRos::getHandConfigurationSrv, this, std::placeholders::_1, std::placeholders::_2));
  get_fingers_movement_status_srv = node_->create_service<allegro_hand_interfaces::srv::GetFingersMovementStatus>("getFingersMovementStatus", std::bind(&AllegroRos::getFingersMovementStatusSrv, this, std::placeholders::_1, std::placeholders::_2));
  pause_finger_srv = node_->create_service<allegro_hand_interfaces::srv::PauseFinger>("pauseFinger", std::bind(&AllegroRos::pauseFingerSrv, this, std::placeholders::_1, std::placeholders::_2));
  continue_finger_srv = node_->create_service<allegro_hand_interfaces::srv::ContinueFinger>("continueFinger", std::bind(&AllegroRos::continueFingerSrv, this, std::placeholders::_1, std::placeholders::_2));
  stop_finger_srv = node_->create_service<allegro_hand_interfaces::srv::StopFinger>("stopFinger", std::bind(&AllegroRos::stopFingerSrv, this, std::placeholders::_1, std::placeholders::_2));    
  
  // get_controller_srv = node_->create_service<allegro_hand_interfaces::srv::GetController>(this->prefix +"getController", std::bind(&AllegroRos::getControllerSrv, this, std::placeholders::_1, std::placeholders::_2));
  // set_controller_srv = node_->create_service<allegro_hand_interfaces::srv::SetController>(this->prefix +"setController", std::bind(&AllegroRos::setControllerSrv, this, std::placeholders::_1, std::placeholders::_2));
  // position_srv = node_->create_service<allegro_hand_interfaces::srv::SetDesPos>(this->prefix +"setDesiredConfiguration", std::bind(&AllegroRos::setDesiredConfigurationSrv, this, std::placeholders::_1, std::placeholders::_2));
  // stop_srv = node_->create_service<allegro_hand_interfaces::srv::StopHand>(this->prefix +"stopHand", std::bind(&AllegroRos::stopHandSrv, this, std::placeholders::_1, std::placeholders::_2));
  // getLimits_srv = node_->create_service<allegro_hand_interfaces::srv::JointLimits>(this->prefix +"getLimits", std::bind(&AllegroRos::getLimitsSrv, this, std::placeholders::_1, std::placeholders::_2));
  // get_pose_fk_srv = node_->create_service<allegro_hand_interfaces::srv::GetPoseFK>(this->prefix +"getPoseFK", std::bind(&AllegroRos::getPoseFKSrv, this, std::placeholders::_1, std::placeholders::_2));
  // set_pose_ik_srv = node_->create_service<allegro_hand_interfaces::srv::SetPoseIK>(this->prefix +"setPoseIK", std::bind(&AllegroRos::setPoseIKSrv, this, std::placeholders::_1, std::placeholders::_2));
  // get_torque_srv = node_->create_service<allegro_hand_interfaces::srv::GetTorque>(this->prefix +"getTorque", std::bind(&AllegroRos::getTorqueSrv, this, std::placeholders::_1, std::placeholders::_2));
  // set_torque_srv = node_->create_service<allegro_hand_interfaces::srv::SetDesTorque>(this->prefix +"setTorque", std::bind(&AllegroRos::setTorqueSrv, this, std::placeholders::_1, std::placeholders::_2));
  // get_gravity_vector_srv = node_->create_service<allegro_hand_interfaces::srv::GetGravityVector>(this->prefix +"getGravityVector", std::bind(&AllegroRos::getGravityVectorSrv, this, std::placeholders::_1, std::placeholders::_2));
  // set_gravity_vector_srv = node_->create_service<allegro_hand_interfaces::srv::SetGravityVector>(this->prefix +"setGravityVector", std::bind(&AllegroRos::setGravityVectorSrv, this, std::placeholders::_1, std::placeholders::_2));
  // coupled_closing_srv = node_->create_service<allegro_hand_interfaces::srv::CoupledClosing>(this->prefix +"coupledClosing", std::bind(&AllegroRos::coupledClosingSrv, this, std::placeholders::_1, std::placeholders::_2));
  // get_hand_configuration_srv = node_->create_service<allegro_hand_interfaces::srv::GetHandConfiguration>(this->prefix +"getHandConfiguration", std::bind(&AllegroRos::getHandConfigurationSrv, this, std::placeholders::_1, std::placeholders::_2));
  // get_fingers_movement_status_srv = node_->create_service<allegro_hand_interfaces::srv::GetFingersMovementStatus>(this->prefix +"getFingersMovementStatus", std::bind(&AllegroRos::getFingersMovementStatusSrv, this, std::placeholders::_1, std::placeholders::_2));
  // pause_finger_srv = node_->create_service<allegro_hand_interfaces::srv::PauseFinger>(this->prefix +"pauseFinger", std::bind(&AllegroRos::pauseFingerSrv, this, std::placeholders::_1, std::placeholders::_2));
  // continue_finger_srv = node_->create_service<allegro_hand_interfaces::srv::ContinueFinger>(this->prefix +"continueFinger", std::bind(&AllegroRos::continueFingerSrv, this, std::placeholders::_1, std::placeholders::_2));
  // stop_finger_srv = node_->create_service<allegro_hand_interfaces::srv::StopFinger>(this->prefix +"stopFinger", std::bind(&AllegroRos::stopFingerSrv, this, std::placeholders::_1, std::placeholders::_2));    
}

/**
 * @brief Function to get the active controller
 */
bool AllegroRos::getController() {
  std::cout << "Active controller: " << current_controller << std::endl;
  return true;
}

/**
 * @brief Function to get the selected controller
 * @param req Void
 * @param res Return True if succeed
 * @return Return a Boolean
 */
bool AllegroRos::getControllerSrv(const std::shared_ptr<allegro_hand_interfaces::srv::GetController::Request> /*req*/, std::shared_ptr<allegro_hand_interfaces::srv::GetController::Response> res){
    res->success = getController();
    return res->success;
}


/**
 * @brief Function to set the selected controller
 * @param req Type of controller
 * @param res Return True if succeed
 * @return Return a Boolean
 */
bool AllegroRos::setControllerSrv(const std::shared_ptr<allegro_hand_interfaces::srv::SetController::Request> req,
                          std::shared_ptr<allegro_hand_interfaces::srv::SetController::Response> res) {
    try{
        std::string new_current_controller = static_cast<std::string>(req->tipo);
        std::cout << "Setting AH-ROS controller: " << new_current_controller << std::endl;
        res->success = setController(new_current_controller);
        return true;
    } catch (std::exception &e) {
        RCLCPP_ERROR(node_->get_logger(), "Caught exception when setting the Controller: %s", e.what());
        return false;
    }
}


/**
 * @brief Function to set the position of each joint of the Allegro hand
 * @param req This parameter contains the position of each joint
 * @param res Return True if succeed
 * @return Return a Boolean
 */
bool AllegroRos::setDesiredConfiguration(std::array<double, n_fingers * finger_n_dofs> &goal_pos) {
  AllegroHandDriver::Vector desired_pos;
  try{
    for (auto i=0; i<AllegroHand::n_fingers*AllegroHand::finger_n_dofs; ++i){
      desired_pos(i) = goal_pos[i];
    }
    // if (!AllegroHand::moveToConf(desired_pos)) { 
    if (!AllegroHand::setDesiredConfiguration(desired_pos)) { // Does not check the controller
      RCLCPP_ERROR(node_->get_logger(), "Invalid setDesiredConfiguration");
      return false;
    }
    return true;
  }catch (std::exception &e) {
      RCLCPP_ERROR(node_->get_logger(), "Caught exception when setting the Desired Position: setDesiredConfiguration: %s", e.what());
      return false;
  }
}

/**
 * @brief Function to set the position of each joint of the Allegro hand
 * @param req This parameter contains the position of each joint
 * @param res Return True if succeed
 * @return Return a Boolean
 */
 bool AllegroRos::setDesiredConfigurationSrv(const std::shared_ptr<allegro_hand_interfaces::srv::SetDesPos::Request> req,
                          std::shared_ptr<allegro_hand_interfaces::srv::SetDesPos::Response> res){
   if(req->pos.size() != n_fingers * finger_n_dofs)
   {
     RCLCPP_ERROR(node_->get_logger(), "Dimension mismatch! (setDesiredConfigurationSrv)");
     res->success = false;
   }
   else
   {
    // If we are using the Torque or the GravityCompensator controller, we set up the PID.
    if ((current_controller == "Torque") || (current_controller == "GravityCompensator")){
        std::cout << "Current controller: " << current_controller << std::endl;
        std::cout << "In order to move to a desired configuration, the PD or PID controller is needed.";
        setController("JointPositionPID");
    }
    std::array<double,n_fingers * finger_n_dofs> pos_array;
    std::move(req->pos.begin(), req->pos.begin() + n_fingers * finger_n_dofs, pos_array.begin());
    res->success = setDesiredConfiguration(pos_array);
   }
  return res->success;
}



/**
 * @brief Function to set the position and orientation of the hand using Inverse Kinematics
 * @param req This parameter contains the position and orientation
 * @param res Return True if succeed
 * @return Return a Boolean
 */
 bool AllegroRos::setPoseIKSrv(const std::shared_ptr<allegro_hand_interfaces::srv::SetPoseIK::Request> req,
                          std::shared_ptr<allegro_hand_interfaces::srv::SetPoseIK::Response> res){
   if(req->desired_position.size() != n_fingers * 3)
   {
     RCLCPP_ERROR(node_->get_logger(), "Dimension mismatch! (setPoseIKSrv)");
     res->success = false;
   }
   else
   {
    executeIK(true,req->desired_position, req->desired_orientation);
    res->success = true;
   }

  return res->success;
}


/**
 * @brief Function to get the position and orientation of the hand using Forward Kinematics
 * @param req This parameter is empty
 * @param res Return True if succeed, and provides the position and orientation (euler angles) of the hand
 * @return Return a Boolean
 */
 bool AllegroRos::getPoseFKSrv(const std::shared_ptr<allegro_hand_interfaces::srv::GetPoseFK::Request> /*req*/,
                          std::shared_ptr<allegro_hand_interfaces::srv::GetPoseFK::Response> res){
  Eigen::Matrix<double,3,4> euler_angles;
  Eigen::Matrix<double,3,4> translation;
  bool success_ = computeFK(true,euler_angles, translation);
  std::vector<double> euler_angles_vec(euler_angles.data(), euler_angles.data() + euler_angles.size());
  std::vector<double> translation_vec(translation.data(), translation.data() + translation.size());

  if(success_){
    // res->euler_angles = euler_angles;
    // res->translation = translation;
    // Print euler_angles
    // std::cout << "** Result: ** " << std::endl;
    // std::cout << "Euler Angles:" << std::endl;
    // RCLCPP_INFO(node_->get_logger(), "Euler Angles: ");
    // for (int i = 0; i < euler_angles.cols(); ++i) {
    //     // std::cout << "Finger " << i << ": ";
    //     for (int j = 0; j < euler_angles.rows(); ++j) {
    //         std::cout << euler_angles(j, i) << " ";
    //         // RCLCPP_INFO_STREAM(node_->get_logger(), "%s ", this->euler_angles(j, i).c_str());
    //     }
    //     // std::cout << std::endl;
    // }
    // // Print translation
    // RCLCPP_INFO(node_->get_logger(), "Positions (x,y,z):");
    // // std::cout << std::endl << "Positions (x,y,z):" << std::endl;
    // for (int i = 0; i < translation.cols(); ++i) {
    //     // std::cout << "Finger " << i << ": ";
    //     for (int j = 0; j < translation.rows(); ++j) {
    //         std::cout << translation(j, i) << " ";
    //         // RCLCPP_INFO_STREAM(node_->get_logger(), "%s ", this->translation(j, i).c_str());
 
    //     }
    //     // std::cout << std::endl;
    // }
    std::stringstream euler_ss, translation_ss;
    euler_ss << "Euler Angles: ";
    for (auto angle : euler_angles_vec) {
        euler_ss << angle << " ";
    }
    translation_ss << "Positions (x,y,z): ";
    for (auto value : translation_vec) {
        translation_ss << value << " ";
    }
    RCLCPP_INFO(node_->get_logger(), "%s", euler_ss.str().c_str());
    RCLCPP_INFO(node_->get_logger(), translation_ss.str().c_str());
  }
  else{
    RCLCPP_ERROR(node_->get_logger(), "Error computing FK (getPoseFKSrv)");
  }
  res->euler_angles = euler_angles_vec;
  res->translation = translation_vec;
  res->success = success_;
  return res->success;
}

// TODO: Think about emergency stop
/**
* @brief Function to stop the hand movement
* @param res Return True if succeed
* @return Return a Boolean
*/
bool AllegroRos::stopHandSrv(const std::shared_ptr<allegro_hand_interfaces::srv::StopHand::Request>  /*req*/,
                    std::shared_ptr<allegro_hand_interfaces::srv::StopHand::Response> res){
  try{
    // ros::spinOnce();
    stopHand();
    mutex_ros.lock();
      stop_ros_thread = true;
    mutex_ros.unlock();
    res->success = true;
    return true;
  } catch (std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Caught exception Stop Hand: %s", e.what());
    return false;
  }
}


/**
 * @brief Get the limits from the Allegro hand
 * @param res Return True if succeed
 * @return Return a Boolean
 */
bool AllegroRos::getLimitsSrv(const std::shared_ptr<allegro_hand_interfaces::srv::JointLimits::Request>  /*req*/,
                      std::shared_ptr<allegro_hand_interfaces::srv::JointLimits::Response> res){
  try{
    std::vector<double> min_limits;
    std::vector<double> max_limits;
    // AllegroHand::Vector foo;
    getJointLimitsMin(min_limits);
    // std::vector<double> minLimits(foo.data(), foo.data() + foo.rows() * foo.cols());
    res->min = min_limits;
    getJointLimitsMax(max_limits);
    // std::vector<double> maxLimits(foo.data(), foo.data() + foo.rows() * foo.cols());
    res->max = max_limits;
    std::cout << "Joint limits: " << std::endl;
    for(size_t i = 0; i < AllegroHand::n_fingers*AllegroHand::finger_n_dofs; ++i){
      std::cout << "Joint " << i << " min: " << min_limits[i] << ", max: " << max_limits[i] << std::endl;
    }
    return true;
  } catch (std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Caught exception getLimitsSrv: %s", e.what());
    return false;
  }
}


bool AllegroRos::getTorqueSrv(const std::shared_ptr<allegro_hand_interfaces::srv::GetTorque::Request> /*req*/, 
                        std::shared_ptr<allegro_hand_interfaces::srv::GetTorque::Response> res){
  // std::cout << "ASDF" ;
  try{
    AllegroHandDriver::Vector torque_;
    AllegroHandDriver::getTorque(torque_);
    std::vector<double> torque_vector;
    for(auto i=0; i<AllegroHandDriver::n_fingers*AllegroHandDriver::finger_n_dofs; ++i){
      torque_vector.push_back(torque_(i));
    }
    res->torque = torque_vector;
    res->success = true;
    return true;
  } catch (std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Caught exception getTorqueSrv: %s", e.what());
    return false;
  }
}

/**
 * @brief Funtion to set the desired torque to each joint
 * @param req Sends the desired torque to controller
 * @param res Return True if succeed
 * @return Return a Boolean
 */
bool AllegroRos::setTorqueSrv(const std::shared_ptr<allegro_hand_interfaces::srv::SetDesTorque::Request> req,
                             std::shared_ptr<allegro_hand_interfaces::srv::SetDesTorque::Response> res){
  try{
    if(req->torque.size() != n_fingers * finger_n_dofs){
      RCLCPP_ERROR(node_->get_logger(), "Dimension mismatch! (setTorqueSrv)");
      res->success = false;
      return false;
    }
    else{
      AllegroHandDriver::Vector torque_;
      std::string new_current_controller = "Torque";
      setController(new_current_controller);
      for(auto i=0; i<AllegroHandDriver::n_fingers*AllegroHandDriver::finger_n_dofs; ++i){
        torque_(i) = req->torque[i];
      }
      setTorque(torque_);
      res->success = true;
      return true;
    }
  } catch (std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Caught exception setDesiredTorqueSrv: %s", e.what());
    return false;
  }
}

bool AllegroRos::getGravityVectorSrv(const std::shared_ptr<allegro_hand_interfaces::srv::GetGravityVector::Request> /*req*/,
                        std::shared_ptr<allegro_hand_interfaces::srv::GetGravityVector::Response> res){
  try{
    std::array<double, 3> gravity_vect;
    getGravityVector(gravity_vect);
    std::vector<double> gravity_vector;
    for(long unsigned int i=0; i<gravity_vect.size(); ++i){
      gravity_vector.push_back(gravity_vect[i]);
    }
    res->gravity_vector = gravity_vector;
    res->success = true;
    return true;
  } catch (std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Caught exception getGravityVectorSrv: %s", e.what());
    return false;
  }
}

bool AllegroRos::setGravityVectorSrv(const std::shared_ptr<allegro_hand_interfaces::srv::SetGravityVector::Request> req,
                        std::shared_ptr<allegro_hand_interfaces::srv::SetGravityVector::Response> res){
  try{
    if(req->gravity_vector.size() != 3){
      RCLCPP_ERROR(node_->get_logger(), "Dimension mismatch! (setGravityVectorSrv)");
      res->success = false;
      return false;
    }
    else{
      std::array<double, 3> gravity_vect;
      for(auto i=0; i<3; ++i){
        gravity_vect[i] = req->gravity_vector[i];
      }
      setGravityVector(gravity_vect);
      res->success = true;
      return true;
    }
  } catch (std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Caught exception setGravityVectorSrv: %s", e.what());
    return false;
  }
}


bool AllegroRos::coupledClosingSrv(const std::shared_ptr<allegro_hand_interfaces::srv::CoupledClosing::Request> req,
                              std::shared_ptr<allegro_hand_interfaces::srv::CoupledClosing::Response> res){
  try{
      coupledClosing(req->coupled_value);
      res->success = true;
      return true;
  } catch (std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Caught exception coupledClosingSrv: %s", e.what());
    return false;
  }
}

bool AllegroRos::getHandConfigurationSrv(const std::shared_ptr<allegro_hand_interfaces::srv::GetHandConfiguration::Request> /*req*/,
                              std::shared_ptr<allegro_hand_interfaces::srv::GetHandConfiguration::Response> res){
  try{
    AllegroHandDriver::Vector q_current;
    getCurrentState(q_current);
    std::vector<double> q_current_vector;
    for(auto i=0; i<q_current.size(); ++i){
      q_current_vector.push_back(q_current(i));
    }

    res->current_configuration = q_current_vector;
    res->success = true;
    return true;
  } catch (std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Caught exception getHandConfigurationSrv: %s", e.what());
    return false;
  }
}

bool AllegroRos::getFingersMovementStatusSrv(const std::shared_ptr<allegro_hand_interfaces::srv::GetFingersMovementStatus::Request> /*req*/,
                                 std::shared_ptr<allegro_hand_interfaces::srv::GetFingersMovementStatus::Response> res){
  try{
    std::array<bool, n_fingers> finger_moving_;
    getFingerMovementStatus(finger_moving_);
    std::vector<bool> finger_moving_vector;
    for(auto i=0; i<n_fingers; ++i){
      finger_moving_vector.push_back(finger_moving_[i]);
    }

    res->fingers_status = finger_moving_vector;
    return true;
  } catch (std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Caught exception getFingersMovementStatusSrv: %s", e.what());
    return false;
  }
}

bool AllegroRos::pauseFingerSrv(const std::shared_ptr<allegro_hand_interfaces::srv::PauseFinger::Request> req,
                          std::shared_ptr<allegro_hand_interfaces::srv::PauseFinger::Response> res){
  try{
    // Check if the finger_id is correct
    if(req->finger_id > n_fingers){
      RCLCPP_ERROR(node_->get_logger(), "Invalid Finger ID for pauseFingerSrv");
    }
    // Check if the hand is moving
    else if(!this->moving){
      RCLCPP_ERROR(node_->get_logger(), "The hand is not moving, thus we can not stop a finger");
    }
    else{
      mutex_finger_moving.lock();
        bool finger_moving_ = finger_moving[req->finger_id];
      mutex_finger_moving.unlock();
      if(finger_moving_){
        // Get the current joint values (since the hand is moving is better to get them from here instead of the trajectory list)
        Vector q_current;
        getCurrentState(q_current);
        int index;

        mutex_current_trajectory.lock();
        RCLCPP_INFO(node_->get_logger(), "Before pause: current_trajectory size: %lu, stored_trajectory size: %lu",
              current_trajectory.size(), stored_trajectory.size());
          // If we haven't stoped we set the stop matrix to 0 
          if(stored_trajectory.empty()){
            // trajectory_size = current_trajectory.size();
            stored_trajectory.resize(current_trajectory.size() - num_current_configuration);
            for(long unsigned int i = 0; i < stored_trajectory.size(); ++i) {
                stored_trajectory[i].setZero();
            } 
            index = 0;
          }
          else{
            index = stored_trajectory.size() - (current_trajectory.size()-num_current_configuration);
          }
          // If we haven't stopped yet we will store them
          if(paused_index_stored_trajectory[req->finger_id] == -1){
            // We populate the stored_trajectory with the future values of the paused finger and the global current_trajoctery with the pause values
            for(long unsigned int i = num_current_configuration;i < current_trajectory.size(); i++){
              for(int ii = req->finger_id*n_fingers; ii < (req->finger_id*n_fingers + finger_n_dofs); ii++){
                // Store the next finger configurations to save them
                stored_trajectory[index](ii) = current_trajectory[i](ii);
                // Set the current trajectory all the same configuration values
                current_trajectory[i](ii) = q_current(ii);
              }
              index ++;
            }
            paused_index_stored_trajectory[req->finger_id] = stored_trajectory.size() - (current_trajectory.size()-num_current_configuration);
          }
          else{
            for(long unsigned int i = num_current_configuration;i < current_trajectory.size(); i++){
              for(int ii = req->finger_id*n_fingers; ii < (req->finger_id*n_fingers + finger_n_dofs); ii++){
                // Set the current trajectory all the same configuration values
                current_trajectory[i](ii) = q_current(ii);
              }
            }
            paused_index_stored_trajectory[req->finger_id] += num_current_configuration - continue_index_current_trajectory[req->finger_id];
          }
        RCLCPP_INFO(node_->get_logger(), "After pause: current_trajectory size: %lu, stored_trajectory size: %lu",
              current_trajectory.size(), stored_trajectory.size());
        mutex_current_trajectory.unlock();
        RCLCPP_INFO(node_->get_logger(), "Pausing finger %d at index %d", req->finger_id, paused_index_stored_trajectory[req->finger_id]);
        mutex_finger_moving.lock();
          finger_moving[req->finger_id] = false;
        mutex_finger_moving.unlock();

        res->success = true;
        return true;
      }
      else{
        RCLCPP_ERROR(node_->get_logger(), "The finger is not moving, thus we can not stop a finger");
      }
    }
    res->success = false;
    return false;
  } catch (std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Caught exception pauseFingerSrv: %s", e.what());
    return false;
  }
}

bool AllegroRos::continueFingerSrv(const std::shared_ptr<allegro_hand_interfaces::srv::ContinueFinger::Request> req,
                             std::shared_ptr<allegro_hand_interfaces::srv::ContinueFinger::Response> res){
  try{
    // Check if the finger_id is correct
    if(req->finger_id > n_fingers){
      RCLCPP_ERROR(node_->get_logger(), "Invalid Finger ID for continueFingerSrv");
    }
    // Check if the hand is moving
    else if(!this->moving){
      RCLCPP_ERROR(node_->get_logger(), "The hand is not moving, thus we can not continue the finger movement");
    }
    else{
      mutex_finger_moving.lock();
        bool finger_moving_ = finger_moving[req->finger_id];
      mutex_finger_moving.unlock();
      if(!finger_moving_){
        if(stored_trajectory.empty()){
          RCLCPP_ERROR(node_->get_logger(), "There is no stored trajectory. Check if you paused or stopped the finger");
        }
        else{
        RCLCPP_INFO(node_->get_logger(), "Continuing finger %d from index %d", req->finger_id, paused_index_stored_trajectory[req->finger_id]);
          // We populate the stored_trajectory with the future values of the paused finger and the global current_trajoctery with the pause values
          mutex_current_trajectory.lock();
            RCLCPP_INFO(node_->get_logger(), "Before continue: current_trajectory size: %lu, stored_trajectory size: %lu",
            current_trajectory.size(), stored_trajectory.size());
            if(paused_index_stored_trajectory[req->finger_id] != -1){
              unsigned long int index = paused_index_stored_trajectory[req->finger_id];
              // We populate until the end of the other fingers trajectories
              for(unsigned long int i = num_current_configuration;i<current_trajectory.size(); i++){
                for(int ii = req->finger_id*n_fingers; ii < (req->finger_id*n_fingers + finger_n_dofs); ii++){
                  // Set the current trajectory all the same configuration values
                  current_trajectory[i](ii) = stored_trajectory[index](ii);
                }
                index ++;
                if(index >= stored_trajectory.size()){
                  index = stored_trajectory.size()-1;
                }
              }

              // We populate the other fingers with their final value until this finger finishes its movement 
              AllegroHandDriver::Vector final_configuration = current_trajectory.back();
              for(unsigned long int i = index;i<stored_trajectory.size(); i++){
                for(int ii = req->finger_id*n_fingers; ii < (req->finger_id*n_fingers + finger_n_dofs); ii++){
                  // Set the current trajectory all the same configuration values
                  final_configuration(ii) = stored_trajectory[i](ii);
                }
                current_trajectory.push_back(final_configuration);
              }
              num_points = current_trajectory.size();
            }
            continue_index_current_trajectory[req->finger_id] = num_current_configuration;
            RCLCPP_INFO(node_->get_logger(), "After continue: current_trajectory size: %lu, stored_trajectory size: %lu",
            current_trajectory.size(), stored_trajectory.size());
          mutex_current_trajectory.unlock();
          mutex_finger_moving.lock();
            finger_moving[req->finger_id] = true;
          mutex_finger_moving.unlock();
          res->success = true;
          return true;
        }
      }
      else{
        RCLCPP_ERROR(node_->get_logger(), "The finger is already moving, thus we can not continue the finger movement");
      }
    }
    res->success = false;
    return false;
    
  } catch (std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Caught exception continueFingerSrv: %s", e.what());
    return false;
  }
}

bool AllegroRos::stopFingerSrv(const std::shared_ptr<allegro_hand_interfaces::srv::StopFinger::Request> req,
                         std::shared_ptr<allegro_hand_interfaces::srv::StopFinger::Response> res){
  try{
    // Check if the finger_id is correct
    if(req->finger_id > n_fingers){
      RCLCPP_ERROR(node_->get_logger(), "Invalid Finger ID for stopFingerSrv");
    }
    // Check if the hand is moving
    else if(!this->moving){
      RCLCPP_ERROR(node_->get_logger(), "The hand is not moving, thus we can not stop a finger");
    }
    else{
      // Get the current joint values (since the hand is moving is better to get them from here instead of the trajectory list)
      Vector q_current;
      getCurrentState(q_current);
      // We populate the global current_trajoctery with the pause values
      mutex_current_trajectory.lock();
        for(long unsigned int i = num_current_configuration;i<current_trajectory.size(); i++){
          for(int ii = req->finger_id*n_fingers; ii < (req->finger_id*n_fingers + finger_n_dofs); ii++){
            // Set the current trajectory all the same configuration values
            current_trajectory[i](ii) = q_current(ii);
          }
        }
      mutex_current_trajectory.unlock();
      mutex_finger_moving.lock();
        finger_moving[req->finger_id] = false;
      mutex_finger_moving.unlock();
      res->success = true;
      return true;
      res->success = true;
      return true;
    }
    res->success = false;
    return false;
  } catch (std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Caught exception stopFingerSrv: %s", e.what());
    return false;
  }
}


void AllegroRos::setCommandHand(const sensor_msgs::msg::JointState& msg)
{
  AllegroHandDriver::Vector desired_pos;
  // Publishing jointStates
  for (auto i = 0; i < AllegroHand::n_fingers; ++i) {
    for (auto j = 0; j < AllegroHand::finger_n_dofs; ++j){
      desired_pos(AllegroHand::finger_n_dofs*i+j) = msg.position.at(AllegroHand::finger_n_dofs*i+j);
    }
  }
  try{
    commandUpdate = true;
    if (!AllegroHand::setDesiredConfiguration(desired_pos)) {
      RCLCPP_ERROR(node_->get_logger(), "Invalid Desired Pos");
    }
    commandUpdate = false;
  }catch (std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Caught exception setCommandHand: %s", e.what());
  }
}

/**
 * @brief Start loop function of the node
 */
void AllegroRos::start_loop_function(){
  mutex_ros.lock();
    stop_ros_thread = false;
  mutex_ros.unlock();
  ros_thread = std::thread(&AllegroRos::loop_function, this);
}

/**
 * @brief Stop loop function of the node
 */
void AllegroRos::stop_loop_function(){
  mutex_ros.lock();
    stop_ros_thread = true;
  mutex_ros.unlock();
}

/**
 * @brief Loop function of the node
 */
void AllegroRos::loop_function(){

  std::vector<double> joints;
  joints.resize(n_fingers * finger_n_dofs);

  // //int shape = visualization_msgs::Marker::ARROW;
  KDL::Frame contactPointFrame, previousFrame;
  KDL::Wrench newBaseWrench, oldBaseWrench;
  KDL::Vector origin(0.,0.,0.), contact(0., 0., 0.);
  geometry_msgs::msg::Transform tf_contact;
  Vector joint_positions, vel_, acc_, torque_;
  Vector torqueKDL, torque_gc;

  while(rclcpp::ok() && !stop_ros_thread) {

    // Get status from hand
    getCurrentState(joint_positions,vel_,acc_,torque_);
    getGravityComp(torque_gc); //TODO delete (torque_gc i torqueKDL si no tenim en compte la gc) or nah
    torqueKDL = torque_;// - torque_gc; should we take out the gravity compensation?

    for (auto dof=0; dof < AllegroHand::n_fingers*AllegroHand::finger_n_dofs; ++dof){
      joints[dof]=joint_positions(dof);
    }
    // Compute Jacobian considering contact point
    // this function update the  Allegro->FingersFrames with
    // the contact point
    ComputeJacobian(joints,cog_x,cog_y);
    std::array<std::vector<KDL::Frame>, n_fingers> finger_frames;
    getFingerFrames(finger_frames); //TODO: Take it out of AHC and put it into AH

    for (size_t nf=0; nf< AllegroHand::n_fingers; ++nf){
      //realJTI_kdl[nf] = Allegro->RealJacobianTransposeKDL[nf].completeOrthogonalDecomposition().pseudoInverse();
      realJ_kdl[nf] = getRealJacobianKDLAtIndex(nf);

      realWrenchKDL[nf] = realJ_kdl[nf] * torqueKDL.segment<AllegroHand::finger_n_dofs>(nf*AllegroHand::finger_n_dofs);

      for(size_t i = 0; i<AllegroHand::n_fingers; ++i){
        oldBaseWrench(i) = realWrenchKDL[nf](i);
      }

      // Now we want to express this wrench w.r.t. the contact point.
      // KDL has
      
      int n = finger_frames[nf].size();
      // which is the last frame w.r.t base_link
      contactPointFrame = finger_frames[nf].at(n-1);
      // change base w.r.t. new frame
      newBaseWrench = contactPointFrame * oldBaseWrench;

      previousFrame = finger_frames[nf].at(n-2);
      contactPointFrame = previousFrame.Inverse() * contactPointFrame;
      // now contactPointFrame is w.r.t previous Frame

      // publish in Tf
      // static tf2_ros::TransformBroadcaster br_contact;
      // we need to publish the tf with the contact point for each finger
      // Convert KDL frame to TransformStamped      
      geometry_msgs::msg::TransformStamped tf_contact_stamped = tf2::kdlToTransform(contactPointFrame);
      tf_contact = tf_contact_stamped.transform;

      contact_point_frame_stamped[nf].header.stamp = node_->get_clock()->now();
      // contact_point_frame_stamped[nf].transform = tf2::toMsg(contact_frame);
      contact_point_frame_stamped[nf].transform = tf_contact;
      // br_contact_frame->sendTransform(contact_point_frame_stamped[nf]); // HEREEEE 

      wrench_array[nf].force.x  = newBaseWrench(0);
      wrench_array[nf].force.y  = newBaseWrench(1);
      wrench_array[nf].force.z  = newBaseWrench(2);
      wrench_array[nf].torque.x = newBaseWrench(3);
      wrench_array[nf].torque.y = newBaseWrench(4);
      wrench_array[nf].torque.z = newBaseWrench(5);

      wrench_stamped_array[nf].header.stamp = node_->get_clock()->now();
      wrench_stamped_array[nf].wrench = wrench_array[nf];

      // Publish the WrenchStamped message
      wrench_pub[nf]->publish(wrench_stamped_array[nf]);
    }

    //std::cout << "Publishing joint_states" << std::endl;
    for (auto i = 0; i < AllegroHand::n_fingers; ++i) {
      for (auto j = 0; j < AllegroHand::finger_n_dofs; ++j){
        jointStatesMsg.position.at(AllegroHand::finger_n_dofs*i+j) = joint_positions(AllegroHand::finger_n_dofs*i+j);
        jointStatesMsg.velocity.at(AllegroHand::finger_n_dofs*i+j) = vel_(AllegroHand::finger_n_dofs*i+j);
        //jointStatesMsg.acceleration.at(AllegroHand::finger_n_dofs*i+j) = acc_(AllegroHand::finger_n_dofs*i+j);
        jointStatesMsg.effort.at(AllegroHand::n_fingers*i+j) = torque_(AllegroHand::n_fingers*i+j);
      }
    }

    jointStatesMsg.header.stamp.sec++;
    jointStatesMsg.header.frame_id = "joint_states";
    jointStatesMsg.header.stamp = node_->get_clock()->now();

    joints_pub->publish(jointStatesMsg);
    rclcpp::spin_some(node_);
    // rclcpp::spin_some(shared_from_this());
    rate.sleep();
  }
}


//! \brief Convert Xacro to string
std::string AllegroRos::xacroToString() {
  std::array<char, 128> buffer;
  std::string urdf_string;
  std::string package_share_directory, xacro_file_path, hand;
  
  // Parameter Hand (only used here)
  auto hand_desc = rcl_interfaces::msg::ParameterDescriptor{};
  hand_desc.description = "Hand parameter";
  node_->declare_parameter("hand", "", hand_desc);
  hand = node_->get_parameter("hand").as_string();
  RCLCPP_INFO_STREAM(node_->get_logger(), "hand: " << hand);

  // Get the package share directory for allegro_hand_des
  try {
      package_share_directory = ament_index_cpp::get_package_share_directory("allegro_hand_des");
  } catch (const std::exception &e) {
      std::cerr << "Failed to find package share directory: " << e.what() << std::endl;
      return urdf_string;
  }

  // Construct the full path to the Xacro file
  xacro_file_path = "xacro " + package_share_directory + "/urdf/allegro_hand_des.xacro";
  
  // Append parameters to the xacro_file_path
  xacro_file_path = xacro_file_path + " hand:=" + hand;
  xacro_file_path = xacro_file_path + " tip:=" + tip;
  xacro_file_path = xacro_file_path + " link_prefix:=" + link_prefix;
  xacro_file_path = xacro_file_path + " joint_prefix:=" + joint_prefix;

  std::shared_ptr<FILE> pipe(popen(xacro_file_path.c_str(), "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    urdf_string += buffer.data();
  }

  if (urdf_string.empty()) {
    throw std::runtime_error("Failed to convert Xacro to URDF.");
  }

  RCLCPP_INFO_STREAM(node_->get_logger(), "URDF: " << urdf_string);

  return urdf_string;
}