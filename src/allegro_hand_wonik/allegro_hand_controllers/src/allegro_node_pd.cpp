using namespace std;

#include "allegro_node_pd.h"
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"

// Conversion macros.
#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI))
#define DEGREES_TO_RADIANS(angle) ((angle) / 180.0 * M_PI)

// --- Default PD Controller Parameters ---

// Default proportional gains (P gains) for the PD controller.
// These are used if the "gains_pd.yaml" parameter file is not loaded.
double k_p[DOF_JOINTS] = {
    600.0, 600.0, 600.0, 1000.0,
    600.0, 600.0, 600.0, 1000.0,
    600.0, 600.0, 600.0, 1000.0,
    1000.0, 1000.0, 1000.0, 600.0
};

// Default derivative gains (D gains) for the PD controller.
double k_d[DOF_JOINTS] = {
    15.0, 20.0, 15.0, 15.0,
    15.0, 20.0, 15.0, 15.0,
    15.0, 20.0, 15.0, 15.0,
    30.0, 20.0, 20.0, 15.0
};

// Default home pose (in degrees). This is applied if the initial_position
// parameter file is not loaded.
double home_pose[DOF_JOINTS] = {
    0.0, -10.0, 45.0, 45.0,
    0.0, -10.0, 45.0, 45.0,
    5.0, -5.0, 50.0, 45.0,
    60.0, 25.0, 15.0, 45.0
};

// Parameter names for each joint's P gain (as stored on the parameter server).
std::string pGainParams[DOF_JOINTS] = {
    "~gains_pd/p/j00", "~gains_pd/p/j01", "~gains_pd/p/j02", "~gains_pd/p/j03",
    "~gains_pd/p/j10", "~gains_pd/p/j11", "~gains_pd/p/j12", "~gains_pd/p/j13",
    "~gains_pd/p/j20", "~gains_pd/p/j21", "~gains_pd/p/j22", "~gains_pd/p/j23",
    "~gains_pd/p/j30", "~gains_pd/p/j31", "~gains_pd/p/j32", "~gains_pd/p/j33"
};

// Parameter names for each joint's D gain.
std::string dGainParams[DOF_JOINTS] = {
    "~gains_pd/d/j00", "~gains_pd/d/j01", "~gains_pd/d/j02", "~gains_pd/d/j03",
    "~gains_pd/d/j10", "~gains_pd/d/j11", "~gains_pd/d/j12", "~gains_pd/d/j13",
    "~gains_pd/d/j20", "~gains_pd/d/j21", "~gains_pd/d/j22", "~gains_pd/d/j23",
    "~gains_pd/d/j30", "~gains_pd/d/j31", "~gains_pd/d/j32", "~gains_pd/d/j33"
};

// Parameter names for each joint's initial position.
std::string initialPosition[DOF_JOINTS] = {
    "~initial_position/j00", "~initial_position/j01", "~initial_position/j02", "~initial_position/j03",
    "~initial_position/j10", "~initial_position/j11", "~initial_position/j12", "~initial_position/j13",
    "~initial_position/j20", "~initial_position/j21", "~initial_position/j22", "~initial_position/j23",
    "~initial_position/j30", "~initial_position/j31", "~initial_position/j32", "~initial_position/j33"
};

// --- Constructor and Destructor ---

// The constructor subscribes to command and joint state topics.
AllegroNodePD::AllegroNodePD(const std::string nodeName)
    : AllegroNode(nodeName)
{
  // Initially, hand control is turned off.
  control_hand_ = false;

  // Initialize the controller (gains and initial positions).
  initController(whichHand);

  // Subscription for external command strings.
  lib_cmd_sub = this->create_subscription<std_msgs::msg::String>(
      LIB_CMD_TOPIC, 1,
      std::bind(&AllegroNodePD::libCmdCallback, this, std::placeholders::_1));

  // Subscription for desired joint states.
  joint_cmd_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      DESIRED_STATE_TOPIC, 3,
      std::bind(&AllegroNodePD::setJointCallback, this, std::placeholders::_1));
}

// Destructor logs shutdown.
AllegroNodePD::~AllegroNodePD() {
  RCLCPP_INFO(this->get_logger(), "PD controller node is shutting down");
}

// --- Callback Functions ---

// Callback for external commands (strings) such as "pdControl", "home", "off", "save".
void AllegroNodePD::libCmdCallback(const std_msgs::msg::String &msg)
{
  RCLCPP_INFO(this->get_logger(), "CTRL: Heard: [%s]", msg.data.c_str());
  const std::string lib_cmd = msg.data;

  if (lib_cmd.compare("pdControl") == 0) {
    // Activate PD control.
    control_hand_ = true;
  }
  else if (lib_cmd.compare("home") == 0) {
    // Set the desired joint states to the home pose.
    mutex->lock();
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_joint_state.position[i] = DEGREES_TO_RADIANS(home_pose[i]);
    control_hand_ = true;
    mutex->unlock();
  }
  else if (lib_cmd.compare("off") == 0) {
    // Turn off control.
    control_hand_ = false;
  }
  else if (lib_cmd.compare("save") == 0) {
    // Save the current position as the desired position.
    mutex->lock();
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_joint_state.position[i] = current_position[i];
    mutex->unlock();
  }
}

// Called when a desired joint position message is received
// void AllegroNodePD::setJointCallback(const sensor_msgs::msg::JointState &msg) {
//   if(!control_hand_)
//   {
//     RCLCPP_WARN(this->get_logger(), "Setting control_hand_ to True because of "
//                                                           "received JointState message");
//     // TODO complete the function refering to AllegroNodeGrasp::setJointCallback
//   }
//   control_hand_ = true;
// }

// Callback for desired joint state messages.
void AllegroNodePD::setJointCallback(const sensor_msgs::msg::JointState &msg) {
  // control_hand_가 false인 경우 (grasp 노드의 동작과 유사하게) 경고 메시지를 출력. JointState 메시지가 핸드를 제어 모드로 강제 전환시키고 있음
  if (!control_hand_) {
    RCLCPP_WARN(this->get_logger(), "Setting control_hand_ to True because of received JointState message");
  }
  // 공유 변수인 desired_joint_state를 업데이트하기 전에 뮤텍스를 잠그고, 업데이트 후에 해제하여 스레드 안전성을 보장
  // Lock the mutex to safely update shared data.
  mutex->lock();
  // 수신된 msg.position 벡터가 정확히 DOF_JOINTS 개의 요소를 포함하고 있는지 확인. 만약 그렇지 않으면 오류를 기록
  if (msg.position.size() == DOF_JOINTS) {
    // Update each joint's desired position.
    for (int i = 0; i < DOF_JOINTS; i++) {
      desired_joint_state.position[i] = msg.position[i];
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Received JointState message with %zu positions; expected %d",
                 msg.position.size(), DOF_JOINTS);
  }
  mutex->unlock();

  // control_hand_를 true로 설정하여 이후의 컨트롤 반복(computeDesiredTorque()에서)에서 새로운 원하는 위치를 사용하도록
  // Set control flag so that PD control is applied.
  control_hand_ = true;
}


// --- Control Computation ---

// This function computes the desired torque to apply using PD control.
void AllegroNodePD::computeDesiredTorque() {
  // If control is disabled, set all desired torques to zero.
  if (!control_hand_) {
    for (int i = 0; i < DOF_JOINTS; i++) {
      desired_torque[i] = 0.0;
    }
    return;
  }

  // If both positions and efforts (torques) are specified, warn and do nothing.
  if (desired_joint_state.position.size() > 0 &&
      desired_joint_state.effort.size() > 0) {
    RCLCPP_WARN(this->get_logger(), "Error: both positions and torques are specified in the desired state. You cannot control both at the same time.");
    return;
  }

  // Lock the mutex for thread-safe access.
  mutex->lock();
  if (desired_joint_state.position.size() == DOF_JOINTS) {
    // Calculate PD control torque for each joint.
    double error;
    for (int i = 0; i < DOF_JOINTS; i++) {
      error = desired_joint_state.position[i] - current_position_filtered[i];
      desired_torque[i] = 1.0 / canDevice->torqueConversion() *
                          (k_p[i] * error - k_d[i] * current_velocity_filtered[i]);
    }
  } else if (desired_joint_state.effort.size() > 0) {
    // If desired torques are provided directly, use them.
    for (int i = 0; i < DOF_JOINTS; i++) {
      desired_torque[i] = desired_joint_state.effort[i];
    }
  }
  mutex->unlock();
}

// --- Initialization ---

// Initialize the PD controller by loading gains and initial joint positions.
void AllegroNodePD::initController(const std::string &whichHand) {
  // Log which hand is being controlled.
  if (whichHand.compare("left") == 0) {
    RCLCPP_WARN(this->get_logger(), "CTRL: Left Allegro Hand controller initialized.");
  } else {
    RCLCPP_WARN(this->get_logger(), "CTRL: Right Allegro Hand controller initialized.");
  }

  // --- Load PD Gains ---
  if (this->has_parameter("gains_pd"))
  {
    RCLCPP_INFO(this->get_logger(), "CTRL: PD gains loaded from param server.");
    for (int i = 0; i < DOF_JOINTS; i++) {
      std::string p_gain_param_name = pGainParams[i];
      std::string d_gain_param_name = dGainParams[i];
      rclcpp::Parameter p_gain_param, d_gain_param;
      this->get_parameter(p_gain_param_name, p_gain_param);
      this->get_parameter(d_gain_param_name, d_gain_param);
      k_p[i] = p_gain_param.as_double();
      k_d[i] = d_gain_param.as_double();
    }
  }
  else {
    RCLCPP_WARN(this->get_logger(), "CTRL: PD gains not loaded");
    RCLCPP_WARN(this->get_logger(), "Check launch file is loading /parameters/gains_pd.yaml");
    RCLCPP_WARN(this->get_logger(), "Loading default PD gains...");
  }

  // --- Load Initial Joint Positions ---
  if (this->has_parameter("~initial_position"))
  {
    RCLCPP_INFO(this->get_logger(), "CTRL: Initial Pose loaded from param server.");
    double tmp;
    mutex->lock();
    desired_joint_state.position.resize(DOF_JOINTS);
    for (int i = 0; i < DOF_JOINTS; i++) {
      this->get_parameter(initialPosition[i], tmp);
      desired_joint_state.position[i] = DEGREES_TO_RADIANS(tmp);
    }
    mutex->unlock();
  }
  else {
    RCLCPP_WARN(this->get_logger(), "CTRL: Initial position not loaded.");
    RCLCPP_WARN(this->get_logger(), "Check launch file is loading /parameters/initial_position.yaml");
    RCLCPP_WARN(this->get_logger(), "Loading Home position instead...");
    // Use the default home pose.
    mutex->lock();
    desired_joint_state.position.resize(DOF_JOINTS);
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_joint_state.position[i] = DEGREES_TO_RADIANS(home_pose[i]);
    mutex->unlock();
  }

  // Ensure control is initially turned off.
  control_hand_ = false;

  // Print control method information.
  RCLCPP_INFO(this->get_logger(), "*************************************\n");
  RCLCPP_INFO(this->get_logger(), "      Joint PD Control Method        \n");
  RCLCPP_INFO(this->get_logger(), "-------------------------------------\n");
  RCLCPP_INFO(this->get_logger(), "  Only 'H', 'O', 'S', 'Space' works. \n");
  RCLCPP_INFO(this->get_logger(), "*************************************\n");
}

// --- Main Control Loop ---

// Runs the control loop. If polling is enabled, use a loop with spin_some; otherwise, use a timer callback.
void AllegroNodePD::doIt(bool polling)
{
  // Create a shared pointer to this node.
  auto this_node = std::shared_ptr<AllegroNodePD>(this);
  int control_cycle = (int)(1 / ALLEGRO_CONTROL_TIME_INTERVAL);
  rclcpp::Rate rate(control_cycle);

  if (polling)
  {
    RCLCPP_INFO(this->get_logger(), "Polling = true.");
    while (rclcpp::ok())
    {
      updateController();            // Update controller logic.
      rclcpp::spin_some(this_node);  // Process any pending callbacks.
      rate.sleep();                  // Sleep to maintain the control rate.
    }
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Polling = false.");
    // Use timer callback (less recommended).
    rclcpp::TimerBase::SharedPtr timer = startTimerCallback();
    rclcpp::spin(this_node);
  }
}

// --- Main Function ---

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // Determine control mode (polling or timer) based on command-line argument.
  bool polling = false;
  if (argv[1] == std::string("true"))
  {
    polling = true;
  }

  RCLCPP_INFO(rclcpp::get_logger("allgro_node_pd"), "Start controller with polling = %d", polling);
  AllegroNodePD allegroNode("allegro_node_pd");
  allegroNode.doIt(polling);
}
