
#include <allegro_hand/AllegroHand.h>

/**

The functions of the AllegroHand class have been grouped in different files for enhancing readability.

In this file we can find the constructor, destructor and the main control loops and a function to read the URDF

The rest of the functions are grouped in:
- AllegroHandConfigurations. Related to configurations and trajectories
- AllegroHandControl. Related to controller functions.
- AllegroHandJointLimits. Related to AllegroHand Joint limits
- AllegroHandKinematics. Related to FK, Ik, Jacobian.

This class extends the AllegroHandDriver class, that contains the basic driver functions to communicate with the hand.

**/

//! \brief Constructor Default constructor.
// Creates an instance of the AllegroHandDriver (Opens CAN communication without loop inside AHDriver) and runs control
// loop with control mode to TORQUE and commanded torque = Zero
//! \param _conf_file Configuration file of the hand, can be the directory of the file or the file name (with or without
//! extension) (Provides encoder offset, motor directions, ...) \param _pcan_port Communications port \param channel
//! Communications channel
AllegroHand::AllegroHand(std::string _conf_file, std::string _pcan_port, unsigned int _channel)
  : AllegroHandDriver(false, _conf_file, _pcan_port, _channel)
  ,  // AllegroHandDriver without inner loop
  moving(false)
  , link_prefix("")
  , joint_prefix("")
  , q_desired(Vector::Zero())
  ,
  // torque_desired(Vector::Zero()),
  status(true)
  ,  // Execute controlLoop
  listening_to_trajectory_to_store(false)
  , safety_margin_joint_limits_values(16, 0.0)
{
  setUpAH(false);
}

//! \brief Constructor Default constructor. Creates a simulated hand to operate.
//! \param _conf_file Configuration file of the hand, can be the directory of the file or the file name (with or without
//! extension) (Provides encoder offset, motor directions, ...)
AllegroHand::AllegroHand(std::string _conf_file)
  : AllegroHandDriver(_conf_file)
  , moving(false)
  , link_prefix("")
  , joint_prefix("")
  , q_desired(Vector::Zero())
  ,
  // torque_desired(Vector::Zero()),
  status(true)
  , listening_to_trajectory_to_store(false)
  , safety_margin_joint_limits_values(16, 0.0)
{
  setUpAH(true);
}

//! \brief Empty constructor
AllegroHand::AllegroHand(bool only_conf, std::string _conf_file)
  : AllegroHandDriver(_conf_file)
  , moving(false)
  , link_prefix("")
  , joint_prefix("")
  , q_desired(Vector::Zero())
  , status(true)
  , listening_to_trajectory_to_store(false)
  , safety_margin_joint_limits_values(16, 0.0)
{
  // File where we will write the configurations that we want to store
  if (is_left)
  {
    config_yaml_file = directory_conf_folder + "/configurations/left/configs.yaml";
    config_yaml_file_opposite_hand = directory_conf_folder + "/configurations/right/configs.yaml";
  }
  else
  {
    config_yaml_file = directory_conf_folder + "/configurations/right/configs.yaml";
    config_yaml_file_opposite_hand = directory_conf_folder + "/configurations/left/configs.yaml";
  }
  // Uploading the list of predefined configurations by reading conf/configurations
  searchPredefinedConfigurations();
  // In case there are no predefined configurations in the yaml file we set a list of defined configurations from the
  // AllegroHand function setConfList
  if (configurations_list.empty())
  {
    std::cout << "No configuration YAML files found. Proceeding to set a set of basic Allegro Hand configurations"
              << std::endl;
    setConfList();
  }
  else
  {
    std::cout << "Configuration YAML file has been uploaded" << std::endl;
    // Print in the console the names of the configurations in the yaml file
    getConfList(true);
  }
  // Read predefined trajectories
  readTrajectories();
}

//! \brief Empty constructor
AllegroHand::AllegroHand()
  : AllegroHandDriver()
  , moving(false)
  , link_prefix("")
  , joint_prefix("")
  , q_desired(Vector::Zero())
  ,
  // torque_desired(Vector::Zero()),
  status(true)
  , listening_to_trajectory_to_store(false)
  , safety_margin_joint_limits_values(16, 0.0)
  , AllegroController(nullptr)
{
}

//! \brief Default destructor
AllegroHand::~AllegroHand()
{
  // Stop controlLoop
  mutex.lock();
  status = false;
  mutex.unlock();
  // Finish control thread
  controlThread.join();
  // Delete the current controller instance
  delete AllegroController;
  // Delete Kinematics instances
  delete index_finger;
  delete middle_finger;
  delete anular_finger;
  delete thumb_finger;
}

//! \brief  Initialization function. Used in the constructor
//! \param simulated_hand boolean to know if it is a real or a simulated hand
void AllegroHand::setUpAH(bool simulated_hand)
{
  mutex_controller.lock();
  controller_available = false;
  mutex_controller.unlock();
  // Read the URDF file.
  readConfUrdf();

  // Set up the controllers depending on whether the hand is real or simulated
  if (!simulated_hand)
  {
    // Get torque conversion (provided by the manufacturer)
    torqueConv = getTorqueConversion();
    // Load available controllers from folder conf/controllers
    searchControllers();
    // Include Basic Controllers
    controllers_names.push_back("Torque");
    controllers_names.push_back("GravityCompensator");
    // Initialize AllegroHandController
    // Set torque controller
    current_controller = "";
    setController("Torque");
  }
  else
  {
    torqueConv = 1200;
    // Include Fake Controller
    controllers_names.push_back("Fake");
    // Initialize AllegroHandController
    // Set fake controller
    current_controller = "";
    setController("Fake");
  }
  // Print available controllers
  getControllersNames();

  // File where we will write the configurations that we want to store
  if (is_left)
  {
    config_yaml_file = directory_conf_folder + "/configurations/left/configs.yaml";
    config_yaml_file_opposite_hand = directory_conf_folder + "/configurations/right/configs.yaml";
  }
  else
  {
    config_yaml_file = directory_conf_folder + "/configurations/right/configs.yaml";
    config_yaml_file_opposite_hand = directory_conf_folder + "/configurations/left/configs.yaml";
  }
  // Uploading the list of predefined configurations by reading conf/configurations
  searchPredefinedConfigurations();
  // In case there are no predefined configurations in the yaml file we set a list of defined configurations from the
  // AllegroHand function setConfList
  if (configurations_list.empty())
  {
    std::cout << "No configuration YAML files found. Proceeding to set a set of basic Allegro Hand configurations"
              << std::endl;
    setConfList();
  }
  else
  {
    std::cout << "Configuration YAML file has been uploaded" << std::endl;
    // Print in the console the names of the configurations in the yaml file
    getConfList(true);
  }
  // Read predefined trajectories
  readTrajectories();
  // Read predefined trajectories defined with fingertip poses
  getFileNamesFromFolder(name_folder_trajectories + "/finger_poses/", trajectories_poses_list);

  // Print list of trajectories
  getTrajectoriesListNames();

  // Initialize each finger. Related to the kinematics
  // We can use the predefined values of joint limits and link lengths
  // index_finger = new IMAFinger(0);
  // middle_finger = new IMAFinger(1);
  // anular_finger = new IMAFinger(2);
  // thumb_finger = new ThumbFinger();
  // We rather use the values obtained from the URDF
  index_finger = new IMAFinger(0, lengths_links_finger_I,
                               std::vector<double>(q_min_urdf.begin(), q_min_urdf.begin() + finger_n_dofs),
                               std::vector<double>(q_max_urdf.begin(), q_max_urdf.begin() + finger_n_dofs));
  middle_finger =
      new IMAFinger(1, lengths_links_finger_M,
                    std::vector<double>(q_min_urdf.begin() + finger_n_dofs, q_min_urdf.begin() + 2 * finger_n_dofs),
                    std::vector<double>(q_max_urdf.begin() + finger_n_dofs, q_max_urdf.begin() + 2 * finger_n_dofs));
  anular_finger = new IMAFinger(
      2, lengths_links_finger_A,
      std::vector<double>(q_min_urdf.begin() + 2 * finger_n_dofs, q_min_urdf.begin() + 3 * finger_n_dofs),
      std::vector<double>(q_max_urdf.begin() + 2 * finger_n_dofs, q_max_urdf.begin() + 3 * finger_n_dofs));
  thumb_finger = new ThumbFinger(
      lengths_links_finger_Thumb,
      std::vector<double>(q_min_urdf.begin() + 3 * finger_n_dofs, q_min_urdf.begin() + 4 * finger_n_dofs),
      std::vector<double>(q_max_urdf.begin() + 3 * finger_n_dofs, q_max_urdf.begin() + 4 * finger_n_dofs));

  // Starting controlLoop. It will read the current configuration from the hand, then execute the current controller and
  // sent the calculated torque towards the hand.
  std::cout << std::endl << "Starting control thread" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  if (!simulated_hand)
  {
    controlThread = std::thread(&AllegroHand::controlLoop, this);
  }
  else
  {
    controlThread = std::thread(&AllegroHand::controlLoopFake, this);
  }
}

//! \brief  Set up function to initialize the real hand. Useful when empty constructor By initializing the
//! communications too. \param _conf_file Configuration file of the hand, can be the directory of the file or the file
//! name (with or without extension) (Provides encoder offset, motor directions, ...) \param _pcan_port Communications
//! port \param channel Communications channel \param urdf_string_ String of the URDF that we want to use
void AllegroHand::setUp(std::string _conf_file, std::string _pcan_port, unsigned int _channel, std::string urdf_string_)
{
  // Call the set up function of the allegro hand driver
  setUpAHD(_conf_file, _pcan_port, _channel);
  // Store the string of the urdf that we want to use
  urdf_string = urdf_string_;
  // Call the set up function of the allegro hand
  setUpAH(false);
}
//! \brief  Set up function to initialize the simulated hand
//! \param _conf_file Configuration file of the hand, can be the directory of the file or the file name (with or without
//! extension) (Provides encoder offset, motor directions, ...) \param urdf_string_ String of the URDF that we want to
//! use
void AllegroHand::setUp(std::string _conf_file, std::string urdf_string_)
{
  // Call the set up function of the allegro hand driver
  setUpAHD(_conf_file);
  // Store the string of the urdf that we want to use
  urdf_string = urdf_string_;
  // Call the set up function of the allegro hand
  setUpAH(true);
}

//! \brief  Set up function to initialize the real hand. Useful when empty constructor By initializing the
//! communications too. \param _conf_file Configuration file of the hand, can be the directory of the file or the file
//! name (with or without extension) (Provides encoder offset, motor directions, ...) \param _pcan_port Communications
//! port \param channel Communications channel
void AllegroHand::setUp(std::string _conf_file, std::string _pcan_port, unsigned int _channel)
{
  // Call the set up function of the allegro hand driver
  setUpAHD(_conf_file, _pcan_port, _channel);
  // Call the set up function of the allegro hand
  setUpAH(false);
}
//! \brief  Set up function to initialize the simulated hand
//! \param _conf_file Configuration file of the hand, can be the directory of the file or the file name (with or without
//! extension) (Provides encoder offset, motor directions, ...)
void AllegroHand::setUp(std::string _conf_file)
{
  // Call the set up function of the allegro hand driver
  setUpAHD(_conf_file);
  // Call the set up function of the allegro hand
  setUpAH(true);
}
//! \brief Control loop. Reads and executes the control and sends the torque to the hand
void AllegroHand::controlLoop()
{
  bool status_;
  bool controller_available_;
  Vector torque_ = Vector::Zero();
  // Read the current configuration
  AllegroHandDriver::getCurrentStateComm(q_current);
  // Initialize status and previous configuration
  mutex.lock();
  status_ = status;
  q_prev = q_current;
  mutex.unlock();
  mutex_torque.lock();
  torque_ = torque;
  mutex_torque.unlock();
  while (status_)
  {
    // Read the current configuration from the hand
    AllegroHandDriver::getCurrentStateComm(q_current);
    mutex_controller.lock();
    controller_available_ = controller_available;
    mutex_controller.unlock();
    if (controller_available_)
    {
      mutex_torque.lock();
      torque_ = torque_desired;
      mutex_torque.unlock();
      // Execute Controller. We calculate the torque we want to send based on the controller
      AllegroController->executeController(q_prev, q_current, torque_, q_desired);
      // Set and send torque. Send the calculated torque with the controller towards the Allegro Hand.
      AllegroHandDriver::sendTorque(torque_);
      // Update the previous configuration and the status
      mutex.lock();
        q_prev = q_current;
        status_ = status;
      mutex.unlock();
      mutex_torque.lock();
      torque = torque_;
      mutex_torque.unlock();
    }
  }
  std::cout << "Control Loop has been stopped" << std::endl;
}

//! \brief Fake control loop. Reads and executes the control and simulates sending the torque towards the hand
void AllegroHand::controlLoopFake()
{
  bool status_;
  bool success;
  bool controller_available_;

  // Initialize current configuration to 0.
  q_current.setZero();
  // copy to q_current the home configuration from config_yaml_file
  q_current = configurations_list["home"];
  q_desired = q_current;
  // Initialize status and previous configuration
  mutex.lock();
  status_ = status;
  q_prev = q_current;
  mutex.unlock();

  while (status_)
  {
    mutex_controller.lock();
    controller_available_ = controller_available;
    mutex_controller.unlock();
    if (controller_available_)
    {
      mutex.lock();
      q_current = q_desired;
      mutex.unlock();
      // Execute Controller. We calculate the torque we want to send based on the controller
      AllegroController->executeController(q_prev, q_current, torque, q_desired);
      // Check/Adjust the torque wiwthin the actual limits
      success = AllegroHandDriver::setDesiredTorque(torque);
      if (!success)
      {
        std::cout << "Torque out of range, ignored" << std::endl;
      }
      // Update the previous configuration and the status
      mutex.lock();
      q_prev = q_current;
      status_ = status;
      mutex.unlock();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  std::cout << "Control Loop has been stopped" << std::endl;
}

//! \brief Read the URDF file. Extract the q, vel, torque limits and the KDL tree (and thus the finger_chain and the
//! chain)
bool AllegroHand::readConfUrdf()
{
  YAML::Node config;

  urdf::Model urdf_model;

  Vector joint_origin_x;
  Vector joint_origin_y;
  Vector joint_origin_z;
  Vector joint_origin_tip_x;
  Vector joint_origin_tip_z;

  // Get URDF
  if (!urdf_model.initString(urdf_string))
  {
    throw std::runtime_error("Failed to parse URDF string.");
  }

  // Read joint limits from urdf
  std::cout << "Reading joint limits from URDF" << std::endl;
  for (unsigned int i = 0; i < n_fingers * finger_n_dofs; ++i)
  {
    auto limits = urdf_model.getJoint(joint_prefix + "joint_" + std::to_string(i) + "_0")->limits;
    q_min_urdf.push_back(limits->lower);
    q_max_urdf.push_back(limits->upper);
    q_min.push_back(limits->lower);
    q_max.push_back(limits->upper);
    vel_max.push_back(limits->velocity);
    torque_max[i] = limits->effort;
    std::cout << joint_prefix << "joint_" << i << "_0: q_min:" << q_min_urdf[i] << ", q_max:" << q_max_urdf[i]
              << ", vel_max: " << vel_max[i] << ", torque_max: " << torque_max[i] << std::endl;
  }
  updateJointLimitsURDF();
  updateJointLimits();

  // Read joint origin information (xyz). Useful for the kinematics part

  std::cout << "Reading joint origin" << std::endl;
  for (unsigned int i = 0; i < n_fingers * finger_n_dofs; ++i)
  {
    auto origin =
        urdf_model.getJoint(joint_prefix + "joint_" + std::to_string(i) + "_0")->parent_to_joint_origin_transform;
    joint_origin_x(i) = origin.position.x;
    joint_origin_y(i) = origin.position.y;
    joint_origin_z(i) = origin.position.z;
    // std::cout << "Origin joint " << i <<"_0: " << joint_origin_x(i) << ", " << joint_origin_y(i) << ", " <<
    // joint_origin_z(i) << std::endl;
  }
  for (unsigned int i = 0; i < n_fingers; ++i)
  {
    auto origin = urdf_model.getJoint(joint_prefix + "joint_" + std::to_string(4 * i + 3) + "_0_tip")
                      ->parent_to_joint_origin_transform;
    joint_origin_tip_x(i) = origin.position.x;
    joint_origin_tip_z(i) = origin.position.z;
    // std::cout << "Origin joint " << i <<"_0_tip: " << joint_origin_tip_x(i) << ", " << joint_origin_tip_z(i) <<
    // std::endl;
  }

  // We store the lengths of the links. Useful for the kinematics part.
  lengths_links_finger_I = { joint_origin_z(1), joint_origin_z(2), joint_origin_z(3), joint_origin_tip_z(0),
                             joint_origin_tip_x(0) };
  lengths_links_finger_M = { joint_origin_z(5), joint_origin_z(6), joint_origin_z(7), joint_origin_tip_z(1),
                             joint_origin_tip_x(1) };
  lengths_links_finger_A = { joint_origin_z(9), joint_origin_z(10), joint_origin_z(11), joint_origin_tip_z(2),
                             joint_origin_tip_x(2) };
  lengths_links_finger_Thumb = { joint_origin_y(13), joint_origin_z(13) + joint_origin_z(14), joint_origin_z(15),
                                 joint_origin_tip_z(3), joint_origin_tip_x(3) };
  // std::cout << "Links finger I: " << lengths_links_finger_I[0] << ", " << lengths_links_finger_I[1] << ", " <<
  // lengths_links_finger_I[2] << ", " << lengths_links_finger_I[3] << ", " << lengths_links_finger_I[4] << std::endl;
  // std::cout << "Links finger M: " << lengths_links_finger_M[0] << ", " << lengths_links_finger_M[1] << ", " <<
  // lengths_links_finger_M[2] << ", " << lengths_links_finger_M[3] << ", " << lengths_links_finger_M[4] << std::endl;
  // std::cout << "Links finger A: " << lengths_links_finger_A[0] << ", " << lengths_links_finger_A[1] << ", " <<
  // lengths_links_finger_A[2] << ", " << lengths_links_finger_A[3] << ", " << lengths_links_finger_A[4] << std::endl;
  // std::cout << "Links finger Thumb: " << lengths_links_finger_Thumb[0] << ", " << lengths_links_finger_Thumb[1] << ",
  // " << lengths_links_finger_Thumb[2] << ", " << lengths_links_finger_Thumb[3] << ", " <<
  // lengths_links_finger_Thumb[4] << std::endl;

  if (!kdl_parser::treeFromUrdfModel(urdf_model, tree))
    throw std::runtime_error("Failed to construct KDL tree.");

  for (unsigned int nf = 0; nf < n_fingers; ++nf)
  {
    // chains from URDF
    if (!tree.getChain("base_link", link_prefix + "link_" + std::to_string(3 + finger_n_dofs * nf) + "_0_tip",
                       chain.at(nf)))
      throw std::runtime_error("Failed to construct " + std::to_string(nf + 1) + "-th KDL chain.");

    if (chain.at(nf).getNrOfJoints() != finger_n_dofs)
      throw std::invalid_argument("Invalid KDL chain.");

    // chains from URDF changing last fix joint
    if (!tree.getChain("base_link", link_prefix + "link_" + std::to_string(3 + finger_n_dofs * nf) + "_0",
                       finger_chain.at(nf)))
      throw std::runtime_error("Failed to construct " + std::to_string(nf + 1) + "-th KDL chain.");

    if (finger_chain.at(nf).getNrOfJoints() != finger_n_dofs)
      throw std::invalid_argument("Invalid KDL chain.");
  }
  return true;
}