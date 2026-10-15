#ifndef ALLEGROHAND_H_
#define ALLEGROHAND_H_

#include "AllegroHandDriver.h"
// Controllers
#include "controllers/AllegroHandController.h"
#include "controllers/AllegroHandControllerGravityCompensator.h"
#include "controllers/AllegroHandControllerFake.h"
#include "controllers/AllegroHandControllerJointPositionPD.h"
#include "controllers/AllegroHandControllerJointPositionPID.h"
#include "controllers/AllegroHandControllerTorque.h"
// Kinematics
#include "kinematics/AllegroFinger.h"
#include "kinematics/IMAFinger.h"
#include "kinematics/ThumbFinger.h"
#include "kinematics/operations.h"
/*
    Library to move the hand to a certain defined poses.
*/

#include "dynamics_AH.h"
#include "CAN_AH.h"

#include <kdl/chain.hpp>
#include <thread>
#include <mutex>

#include <urdf/model.h>
#include <kdl/frames_io.hpp>
#include "kdl_parser/kdl_parser.hpp"

#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <list>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <iterator>

class AllegroHand: public AllegroHandDriver{
public:

    //! Create a simulated hand to operate
    AllegroHand(std::string _conf_file);
    //! Opens CAN communication and runs control loop with control mode to TORQUE and commanded torque = Zero
    AllegroHand(std::string _conf_file, std::string _pcan_port, unsigned int channel);
    // Empty constructor
    AllegroHand();
    AllegroHand(bool only_conf, std::string _conf_file);
    //! Stops control loop and closes CAN communication
    ~AllegroHand();


    /* Functions related to the current state*/
    // Get current state: configuration [rad], velocity [rad/s], acceleration [rad/s²], torque [Nm]
    void getCurrentState(AllegroHandDriver::Vector &q_, AllegroHandDriver::Vector &vel, AllegroHandDriver::Vector &acc, AllegroHandDriver::Vector &torque);
    // Get current state: configuration [rad]
    void getCurrentState(AllegroHandDriver::Vector &q_);
    // Delete? Get current state printed in deg but stored in rad: configuration [rad], velocity [rad/s], acceleration [rad/s²], torque [Nm]
    void getCurrentStatePrintDeg(AllegroHandDriver::Vector &q_, AllegroHandDriver::Vector &vel, AllegroHandDriver::Vector &acc, AllegroHandDriver::Vector &torque);
    // Get Status
    inline void getStatus(bool &status_){status_ = status;}
    // Get State Error
    void getError(AllegroHandDriver::Vector &pos_error);

    /* Functions related to the controller*/
    // Get the available controllers names
    void getControllersNames(bool print,std::vector<std::string> &controllers_names_);
    void getControllersNames();
    // Get current controller
    std::string getCurrentController() {return current_controller;}
   // Set desired controller
    bool setController(std::string controller);
    // Set controller Torque
    bool setTorqueController();
    // Set controller Gravity Compensator
    bool setGravityController();
    // Set controller PD
    bool setPDController();
    // Set controller PID
    bool setPIDController();
    // Get Gravity Vector (Used to compensate gravity effects)
    void getGravityVector(KDL::Vector &gravity_);
    void getGravityVector(std::array<double, 3> &gravity_vect);

    // Set Hand Gravity Vector (Used to compensate gravity effects)
    void setGravityVector(KDL::Vector &gravity_);
    void setGravityVector(std::array<double, 3> &gravity_vect);

    // Get torque to compensate gravity effects)
    void getGravityComp(AllegroHandDriver::Vector &gc_);

    /* Functions related to moving the hand to a configuration*/
    // Read yaml with the predefined configurations and store them
    bool readPredefinedConfigurations(std::string yaml_file, std::map<std::string, AllegroHandDriver::Vector> &configurations_list_);
    // Get a list with the predefined hand configurations and its values in rad and in degrees.
    void getConfList(bool only_name);
    void getConfList(std::map<std::string, AllegroHandDriver::Vector> &configurations_list);
    // Obtain the configuration values given a predefined configuration name
    void getPredefinedConfigurationValues(std::string conf_name, AllegroHandDriver::Vector &configuration_values);
    // Add a new configuration to the list
    void addConf(std::string conf_name, AllegroHandDriver::Vector configuration, bool temporal_addition);
    // Erase a configuration from the list
    void delConf(std::string conf_name);
    // Store current configuration
    void storeCurrentConf(std::string conf_name, bool temporal_addition);
    // Set desired hand configuration
    bool setDesiredConfiguration(AllegroHandDriver::Vector &q_des_);
    // Set desired finger configuration
    bool setDesiredConfiguration(std::vector<double> q_finger_desired, int finger_index);
    // Move hand to a configuration (recall only available with the PD or PID controller)
    void moveToConf(AllegroHandDriver::Vector conf);
    // Move hand to a configuration with a defined velocity (recall only available with the PD or PID controller)
    void moveToConf(AllegroHandDriver::Vector q_final, double percentage_velocity);
    // Move hand to a configuration with a defined velocity and store the trajectory in a yaml file (recall only available with the PD or PID controller)
    void moveToConfAndSaveTrajectory(AllegroHandDriver::Vector q_final, double percentage_velocity, std::string name_trajectory);
    // Move hand to a configuration in deg (recall only available with the PD or PID controller)
    void moveToConfDeg(AllegroHandDriver::Vector conf_deg) {moveToConf(M_PI/180.*conf_deg);}
    // Move hand to a predefined configuration (recall only available with the PD or PID controller)
    void moveToConf(std::string conf_name);
        // Move to home configuration
        void moveToHomeConf() {moveToConf("home");}
        // Move to rock configuration
        void moveToRockConf() {moveToConf("rock");}
        // Move to paper configuration
        void moveToPaperConf() {moveToConf("paper");}
        // Move to scissors configuration
        void moveToScissorsConf() {moveToConf("scissors");}
        // Move to aloha configuration
        void moveToAlohaConf() {moveToConf("aloha");}
        // Move to punk configuration
        void moveToPunkConf() {moveToConf("punk");}
        // Move to win configuration
        void moveToWinConf() {moveToConf("win");}
        // Move to botifarra configuration
        void moveToBotifarraConf() {moveToConf("botifarra");}
        // Move to thumb_up configuration
        void moveToThumbUpConf() {moveToConf("thumb_up");}
        // Move to one configuration
        void moveToOneConf() {moveToConf("one");}
    // Perform a coupled closing of 5 deg of the IMA (j1,j2,j3) and Thumb (j2, j3)
    void coupledClosing();
    // Perform coupled closing of the IMA (j1,j2,j3) and Thumb (j2, j3)
    void coupledClosing(double coupling_value);
    // Perform coupled closing of all the joints at the desired couple values
    void coupledClosing(std::array<double, n_fingers*finger_n_dofs> coupling_value);
    // Perform coupled closing of each finger the desired value. For the IMA (j1,j2,j3) and Thumb (j2, j3)
    void coupledClosing(std::array<double, n_fingers> coupling_value);


    /*Functions related with trajectories*/
    // Print a list with the names of the predefined hand trajectories
    void getTrajectoriesListNames(std::vector<std::string>& trajectories_list_) ;
    void getTrajectoriesListNames();
    // Print a list with the names of the predefined hand trajectories defined by fingertip poses
    void getTrajectoriesPosesListNames(std::vector<std::string>& trajectories_poses_list_);
    void getTrajectoriesPosesListNames();
    // Move the hand along a defined trajectory
    void moveAlongTrajectory(std::string trajectory_name);
    void moveAlongPosesTrajectory(std::string filename);
    // Move along trajectory with percentage velocity
    void moveAlongTrajectory(AllegroHandDriver::Vector q_initial, AllegroHandDriver::Vector q_final, double percentage_velocity);
    // Move along trajectory with desired velocity in rad (recalculates the whole trajectory)
    void moveAlongTrajectory( std::vector<AllegroHandDriver::Vector> trajectory, double max_velocity_);
    void moveAlongTrajectory( std::string trajectory_name, double max_velocity_);
    // Store the current trajectory. The hand movement is read and the current trajectory and then stores it
    // bool storeCurrentTrajectory(std::vector<AllegroHandDriver::Vector> &trajectory);
    bool storeCurrentTrajectory(std::string name_trajectory, double max_time_threshold_seconds, std::vector<AllegroHandDriver::Vector> &trajectory);
    // bool storeCurrentTrajectory2(std::string name_trajectory, double max_time_threshold_seconds, std::vector<AllegroHandDriver::Vector> &trajectory);
    void stopListeningCurrentTrajectory();

    /* Functions related to predefined grasps*/
    // Move hand along a prismatic 3 fingers grasp
    void moveGraspPrismatic3(double percentage_velocity);
    // Move hand along a prismatic 4 fingers grasp
    void moveGraspPrismatic4(double percentage_velocity);
    // Move hand aalong a grasp with the Pinch with the Index and Thumb finger
    void moveGraspPinchIndexThumb(double percentage_velocity);
    // Move hand along a grasp with the Pinch with the Middle and Thumb finger
    void moveGraspPinchMiddleThumb(double percentage_velocity);

    /* Related with the kinematics*/
    // Forward kinematics
    // Forward kinematics of one fingertip
    bool computeFKFinger(bool print_values, unsigned int finger_number, std::vector<double> q_values_finger, Eigen::Matrix<double,3,1> &eul_angles_solved, Eigen::Matrix<double,3,1> &translation_solved);
    // Forward kinematics of the whole hand (4 fingertips)
    bool computeFK(bool print_values, std::vector<double> q_values_hand, Eigen::Matrix<double,3,4> &eul_angles_solved, Eigen::Matrix<double,3,4> &translation_solved);
    // Forward kinematics of the current configuration of the whole hand (4 fingertips)
    bool computeFK(bool print_values, Eigen::Matrix<double,3,4> &eul_angles_solved, Eigen::Matrix<double,3,4> &translation_solved);

    // Inverse kinematics
    // Computes the inverse kinematics of one fingertip
    bool computeIKFinger(bool print_values, int finger_number, std::vector<double> desired_position, std::vector<double> desired_orientation, std::vector<double> &q_values_finger);
    // bool computeIKFinger(bool print_values, int finger_number, std::vector<double> desired_position, std::vector<double> desired_normal);
    
    // Computes the inverse kinematics of the whole hand
    bool computeIKHand(bool print_values, std::vector<double> desired_position_hand, std::vector<double> desired_orientation_hand, std::vector<double> &q_values_hand);
    // Executes the inverse kinematics. The fingers move to the desired pose
    void executeIK(bool print_values, std::vector<double> desired_position, std::vector<double> desired_orientation);

    // Motion 1 and II
    void calculeMotionHand(unsigned int which_motion, std::vector<double> desired_pos, std::vector<double> desired_orntn);

    // Jacobian
    void ComputeJacobian(const std::vector<double> &joints, const std::vector<double> &cog_x, const std::vector<double> &cog_y);
    void ComputeJacobian(const std::vector<double> &joints, const std::vector<KDL::Frame> &contactpoints);
    void ComputeJacobian(const std::vector<KDL::Frame> &contactpoints, AllegroHandDriver::Vector q_current);
    void getRealJacobianKDL(std::array<Eigen::MatrixXd,n_fingers> &realJacobianKDL);
    Eigen::MatrixXd getRealJacobianKDLAtIndex(int index);
    void getFingerFrames(std::array<std::vector<KDL::Frame>, n_fingers> &finger_frames_);

    /* Related to joint limits*/
    // Get the joint limits and print them
    void getJointLimits();
    void getJointLimits(std::vector<double> &q_min_, std::vector<double> &q_max_);
    // Get the safety margin of the joint limits
    bool getJointLimitsSafetyMargin(std::vector<double> &safety_margin_joint_limits_values_);
    //  Set Joint Limits Safety Margin. Set a percentage for them all
    void setJointLimitsSafetyMargin(double safety_margin_percentage);
    // Set Joint Limits Safety Margin. Set a specific value for them all
    void setJointLimitsSafetyMargin(std::vector<double> safety_margin_joint_limits_values_);
    // Choose whether to use the joint limits with the safety margin or without it
    void useSafetyMargin(bool use_safety_margin_joint_limits_);

// Read yaml with a trajectory configurations defined as list of configurations
    bool readTrajectory(std::string file_name, std::vector<AllegroHandDriver::Vector> &trajectory_values_);

protected:
    // Current controller
    std::string current_controller;
    /* Initialization functions for both the allegro hand and the allegro hand driver */
    // Set up function to initialize the real hand
    void setUp(std::string _conf_file, std::string _pcan_port, unsigned int _channel);
    void setUp(std::string _conf_file, std::string _pcan_port, unsigned int _channel, std::string urdf_string_);
    // Set up function to initialize the simulated hand
    void setUp(std::string _conf_file);
    void setUp(std::string _conf_file, std::string urdf_string_);
    // Calculate a trajectory
    bool calculateTrajectory(std::vector<double> q_initial_hand, std::vector<double> q_final_hand, double percentage_velocity, std::vector<AllegroHandDriver::Vector> &trajectory);
    // Calculate a trajectory from the current configuration
    bool calculateTrajectory(std::vector<double> q_final_hand, double percentage_velocity, std::vector<AllegroHandDriver::Vector> &trajectory);
    // Calculates a new trajectory, slowing down the previous trajectory
    void calculateTrajectory( std::vector<AllegroHandDriver::Vector> &trajectory, double max_velocity_);
    // Calculate the trajectory from the current configuration towards a configuration that sets the fingertips into the desired poses
    bool calculateTrajectoryFromFingertipPoses(std::vector<double> desired_positions, std::vector<double> desired_orientations, double percentage_velocity, std::vector<AllegroHandDriver::Vector> &trajectory);

    // Move along a trajectory
    void moveAlongTrajectory(std::vector<AllegroHandDriver::Vector> trajectory);
    bool moving; // True if it is moving along a trajectory, false otherwise
    void stopMovement(); // Stop movement along a trajectory
    bool isMoving(); // Get wether is moving or not

    // link and joint prefixes of the urdf
    std::string link_prefix, joint_prefix;
private:
    // Initialization of the allegro hand for the constructor
    void setUpAH (bool simulated_hand);
    /* Variables related to the main control Loop*/
    // Previous, current, desired configurations and desired and current torque sent.
    AllegroHandDriver::Vector q_prev, q_current, q_desired;
    // // Mutual exclusive to ensure variables are not written/read at the same time
    // std::mutex mutex;
    // Control Thread that will call the control Loop
    std::thread controlThread;
    // Main control loop. Reads the actual configuration, executes the controller and sends a message with the desired torque towards the hand
    void controlLoop();
    // Fake control loop. Reads and executes the control and simulates sending the torque towards the hand
    void controlLoopFake();
    // Boolean to control wether to run or stop the control Thread
    bool status;

    /* Related to the control*/
    // Pointer to class AllegroHandController
    AllegroHandController* AllegroController;
    // To ensure that we have a control instanciated when we call the desired actions
    std::mutex mutex_controller;
    bool controller_available;
    // Available controllers
    std::vector<std::string> controllers_names;
    // Create controller instance based on the controller name
    AllegroHandController* createController(const std::string& controller_name);
    /* Variables related to reading the urdf and building up a KDL tree */
    KDL::Tree tree;
    //!KDL chain with virtual joints for FK
    std::array<KDL::Chain,n_fingers> chain;
    //!KDL chain without virtual joints for IK
    std::array<KDL::Chain,n_fingers> finger_chain;
    // Torque conversion given by the manufacturer (defined in CAN communications)
    double torqueConv;

    /* Related to the defined configurations*/
    // Available configurations. Name and configuration values in rad.
    std::map<std::string, AllegroHandDriver::Vector> configurations_list;
    // List of trajectories names
    std::vector<std::string> trajectories_list;
    // List of names of the available trajectories defined by the poses (position + orientation) of the fingertips
    std::vector<std::string> trajectories_poses_list;
    // name of the folder trajectories
    std::string name_folder_trajectories = "trajectories";
    // boolean that is used when storing a trajectory while moving the current hand
    bool listening_to_trajectory_to_store;
    // Yaml file with predefined configurations
    std::string config_yaml_file;
    // Yaml file with predefined configurations
    std::string config_yaml_file_opposite_hand;
    // Define a list of configurations (does not read from the yaml file)
    void setConfList();
    // Write configurations into the configuration yaml file.
    bool writeConfiguration(std::string yaml_file, std::string name_config, AllegroHandDriver::Vector values_config);
    // Write trajectory (list configurations values) into the trajectories folder in a yaml file with the name of the trajectory.
    bool writeTrajectories(bool store_fk_trajectory, bool is_left_, std::string name_trajectory,  std::vector<AllegroHandDriver::Vector> values_trajectory);
    // Write the poses of a trajectory (list of coordinates and euler angles) onto a Yaml file.
    void writePosesTrajectory(const std::vector<Eigen::Matrix<double, 3, 4>>& eulAngles, const std::vector<Eigen::Matrix<double, 3, 4>>& translations, const std::string& filename);
    bool ConvertTrajectoryFromConfigurationToPoses(std::vector<AllegroHandDriver::Vector> trajectory, std::vector<Eigen::Matrix<double, 3, 4>> &eulAnglesSolved, std::vector<Eigen::Matrix<double, 3, 4>> &translationsSolved);

    // Calculate the configs so that the oposite hand moves to the same position
    void getConfigsForTheOppositeHand(AllegroHandDriver::Vector config_values_actual, AllegroHandDriver::Vector &config_values_opposite_hand);

    /* Related to reading YAML files */
    // Read files from folder
    void getFileNamesFromFolder(std::string folder_name, std::vector<std::string> &file_names_list);
    // Read the urdf file and obtain a tree and the limits
    bool readConfUrdf();
    // Obtain list of available controllers from folder /conf/controllers
    bool searchControllers();
    // Search and read all the predefined configurations from all the files in the conf/configurations.
    bool searchPredefinedConfigurations();
    // Read all the trajectories from the conf/trajectories/left (or /right) folder and store the trajectory names in the std::vector<std::string> trajectories_list variable.
    void readTrajectories();

    // Read yaml with a trajectory configurations  defined as list of poses of fingertips
    bool readPosesTrajectory(const std::string file_name, std::vector<Eigen::Matrix<double, 3, 4>>& eulAngles, std::vector<Eigen::Matrix<double, 3, 4>>& translations);


    /* Related to kinematics */
    // Lengths of each link. We extract this from the urdf file.
    std::vector<double> lengths_links_finger_I;
    std::vector<double> lengths_links_finger_M;
    std::vector<double> lengths_links_finger_A;
    std::vector<double> lengths_links_finger_Thumb;

    // Instance of each finger
    IMAFinger* index_finger;
    IMAFinger* middle_finger;
    IMAFinger* anular_finger;
    ThumbFinger* thumb_finger;

    // Verify that the configurations are within the available limits and change them
    bool checkAndAjustJointConfigurationValues(AllegroHandDriver::Vector &q_values);
    // Verify that configuration are within the URDF limits.
    bool checkJointConfigurationValuesWithinLimits(std::vector<double> q_values_hand);

    void calculeMotion1Finger(unsigned int finger_number, std::vector<double> desired_pos, std::vector<double> desired_orntn, std::vector<std::vector<double>> &motion1_sols_finger);
    void calculeMotion2Finger(unsigned int finger_number, std::vector<double> desired_pos, std::vector<double> desired_orntn, std::vector<std::vector<double>> &motion2_sols_finger);

    /* Related to joint limits*/
    // Joint limits obtained from the urdf (actual joint limits of the hand)
    std::vector<double> q_min_urdf, q_max_urdf, safety_margin_joint_limits_values;
    // Real joint limit values obtained from the conf/joints_limit folder
    std::vector<double> q_min_real, q_max_real;
    // URDF joint limits avoiding saturated values compared to the real joint limits
    std::vector<double> q_min_urdf_unsaturated, q_max_urdf_unsaturated;
    bool use_safety_margin_joint_limits = false;
    // Update the values of the joint limits
    void updateJointLimits();
    // Check that the joint limit values are within the limits of the URDF unsaturated values
    void checkNewJointLimitsWithinJointLimits(std::vector<double> &q_min_, std::vector<double> &q_max_);
    // Read joint limits from a yaml file in conf/joints_limits
    bool readJointLimits();
    // Update joint limits URDF to avoid saturated values regarding the real joint limits
    void updateJointLimitsURDF();

};

#endif // ALLEGROHANDBASIC_H_
