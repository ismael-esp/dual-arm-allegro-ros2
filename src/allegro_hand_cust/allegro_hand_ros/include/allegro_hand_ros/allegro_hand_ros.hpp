#ifndef ALLEGRO_HAND_ROS_H
#define ALLEGRO_HAND_ROS_H

// ROS 2
#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

// #include <geometry_msgs/msg/quaternion.h>
// #include <visualization_msgs/msg/marker_array.h>
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_kdl/tf2_kdl.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp> //Find a package (used for reading the xacro)

// #include <std_msgs/msg/float64.h>

#include <memory>
#include <iostream>     // Include this header for std::cout, std::cin, std::endl...
#include <cmath>        // Include this header for std::sqrt, std::sin...
#include <memory>       // Include this header for std::make_shared, std::make_unique...
#include <chrono>       // Include this header for std::chrono
#include <array>        // Include this header for std::array
#include <vector>       // Include this header for std::vector
#include <algorithm>    // Include this header for std::copy...

// Include services from package allegro_hand_interfaces
#include "allegro_hand_interfaces/srv/get_controller.hpp"
#include "allegro_hand_interfaces/srv/get_gravity_vector.hpp"
#include "allegro_hand_interfaces/srv/get_fingers_movement_status.hpp"
#include "allegro_hand_interfaces/srv/get_hand_configuration.hpp"
#include "allegro_hand_interfaces/srv/get_pose_fk.hpp"
#include "allegro_hand_interfaces/srv/get_torque.hpp"
#include "allegro_hand_interfaces/srv/joint_limits.hpp"
#include "allegro_hand_interfaces/srv/coupled_closing.hpp"
#include "allegro_hand_interfaces/srv/set_controller.hpp"
#include "allegro_hand_interfaces/srv/set_des_pos.hpp"
#include "allegro_hand_interfaces/srv/set_des_pos_vel.hpp"
#include "allegro_hand_interfaces/srv/set_des_torque.hpp"
#include "allegro_hand_interfaces/srv/set_gravity_vector.hpp"
#include "allegro_hand_interfaces/srv/set_pose_ik.hpp"
#include "allegro_hand_interfaces/srv/stop_hand.hpp"
#include "allegro_hand_interfaces/srv/pause_finger.hpp"
#include "allegro_hand_interfaces/srv/continue_finger.hpp"
#include "allegro_hand_interfaces/srv/stop_finger.hpp"

// Include actions from package allegro_hand_interfaces
#include <allegro_hand_interfaces/action/move_config.hpp>
#include <allegro_hand_interfaces/action/move_config_finger.hpp>
#include <allegro_hand_interfaces/action/move_fingertip_poses.hpp>
#include <allegro_hand_interfaces/action/move_fingertip_pose.hpp>
#include <allegro_hand_interfaces/action/move_trajectory.hpp>
#include <allegro_hand_interfaces/msg/trajectory.hpp>
#include <allegro_hand_interfaces/action/move_trajectory_vel.hpp>

// Include allegro hand class
#include "allegro_hand/AllegroHand.h"


class AllegroRos: public AllegroHand{
public:

    AllegroRos(rclcpp::Node::SharedPtr _node, const std::string& _prefix);

    // Related with the ros loop function
    void loop_function();
    void start_loop_function();
    void stop_loop_function();

    // // Tactile data
    std::vector<bool> tactileData;
    std::vector<double> cog_x, cog_y;
    std::vector<double> force;
    std::vector<double> area;

    // Print the current controller name
    bool getController();

    bool setDesiredConfiguration(std::array<double, n_fingers * finger_n_dofs> &goal_pos);
    // bool setDesiredTorque(std::array<double, n_fingers * finger_n_dofs> &goal_torque);
    // void setGravityVector(std::array<double, 3> &gravity_vect);

    /* ROS SERVICES */ 
    bool getControllerSrv(const std::shared_ptr<allegro_hand_interfaces::srv::GetController::Request> req,
                             std::shared_ptr<allegro_hand_interfaces::srv::GetController::Response> res);

    bool setControllerSrv(const std::shared_ptr<allegro_hand_interfaces::srv::SetController::Request> req,
                              std::shared_ptr<allegro_hand_interfaces::srv::SetController::Response> res);

    bool setDesiredConfigurationSrv(const std::shared_ptr<allegro_hand_interfaces::srv::SetDesPos::Request> req,
                                std::shared_ptr<allegro_hand_interfaces::srv::SetDesPos::Response> res);

    bool stopHandSrv(const std::shared_ptr<allegro_hand_interfaces::srv::StopHand::Request>  /*req*/,
                        std::shared_ptr<allegro_hand_interfaces::srv::StopHand::Response> res);

    bool getLimitsSrv(const std::shared_ptr<allegro_hand_interfaces::srv::JointLimits::Request>  /*req*/,
                         std::shared_ptr<allegro_hand_interfaces::srv::JointLimits::Response> res);

    bool setPoseIKSrv(const std::shared_ptr<allegro_hand_interfaces::srv::SetPoseIK::Request> req,
                            std::shared_ptr<allegro_hand_interfaces::srv::SetPoseIK::Response> res);
    
    bool getPoseFKSrv(const std::shared_ptr<allegro_hand_interfaces::srv::GetPoseFK::Request> req,
                            std::shared_ptr<allegro_hand_interfaces::srv::GetPoseFK::Response> res);
    
    bool getTorqueSrv(const std::shared_ptr<allegro_hand_interfaces::srv::GetTorque::Request> req,
                            std::shared_ptr<allegro_hand_interfaces::srv::GetTorque::Response> res);
    
    bool setTorqueSrv(const std::shared_ptr<allegro_hand_interfaces::srv::SetDesTorque::Request> req,
                            std::shared_ptr<allegro_hand_interfaces::srv::SetDesTorque::Response> res);
    
    bool getGravityVectorSrv(const std::shared_ptr<allegro_hand_interfaces::srv::GetGravityVector::Request> req,
                            std::shared_ptr<allegro_hand_interfaces::srv::GetGravityVector::Response> res);
    
    bool setGravityVectorSrv(const std::shared_ptr<allegro_hand_interfaces::srv::SetGravityVector::Request> req,
                                   std::shared_ptr<allegro_hand_interfaces::srv::SetGravityVector::Response> res);

    bool coupledClosingSrv(const std::shared_ptr<allegro_hand_interfaces::srv::CoupledClosing::Request> req,
                                 std::shared_ptr<allegro_hand_interfaces::srv::CoupledClosing::Response> res); 
    
    bool getHandConfigurationSrv(const std::shared_ptr<allegro_hand_interfaces::srv::GetHandConfiguration::Request> req,
                                 std::shared_ptr<allegro_hand_interfaces::srv::GetHandConfiguration::Response> res); 

    bool getFingersMovementStatusSrv(const std::shared_ptr<allegro_hand_interfaces::srv::GetFingersMovementStatus::Request> req,
                                 std::shared_ptr<allegro_hand_interfaces::srv::GetFingersMovementStatus::Response> res); 
    
    bool pauseFingerSrv(const std::shared_ptr<allegro_hand_interfaces::srv::PauseFinger::Request> req,
                                 std::shared_ptr<allegro_hand_interfaces::srv::PauseFinger::Response> res); 
    
    bool continueFingerSrv(const std::shared_ptr<allegro_hand_interfaces::srv::ContinueFinger::Request> req,
                                 std::shared_ptr<allegro_hand_interfaces::srv::ContinueFinger::Response> res); 
    
    bool stopFingerSrv(const std::shared_ptr<allegro_hand_interfaces::srv::StopFinger::Request> req,
                                 std::shared_ptr<allegro_hand_interfaces::srv::StopFinger::Response> res); 
    

/* Check: setDesiredPosVelSrv are unused*/
    // bool setDesiredPosVelSrv(const std::shared_ptr<allegro_hand_interfaces::srv::SetDesPosVel::Request> req,
    //                             std::shared_ptr<allegro_hand_interfaces::srv::SetDesPosVel::Response> res);

    void setCommandHand(const sensor_msgs::msg::JointState& msg);



private:
    // Node
    rclcpp::Node::SharedPtr node_;
    // Parameters (we read them from the launch file)
    bool slave;
    const std::string prefix;
    std::string tip;

    // Function to convert Xacro into urdf 
    std::string xacroToString();
    // Function to set up all the services
    void setUpServices();
    // rclcpp::Node::SharedPtr node_;
    std::mutex mutex_ros;
    // std::thread publishThread;
    std::thread ros_thread;

    //Stop ros loop thread or not
    bool stop_ros_thread = false; 
    bool commandUpdate = false;
    // bool tactileWeiss = false;

    // Publisher jointStates
    sensor_msgs::msg::JointState jointStatesMsg;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_commanded;

    // // Publisher force torques arrows
    // visualization_msgs::MarkerArray forceTorquesMsg;
    // ros::Publisher forceTorques_pub;

    // // Publisher force torques arrows
    // visualization_msgs::MarkerArray realForceTorquesMsg;
    // ros::Publisher realForceTorques_pub;

    // Subscriber to command the hand
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr commandHand;

    /* Services */
    // Get current controller
    rclcpp::Service<allegro_hand_interfaces::srv::GetController>::SharedPtr get_controller_srv;
    // Set controller
    rclcpp::Service<allegro_hand_interfaces::srv::SetController>::SharedPtr set_controller_srv;
    // Move to a hand configuration
    rclcpp::Service<allegro_hand_interfaces::srv::SetDesPos>::SharedPtr position_srv;
    // Stop hand
    rclcpp::Service<allegro_hand_interfaces::srv::StopHand>::SharedPtr stop_srv;
    // Get joint limits
    rclcpp::Service<allegro_hand_interfaces::srv::JointLimits>::SharedPtr getLimits_srv;
    // Move to the desired fingertip position and orientation
    rclcpp::Service<allegro_hand_interfaces::srv::SetPoseIK>::SharedPtr set_pose_ik_srv;
    // Get fingertips positions and orientations
    rclcpp::Service<allegro_hand_interfaces::srv::GetPoseFK>::SharedPtr get_pose_fk_srv;
    // Get current controller
    rclcpp::Service<allegro_hand_interfaces::srv::GetTorque>::SharedPtr get_torque_srv;
    // Set controller
    rclcpp::Service<allegro_hand_interfaces::srv::SetDesTorque>::SharedPtr set_torque_srv;
    // Get gravity vector
    rclcpp::Service<allegro_hand_interfaces::srv::GetGravityVector>::SharedPtr get_gravity_vector_srv;
    // Set controller
    rclcpp::Service<allegro_hand_interfaces::srv::SetGravityVector>::SharedPtr set_gravity_vector_srv;
    // Coupled closing
    rclcpp::Service<allegro_hand_interfaces::srv::CoupledClosing>::SharedPtr coupled_closing_srv;
    // Get current configuration
    rclcpp::Service<allegro_hand_interfaces::srv::GetHandConfiguration>::SharedPtr get_hand_configuration_srv;
    // Get wether each finger is moving or not
    rclcpp::Service<allegro_hand_interfaces::srv::GetFingersMovementStatus>::SharedPtr get_fingers_movement_status_srv;
    // Pause finger
    rclcpp::Service<allegro_hand_interfaces::srv::PauseFinger>::SharedPtr pause_finger_srv;
    // Continue finger
    rclcpp::Service<allegro_hand_interfaces::srv::ContinueFinger>::SharedPtr continue_finger_srv;
    // Stop finger
    rclcpp::Service<allegro_hand_interfaces::srv::StopFinger>::SharedPtr stop_finger_srv;


    /* TODO Check: setDesiredPosVelSrv and setDesiredTorqueSrv are unused*/
    // ros::ServiceServer positionReflexxes_srv;
    // ros::ServiceServer torque_srv;

    // AllegroHand::Vector desired_pos;
    // AllegroHand::Vector desired_vel;
    // AllegroHand::Vector desired_torque;
    // AllegroHand::Vector pos, vel, acc, torque;
    // AllegroHand::Vector torque_gc;
    // KDL::Vector         gravity_vector;

    // // Variables to compute force using jacobian and torques
    // std::array<Eigen::MatrixXd,AllegroHand::n_fingers> JTI_kdl;
    
    //wrenchKDL;
    // std::array<Eigen::MatrixXd,AllegroHand::n_fingers> wrenchKDL;
    // Create a std::array of 4 geometry_msgs::Wrench and wrenchStamped
    std::array<geometry_msgs::msg::Wrench, 4> wrench_array;
    std::array<geometry_msgs::msg::WrenchStamped, 4> wrench_stamped_array;

    // // Create four publishers for geometry_msgs::WrenchStamped
    std::array<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr, 4> wrench_pub;

    // Create array for the tip names
    std::array<std::string, 4> tip_names;

    // // Variables to compute force using jacobian and torques
    // std::array<Eigen::MatrixXd,AllegroHand::n_fingers> realJTI_kdl;
    std::array<Eigen::MatrixXd,AllegroHand::n_fingers> realJ_kdl;
    //wrenchKDL;
    std::array<Eigen::MatrixXd,AllegroHand::n_fingers> realWrenchKDL;

    // Name space of the node 
    std::string name_space_node, name_space, handed;

    // // broadcaster to publish the tf of the contact point.
    // // z goes perpendicular to the sensor tip
    tf2::Transform contact_frame;
    // std::array<std::unique_ptr<tf2_ros::TransformBroadcaster>, 4> br_contact_frame;
    std::unique_ptr<tf2_ros::TransformBroadcaster> br_contact_frame;

    std::array<geometry_msgs::msg::TransformStamped, AllegroHand::n_fingers > contact_point_frame_stamped;

    rclcpp::Rate rate;


    /* Related to Actions */

    // Action MoveConfig
    // Move to a given configuration at a speed based on the desired percentage speed
    using MoveConfig = allegro_hand_interfaces::action::MoveConfig;
    rclcpp_action::Server<MoveConfig>::SharedPtr move_config_action_server;
    rclcpp_action::GoalResponse handle_goal_move_config(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const MoveConfig::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel_move_config(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveConfig>> goal_handle);
    void handle_accepted_move_config(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveConfig>> goal_handle);
    void execute_move_config(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveConfig>> goal_handle);

    // Action MoveFingertipPoses
    // Move the fingertips to the desired poses at a speed based on the desired percentage speed
    using MoveFingertipPoses = allegro_hand_interfaces::action::MoveFingertipPoses;
    rclcpp_action::Server<MoveFingertipPoses>::SharedPtr move_fingertip_poses_action_server;
    rclcpp_action::GoalResponse handle_goal_move_fingertip_poses(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const MoveFingertipPoses::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel_move_fingertip_poses(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveFingertipPoses>> goal_handle);
    void handle_accepted_move_fingertip_poses(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveFingertipPoses>> goal_handle);
    void execute_move_fingertip_poses(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveFingertipPoses>> goal_handle);

    // Action MoveTrajectory
    using MoveTrajectory = allegro_hand_interfaces::action::MoveTrajectory;
    rclcpp_action::Server<MoveTrajectory>::SharedPtr move_trajectory_action_server;
    rclcpp_action::GoalResponse handle_goal_move_trajectory(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const MoveTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel_move_trajectory(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveTrajectory>> goal_handle);
    void handle_accepted_move_trajectory(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveTrajectory>> goal_handle);
    void execute_move_trajectory(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveTrajectory>> goal_handle);

    // Action MoveTrajectory using the FollowJointTrajectory from the control action
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr follow_joint_trajectory_action_server;
    rclcpp_action::GoalResponse handle_goal_follow_joint_trajectory(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const FollowJointTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel_follow_joint_trajectory(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle);
    void handle_accepted_follow_joint_trajectory(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle);
    void execute_follow_joint_trajectory(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle);

    // Action MoveTrajectoryMaxVel
    using MoveTrajectoryMaxVel = allegro_hand_interfaces::action::MoveTrajectoryVel;
    rclcpp_action::Server<MoveTrajectoryMaxVel>::SharedPtr move_trajectory_vel_action_server;
    rclcpp_action::GoalResponse handle_goal_move_trajectory_vel(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const MoveTrajectoryMaxVel::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel_move_trajectory_vel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveTrajectoryMaxVel>> goal_handle);
    void handle_accepted_move_trajectory_vel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveTrajectoryMaxVel>> goal_handle);
    void execute_move_trajectory_vel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveTrajectoryMaxVel>> goal_handle);

   // Move a finger to a given configuration at a speed based on the desired percentage speed
    using MoveConfigFinger = allegro_hand_interfaces::action::MoveConfigFinger;
    rclcpp_action::Server<MoveConfigFinger>::SharedPtr move_config_finger_action_server;
    rclcpp_action::GoalResponse handle_goal_move_config_finger(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const MoveConfigFinger::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel_move_config_finger(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveConfigFinger>> goal_handle);
    void handle_accepted_move_config_finger(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveConfigFinger>> goal_handle);
    void execute_move_config_finger(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveConfigFinger>> goal_handle);

    // Move one fingertip to the desired pose at a speed based on the desired percentage speed
    using MoveFingertipPose = allegro_hand_interfaces::action::MoveFingertipPose;
    rclcpp_action::Server<MoveFingertipPose>::SharedPtr move_fingertip_pose_action_server;
    rclcpp_action::GoalResponse handle_goal_move_fingertip_pose(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const MoveFingertipPose::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel_move_fingertip_pose(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveFingertipPose>> goal_handle);
    void handle_accepted_move_fingertip_pose(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveFingertipPose>> goal_handle);
    void execute_move_fingertip_pose(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveFingertipPose>> goal_handle);



    /*Related to the current trajectory we are moving*/
    // Current trajectory where the hand is moving
    std::vector<AllegroHandDriver::Vector> current_trajectory;
    // Number of the current configuration along the current_trajectory list
    int num_current_configuration;
    // Number of points in the current_trajectory list
    unsigned int num_points;
    // Mutex to access everything related to the current_trajectory
    std::mutex mutex_current_trajectory;
    // Array to store wether each finger is paused/stopped (0) or moving (1).
    std::array<bool, n_fingers> finger_moving;
    // When a finger is paused the future joint values are stored
    std::vector<AllegroHandDriver::Vector> stored_trajectory;
    // To store only once each finger trajectory, once stored to allow multiple stops we will store the index of the stop
    std::array<int, n_fingers> paused_index_stored_trajectory, continue_index_current_trajectory;

    // Setting initial finger movement status. Set to moving or not depending on the initial and final configuration of the trajectory
    void setFingersInitialMovementStatus(AllegroHandDriver::Vector initial_configuration, AllegroHandDriver::Vector final_configuration);
    // Get whether each finger is moving or not
    void getFingerMovementStatus(std::array<bool, n_fingers> &finger_moving_);
    // Mutex to access if a finger is moving or not
    std::mutex mutex_finger_moving;
};

#endif
