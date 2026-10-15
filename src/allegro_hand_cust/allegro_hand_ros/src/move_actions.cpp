#include "allegro_hand_ros/allegro_hand_ros.hpp"

/* Related to moving the hand */

//! \brief Function to only set as moving the fingers whon initial and final configuration differ more than a threshold 
void AllegroRos::setFingersInitialMovementStatus(AllegroHandDriver::Vector initial_configuration, AllegroHandDriver::Vector final_configuration){
    const double joint_threshold = 0.1;
    double joint_difference;
    // We start by setting all fingers as they will not move
    mutex_finger_moving.lock();
        this->finger_moving.fill(false);
        for (unsigned int i = 0; i<n_fingers; i++){
            for (unsigned int ii = 0; ii<finger_n_dofs; ii++){
                // If the difference in one joint is bigger than the threshold we consider that the finger will move
                joint_difference = abs(final_configuration(i*n_fingers + ii) - initial_configuration((i*n_fingers + ii)));
                if(joint_difference>joint_threshold){
                    this->finger_moving[i] = true;
                    break;
                }
            }
        }
    mutex_finger_moving.unlock();
    std::ostringstream moving_fingers;
    moving_fingers << "Fingers movement status: ";
    for (unsigned int i = 0; i < n_fingers; i++) {
        moving_fingers << "Finger " << i << ": " << (this->finger_moving[i] ? "true" : "false");
        if (i < n_fingers - 1) {
            moving_fingers << ", ";
        }
    }
    RCLCPP_INFO(node_->get_logger(), moving_fingers.str().c_str());
}

void AllegroRos::getFingerMovementStatus(std::array<bool, n_fingers> &finger_moving_){
    mutex_finger_moving.lock();
        finger_moving_ = finger_moving;
    mutex_finger_moving.unlock();
}

/* Move to a configuration at the desired velocity percentage */
    //! \brief Callback function for handling goal of MoveConfig. 
    //! We will reject if the size of the configurations do not match the number of joints of the hand
    rclcpp_action::GoalResponse AllegroRos::handle_goal_move_config(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveConfig::Goal> goal)
    {
        RCLCPP_INFO(node_->get_logger(), "Received goal request for the hand %s", this->hand_name.c_str());
        (void)uuid;
    
        // If the number of joints of the trajectory is correct
        if (goal->final_configuration.size() != (this->n_fingers * this->finger_n_dofs)) {
            RCLCPP_WARN(node_->get_logger(), "Received joint configuration size is not valid. Size: %zu", goal->final_configuration.size());
            return rclcpp_action::GoalResponse::REJECT;
        }
        // If we are already moving the goal gets rejected
        if (this->moving) {
            RCLCPP_WARN(node_->get_logger(), "MoveConfig action cannot be executed, because there is another movement in place.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse AllegroRos::handle_cancel_move_config(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveConfig>> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void AllegroRos::handle_accepted_move_config(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveConfig>> goal_handle)
    {
        // If we are using the Torque or the GravityCompensator controller, we set up the PID.
        if ((current_controller == "Torque") || (current_controller == "GravityCompensator")){
            std::cout << "Current controller: " << current_controller << std::endl;
            std::cout << "In order to move to a desired configuration, the PD or PID controller is needed.";
            setController("JointPositionPID");
        }
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&AllegroRos::execute_move_config, this, std::placeholders::_1), goal_handle}.detach();
    }

    //! \brief execute function to move at the desired speed using ros to control the time
    void AllegroRos::execute_move_config(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveConfig>> goal_handle){
        RCLCPP_INFO(node_->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveConfig::Feedback>();
        auto & feedback_current_configuration = feedback->current_configuration;
        // Fill in the joint names
        for (char i = '0'; i <= '3'; ++i) {
            for (char j = '0'; j <= '3'; ++j) {
                feedback_current_configuration.name.push_back("j" + std::string(1, i) + std::string(1, j));
            }
        }
        int index = 0;
        // Set the size of the position, velocity and torque
        feedback_current_configuration.position.resize(n_fingers * finger_n_dofs);
        feedback_current_configuration.velocity.resize(n_fingers * finger_n_dofs);
        feedback_current_configuration.effort.resize(n_fingers * finger_n_dofs);
        auto result = std::make_shared<MoveConfig::Result>();
        // control_period*1000000
        // rclcpp::Rate loop_rate(1000000);
        // auto t_start = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double, std::micro> t_sample(control_period);
        
        std::vector<AllegroHandDriver::Vector> trajectory;
        AllegroRos::Vector current_desired_pos_vec, current_conf, current_vel, current_acc, current_torque;
        // Alternative Option. Use AllegroHand function class (the time is controlled there)
            // moveToConf(goal->final_configuration, goal->percentage_velocity);
        // Calculate the trajectory to arrive to the final configuration
        if(calculateTrajectory(goal->final_configuration, goal->percentage_velocity, trajectory)){
            // Store trajectory in the global variable current trajectory
            mutex_current_trajectory.lock();
                num_points = trajectory.size();
                current_trajectory = trajectory;
            mutex_current_trajectory.unlock();
            this->moving = true;
            setFingersInitialMovementStatus(current_trajectory.front(), current_trajectory.back());
            this->paused_index_stored_trajectory.fill(-1);
            RCLCPP_INFO(node_->get_logger(), "Num points: %u", num_points);
            
            // For each configuration in the trajectory we have:
            for (unsigned int i = 0; (i < num_points) && rclcpp::ok(); ++i) {
                // std::cout << "point: "<< i << " / " << num_points << std::endl;
                // Check if there is a cancel request
                if (goal_handle->is_canceling()) {
                    this->moving = false;
                    mutex_finger_moving.lock();
                        this->finger_moving.fill(false);
                    mutex_finger_moving.unlock();
                    result->status = false;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(node_->get_logger(), "MoveConf action canceled");
                    stored_trajectory.clear();
                    return;
                }

                // Move the hand to the ith configuration
                std::array<double, n_fingers * finger_n_dofs> current_desired_pos;
                mutex_current_trajectory.lock();
                    current_desired_pos_vec = current_trajectory[i];
                mutex_current_trajectory.unlock();
                for (int ii=0; ii < n_fingers * finger_n_dofs; ii ++){
                    current_desired_pos[ii] =  current_desired_pos_vec(ii);
                }

                // Alternative if feedback_current_configuration is a std::vector
                // std::vector<double> current_conf_vector;
                if(setDesiredConfiguration(current_desired_pos)){
                    // Store the number of trajectory we are in
                    mutex_current_trajectory.lock();
                        num_current_configuration = i;
                    mutex_current_trajectory.unlock();
                    // Get current State
                    getCurrentState(current_conf, current_vel, current_acc, current_torque);
                    // Populate the feedback with the actual current state
                    for (int ii=0; ii < n_fingers * finger_n_dofs; ii ++){
                        // Populate the feedback header:
                        feedback_current_configuration.header.stamp = rclcpp::Clock().now();
                        feedback_current_configuration.header.frame_id =  std::to_string(index);
                        // Populate the feedback position, velocity and torque
                        feedback_current_configuration.position[ii] = current_conf(ii);
                        feedback_current_configuration.velocity[ii] = current_vel(ii);
                        feedback_current_configuration.effort[ii] = current_torque(ii);
                        // Alternative if feedback_current_configuration is a std::vector
                        // current_conf_vector.push_back(current_conf(ii));
                    }
                    // Alternative if feedback_current_configuration is a std::vector
                    // feedback_current_configuration = current_conf_vector;
                    index ++;
                }
                else{
                    RCLCPP_INFO(node_->get_logger(), "Error in setDesiredConfiguration()");  
                    return;
                }
                // Publish feedback
                goal_handle->publish_feedback(feedback);
                // RCLCPP_INFO(node_->get_logger(), "Publish feedback");
                // Time delay.
                // loop_rate.sleep();
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*1000)));
            }
            RCLCPP_INFO(node_->get_logger(), "Waiting... ");
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*500000)));
            this->moving = false;
            mutex_finger_moving.lock();
                this->finger_moving.fill(false);
            mutex_finger_moving.unlock();
            // Check if goal is done
            if (rclcpp::ok()) {
                // Define maximum diferences for each joint and for them all
                double max_difference_threshold = 0.05; // rad
                double max_additive_difference_threshold = 0.5; //rad
                bool all_within_threshold = true;
                double additive_difference = 0.0;  
                // Read the current hand configuration
                getCurrentState(current_conf);
                std::array<double, n_fingers * finger_n_dofs> joint_difference;
                // Calculate the difference between the desired goal configuration and the final hand configuration
                for (size_t q = 0; q < n_fingers * finger_n_dofs; q++) {
                    joint_difference[q] = abs(goal->final_configuration[q] - current_conf(q));
                    additive_difference += joint_difference[q];
                    if (joint_difference[q] > max_difference_threshold) {
                        all_within_threshold = false;
                    }
                    RCLCPP_INFO(node_->get_logger(), "Joint %zu difference: %f", q, joint_difference[q]);
                }
                RCLCPP_INFO(node_->get_logger(), "Total additive difference: %f", additive_difference);
                // If the difference is lower than the maximum stablished thresholds success is true, false otherwise.
                if (all_within_threshold && additive_difference <= max_additive_difference_threshold) {
                    result->status = true;
                    RCLCPP_INFO(node_->get_logger(), "MoveConf finished successfully!");
                } else {
                    result->status = false;
                    RCLCPP_INFO(node_->get_logger(), "MoveConf finished");
                }
                goal_handle->succeed(result);
                RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
            }
            stored_trajectory.clear();
        }
        else{
            // Error when computing the trajectory
            RCLCPP_WARN(node_->get_logger(), "Error in MoveConf when computing the trajectory");
        }
    }


/* Move fingertips to poses at the desired velocity percentage */
    //! \brief Callback function for handling goal of MoveFingertipPoses. 
    //! We will reject if the size of the configurations do not match the number of joints of the hand
    rclcpp_action::GoalResponse AllegroRos::handle_goal_move_fingertip_poses(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveFingertipPoses::Goal> goal)
    {
        RCLCPP_INFO(node_->get_logger(), "Received goal request for the hand %s", this->hand_name.c_str());
        (void)uuid;
    
        // If the number of euler_angles is correct
        if (goal->euler_angles.size() != (this->n_fingers * 3)) {
            RCLCPP_WARN(node_->get_logger(), "Received ZYZ Euler Angles size is not valid. Size: %zu", goal->euler_angles.size());
            return rclcpp_action::GoalResponse::REJECT;
        }
        // If the number of coordinates is correct
        if (goal->coordinates.size() != (this->n_fingers)) {
            RCLCPP_WARN(node_->get_logger(), "Received coordinates (X,Y,Z) size is not valid. Size: %zu", goal->coordinates.size());
            return rclcpp_action::GoalResponse::REJECT;
        }
        // If we are already moving the goal gets rejected
        if (this->moving) {
            RCLCPP_WARN(node_->get_logger(), "MoveFingertipPoses action cannot be executed, because there is another movement in place.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse AllegroRos::handle_cancel_move_fingertip_poses(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveFingertipPoses>> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void AllegroRos::handle_accepted_move_fingertip_poses(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveFingertipPoses>> goal_handle)
    {
        // If we are using the Torque or the GravityCompensator controller, we set up the PID.
        if ((current_controller == "Torque") || (current_controller == "GravityCompensator")){
            std::cout << "Current controller: " << current_controller << std::endl;
            std::cout << "In order to move to a desired configuration, the PD or PID controller is needed.";
        setController("JointPositionPID");
        }
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&AllegroRos::execute_move_fingertip_poses, this, std::placeholders::_1), goal_handle}.detach();
    }

    //! \brief execute function to move at the desired speed using ros to control the time
    void AllegroRos::execute_move_fingertip_poses(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveFingertipPoses>> goal_handle){
        RCLCPP_INFO(node_->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveFingertipPoses::Feedback>();
        auto & feedback_current_configuration = feedback->current_configuration;
        // Fill in the joint names
        int index = 0;
        for (char i = '0'; i <= '3'; ++i) {
            for (char j = '0'; j <= '3'; ++j) {
                feedback_current_configuration.name.push_back("j" + std::string(1, i) + std::string(1, j));
                ++index;
            }
        }
        index = 0;
        // Set the size of the position, velocity and torque
        feedback_current_configuration.position.resize(n_fingers * finger_n_dofs);
        feedback_current_configuration.velocity.resize(n_fingers * finger_n_dofs);
        feedback_current_configuration.effort.resize(n_fingers * finger_n_dofs);
        auto result = std::make_shared<MoveFingertipPoses::Result>();
        // control_period*1000000
        // rclcpp::Rate loop_rate(1000000);
        // auto t_start = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double, std::micro> t_sample(control_period);
        std::vector<AllegroHandDriver::Vector> trajectory;
        AllegroRos::Vector current_desired_pos_vec, current_conf, current_vel, current_acc, current_torque;
        
        // Store the coordinates of the fingertips
        std::vector<double> desired_positions;
        for (const auto& point : goal->coordinates) {
            desired_positions.push_back(point.x);
            desired_positions.push_back(point.y);
            desired_positions.push_back(point.z);
        }

        // Calculate the trajectory to arrive to the final configuration
        if(calculateTrajectoryFromFingertipPoses(desired_positions, goal->euler_angles, goal->percentage_velocity, trajectory)){
            // Store trajectory in the global variable current trajectory
            mutex_current_trajectory.lock();
                current_trajectory = trajectory;
                num_points = trajectory.size();
            mutex_current_trajectory.unlock();
            this->moving = true;
            setFingersInitialMovementStatus(current_trajectory.front(), current_trajectory.back());
            this->paused_index_stored_trajectory.fill(-1);
            // For each configuration in the trajectory we have:
            for (unsigned int i = 0; (i < num_points) && rclcpp::ok(); ++i) {
                // std::cout << "point: "<< i << " / " << num_points << std::endl;
                // Check if there is a cancel request
                if (goal_handle->is_canceling()) {
                    this->moving = false;
                    mutex_finger_moving.lock();
                        this->finger_moving.fill(false);
                    mutex_finger_moving.unlock();
                    result->status = false;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(node_->get_logger(), "MoveFingerTipPoses action canceled");
                    stored_trajectory.clear();
                    return;
                }

                // Move the hand to the ith configuration
                std::array<double, n_fingers * finger_n_dofs> current_desired_pos;
                mutex_current_trajectory.lock();
                    current_desired_pos_vec = current_trajectory[i];
                mutex_current_trajectory.unlock();
                for (int ii=0; ii < n_fingers * finger_n_dofs; ii ++){
                    current_desired_pos[ii] =  current_desired_pos_vec(ii);
                }

                // Alternative if feedback_current_configuration is a std::vector
                // std::vector<double> current_conf_vector;
                if(setDesiredConfiguration(current_desired_pos)){
                    // Store the number of trajectory we are in
                    mutex_current_trajectory.lock();
                        num_current_configuration = i;
                    mutex_current_trajectory.unlock();
                    // Get current State
                    getCurrentState(current_conf, current_vel, current_acc, current_torque);
                    // Populate the feedback with the actual current state
                    for (int ii=0; ii < n_fingers * finger_n_dofs; ii ++){
                        // Populate the feedback header:
                        feedback_current_configuration.header.stamp = rclcpp::Clock().now();
                        feedback_current_configuration.header.frame_id =  std::to_string(index);
                        // Populate the feedback position, velocity and torque
                        feedback_current_configuration.position[ii] = current_conf(ii);
                        feedback_current_configuration.velocity[ii] = current_vel(ii);
                        feedback_current_configuration.effort[ii] = current_torque(ii);
                        // Alternative if feedback_current_configuration is a std::vector
                        // current_conf_vector.push_back(current_conf(ii));
                    }
                    // Alternative if feedback_current_configuration is a std::vector
                    // feedback_current_configuration = current_conf_vector;
                    index ++;
                }
                else{
                    RCLCPP_INFO(node_->get_logger(), "Error in setDesiredConfiguration()");  
                    return;
                }
                // Publish feedback
                goal_handle->publish_feedback(feedback);
                // RCLCPP_INFO(node_->get_logger(), "Publish feedback");
                // Time delay.
                // loop_rate.sleep();
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*1000)));
            }
            RCLCPP_INFO(node_->get_logger(), "Waiting... ");
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*500000)));            // Check if goal is done
            this->moving = false;
            mutex_finger_moving.lock();
                this->finger_moving.fill(false);
            mutex_finger_moving.unlock();
            if (rclcpp::ok()) {
                // Define maximum diferences for each joint and for them all
                double max_difference_position_threshold = 0.005;
                double max_difference_orientation_threshold = 0.05;
                double max_additive_difference_position_threshold = 0.05; 
                double max_additive_difference_orientation_threshold = 0.1; 
                bool all_within_threshold = true;
                double additive_difference_position = 0.0;  
                double additive_difference_orientation = 0.0;  

                Eigen::Matrix<double,3,4> eul_angles_solved;
                Eigen::Matrix<double,3,4> translation_solved;

                if(computeFK(false, eul_angles_solved, translation_solved)){
                    // Read the current hand configuration
                    std::array<double, n_fingers * 3> position_difference;
                    std::array<double, n_fingers * 3> orientation_difference;
                    // Calculate the difference between the desired goal poses and the final hand poses
                    for (size_t i = 0; i < n_fingers * 3; i++) {
                        position_difference[i] = abs(desired_positions[i]-translation_solved(i));
                        orientation_difference[i] = abs(goal->euler_angles[i]-eul_angles_solved(i));
                        additive_difference_position += position_difference[i];
                        additive_difference_orientation += orientation_difference[i];
                        if (position_difference[i] > max_difference_position_threshold) {
                            all_within_threshold = false;
                        }
                        if (orientation_difference[i] > max_difference_orientation_threshold) {
                            all_within_threshold = false;
                        }
                        // RCLCPP_INFO(node_->get_logger(), "Position %zu difference: %f", i, position_difference[i]);
                        // RCLCPP_INFO(node_->get_logger(), "Orientation %zu difference: %f", i, orientation_difference[i]);
                    }
                    RCLCPP_INFO(node_->get_logger(), "Total additive position difference: %f", additive_difference_position);
                    RCLCPP_INFO(node_->get_logger(), "Total additive orientation difference: %f", additive_difference_orientation);
                    // If the difference is lower than the maximum stablished thresholds success is true, false otherwise.
                    if (all_within_threshold && (additive_difference_position <= max_additive_difference_position_threshold) && (additive_difference_orientation<=max_additive_difference_orientation_threshold)) {
                        result->status = true;
                        RCLCPP_INFO(node_->get_logger(), "MoveFingerTipPoses finished successfully!");
                    } else {
                        result->status = false;
                        RCLCPP_INFO(node_->get_logger(), "MoveFingerTipPoses finished");
                    }
                }
                else{
                    result->status = false;
                    RCLCPP_INFO(node_->get_logger(), "MoveFingerTipPoses finished Error when computing final FK");
                }
                goal_handle->succeed(result);
                RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
            }
            stored_trajectory.clear();
        }
        else{
            // Error when computing the trajectory
            RCLCPP_WARN(node_->get_logger(), "Error in MoveFingerTipPoses when computing the trajectory");
        }
    }

/* Move along a trajectory*/

    //! \brief Callback function for handling goal of MoveTrajectory. 
    //! We will reject if the size of the configurations do not match the number of joints of the hand or if there are less than 2 configurations in the trajectory
    rclcpp_action::GoalResponse AllegroRos::handle_goal_move_trajectory(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveTrajectory::Goal> goal)
    {
        RCLCPP_INFO(node_->get_logger(), "Received goal request for the hand %s", this->hand_name.c_str());
        (void)uuid;
        // Check if there are at least two configurations in the trajectory
        if (goal->trajectory.size() < 2)
        {
            RCLCPP_WARN(node_->get_logger(), "Trajectory must contain at least two hand configurations.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // // Check that each configuration has 16 joint values
        // for (const auto &hand_configuration : goal->trajectory)
        // {
        //     if (hand_configuration.hand_configuration.size() != (this->n_fingers * this->finger_n_dofs))
        //     {
        //         RCLCPP_WARN(node_->get_logger(), "Each configuration must contain %u joint values. Size is not valid.", this->n_fingers * this->finger_n_dofs);
        //         return rclcpp_action::GoalResponse::REJECT;
        //     }
        // }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse AllegroRos::handle_cancel_move_trajectory(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveTrajectory>> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void AllegroRos::handle_accepted_move_trajectory(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveTrajectory>> goal_handle)
    {
        // current_controller = "Torque";
        // If we are using the Torque or the GravityCompensator controller, we set up the PID.
        if ((current_controller == "Torque") || (current_controller == "GravityCompensator")){
            std::cout << "Current controller: " << current_controller << std::endl;
            std::cout << "In order to move to a desired configuration, the PD or PID controller is needed.";
            setController("JointPositionPID");
        }
            // setController("JointPositionPID");

        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&AllegroRos::execute_move_trajectory, this, std::placeholders::_1), goal_handle}.detach();
    }

    //! \brief execute function to move at the desired speed using ros to control the time
    void AllegroRos::execute_move_trajectory(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveTrajectory>> goal_handle)
    {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*1000)));

//         AllegroRos::Vector current_conf, current_vel, current_acc, current_torque, current_desired_pos_vec;
//         const auto goal = goal_handle->get_goal();
//         auto feedback = std::make_shared<MoveTrajectory::Feedback>();
//         auto & feedback_current_configuration = feedback->current_configuration;
//         auto & feedback_desired_current_configuration = feedback->desired_current_configuration;
        
//         // Moving to the initial position in a controlled manner at 50% of the maximum speed
//         RCLCPP_INFO(node_->get_logger(), "Going to initial Trajectory point");
//         for (int ii=0; ii < n_fingers * finger_n_dofs; ii ++){
//             current_conf(ii) =  goal->trajectory[0].hand_configuration[ii];
//         }
//         // moveToConf(current_conf, 0.5);
//         // Store the next configuration in the trajectory
// //             std::array<double, n_fingers * finger_n_dofs> current_desired_pos_;
// //             for (int ii=0; ii < n_fingers * finger_n_dofs; ii ++){
// //                 current_desired_pos_[ii] =  current_conf(ii);
// //             }
// // setDesiredConfiguration(current_desired_pos_);
//         RCLCPP_INFO(node_->get_logger(), "Waiting... ");
//         std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*100000)));

//         RCLCPP_INFO(node_->get_logger(), "Executing goal");
//         // Fill in the joint names
//         for (char i = '0'; i <= '3'; ++i) {
//             for (char j = '0'; j <= '3'; ++j) {
//                 feedback_current_configuration.name.push_back("j" + std::string(1, i) + std::string(1, j));
//                 feedback_desired_current_configuration.name.push_back("j" + std::string(1, i) + std::string(1, j));
//             }
//         }
//         int index = 0;
//         // Set the size of the position, velocity and torque
//         feedback_current_configuration.position.resize(n_fingers * finger_n_dofs);
//         feedback_current_configuration.velocity.resize(n_fingers * finger_n_dofs);
//         feedback_current_configuration.effort.resize(n_fingers * finger_n_dofs);
//         feedback_desired_current_configuration.position.resize(n_fingers * finger_n_dofs);
//         auto result = std::make_shared<MoveTrajectory::Result>();

        
//         // Calculate the trajectory to arrive to the final configuration
//         mutex_current_trajectory.lock();
//             num_points =  goal->trajectory.size();
//         mutex_current_trajectory.unlock();
//         // Pass the goal trajectory to std::vector<AllegroHandDriver::Vector> format
//         std::vector<AllegroHandDriver::Vector> trajectory;
//         for (long unsigned int i=0; i < num_points; i++){
//             AllegroHandDriver::Vector configuration;
//             for (int ii=0; ii < 16; ii ++){
//                 configuration(ii) = goal->trajectory[i].hand_configuration[ii];
//             }
//             trajectory.push_back(configuration);
//         }
//          // Store trajectory in the global variable current trajectory
//         mutex_current_trajectory.lock();
//             current_trajectory = trajectory;
//         mutex_current_trajectory.unlock();

//         this->moving = true;
//         setFingersInitialMovementStatus(current_trajectory.front(), current_trajectory.back());
//         this->paused_index_stored_trajectory.fill(-1);
//         RCLCPP_INFO(node_->get_logger(), "Num points: %u", num_points);
//         // For each configuration in the trajectory we have:
//         for (unsigned int i = 0; (i < num_points) && rclcpp::ok(); ++i) {
//             // Check if there is a cancel request
//             if (goal_handle->is_canceling()) {
//                 this->moving = false;
//                 mutex_finger_moving.lock();
//                     this->finger_moving.fill(false);
//                 mutex_finger_moving.unlock();
//                 result->status = false;
//                 goal_handle->canceled(result);
//                 RCLCPP_INFO(node_->get_logger(), "MoveTrajectory action canceled");
//                 stored_trajectory.clear();
//                 return;
//             }
//             // Store the next configuration in the trajectory
//             std::array<double, n_fingers * finger_n_dofs> current_desired_pos;
//             mutex_current_trajectory.lock();
//                 current_desired_pos_vec = current_trajectory[i];
//             mutex_current_trajectory.unlock();
//             for (int ii=0; ii < n_fingers * finger_n_dofs; ii ++){
//                 current_desired_pos[ii] =  current_desired_pos_vec(ii);
//             }

//             // Move hand to the desired configuration
//             if(setDesiredConfiguration(current_desired_pos)){
//                 // Store the number of trajectory we are in
//                 mutex_current_trajectory.lock();
//                     num_current_configuration = i;
//                 mutex_current_trajectory.unlock();
//                 // Get current State
//                 getCurrentState(current_conf, current_vel, current_acc, current_torque);
//                 // Populate the feedback with the actual current state
//                 for (int ii=0; ii < n_fingers * finger_n_dofs; ii ++){
//                     // Populate the feedback header:
//                     feedback_current_configuration.header.stamp = rclcpp::Clock().now();
//                     feedback_current_configuration.header.frame_id =  std::to_string(index);
//                     // Populate the feedback position, velocity and torque
//                     feedback_current_configuration.position[ii] = current_conf(ii);
//                     feedback_current_configuration.velocity[ii] = current_vel(ii);
//                     feedback_current_configuration.effort[ii] = current_torque(ii);

//                     feedback_desired_current_configuration.header.stamp = rclcpp::Clock().now();
//                     feedback_desired_current_configuration.header.frame_id =  std::to_string(index);
//                     feedback_desired_current_configuration.position[ii] = current_desired_pos[ii];
//                 }
//                 index ++;
//             }
//             else{
//                 RCLCPP_INFO(node_->get_logger(), "Error in setDesiredConfiguration()");  
//                 return;
//             }
//             // Publish feedback
//             goal_handle->publish_feedback(feedback);
//             // RCLCPP_INFO(node_->get_logger(), "Publish feedback");
//             // Time delay.
//             // loop_rate.sleep();
//             std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*1000)));
//         }
//         RCLCPP_INFO(node_->get_logger(), "Waiting... ");
//         std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*500000)));
//         this->moving = false;
//         mutex_finger_moving.lock();
//             this->finger_moving.fill(false);
//         mutex_finger_moving.unlock();
//         // Check if goal is done
//         if (rclcpp::ok()) {
//             // Define maximum diferences for each joint and for them all
//             double max_difference_threshold = 0.05; // rad
//             double max_additive_difference_threshold = 0.2; //rad
//             bool all_within_threshold = true;
//             double additive_difference = 0.0;  
//             // Read the current hand configuration
//             getCurrentState(current_conf);
//             std::array<double, n_fingers * finger_n_dofs> joint_difference;
//             std::vector<double> goal_configuration;
//             for(int ii = 0; ii < (n_fingers * finger_n_dofs); ii++){
//                goal_configuration.push_back(goal->trajectory[goal->trajectory.size()-1].hand_configuration[ii]);
//             }
//             // Calculate the difference between the desired goal configuration and the final hand configuration
//             for (size_t q = 0; q < n_fingers * finger_n_dofs; q++) {
//                 joint_difference[q] = abs(goal_configuration[q] - current_conf(q));
//                 // joint_difference[q] = abs(goal->trajectory[num_points-1].hand_configuration[q] - current_conf(q));
//                 additive_difference += joint_difference[q];
//                 if (joint_difference[q] > max_difference_threshold) {
//                     all_within_threshold = false;
//                 }
//                 RCLCPP_INFO(node_->get_logger(), "Joint %zu difference: %f", q, joint_difference[q]);
//             }
//             RCLCPP_INFO(node_->get_logger(), "Total additive difference: %f", additive_difference);
//             // If the difference is lower than the maximum stablished thresholds success is true, false otherwise.
//             if (all_within_threshold && additive_difference <= max_additive_difference_threshold) {
//                 result->status = true;
//                 RCLCPP_INFO(node_->get_logger(), "MoveConf finished successfully!");
//             } else {
//                 result->status = false;
//                 RCLCPP_INFO(node_->get_logger(), "MoveConf finished");
//             }
//             goal_handle->succeed(result);
//             RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
//         }
//         stored_trajectory.clear();
    }

/* Move along a given trajectory using the FollowJointTrajectory action*/

    //! \brief Callback function for handling goal of FollowJointTrajectory. 
    //! We will reject if the size of the configurations do not match the number of joints of the hand or if there are less than 2 configurations in the trajectory
    rclcpp_action::GoalResponse AllegroRos::handle_goal_follow_joint_trajectory(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        RCLCPP_INFO(node_->get_logger(), "Received goal request for the hand %s", this->hand_name.c_str());
        (void)uuid;
        // Check if there are at least two configurations in the trajectory
        if (goal->trajectory.points.size() < 2)
        {
            RCLCPP_WARN(node_->get_logger(), "Trajectory must contain at least two hand configurations.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Check that each configuration has 16 joint values
        if (goal->trajectory.joint_names.size() != (this->n_fingers * this->finger_n_dofs))
        {
            RCLCPP_WARN(node_->get_logger(), "The path points must contain %u joint values. Size is not valid.", this->n_fingers * this->finger_n_dofs);
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse AllegroRos::handle_cancel_follow_joint_trajectory(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void AllegroRos::handle_accepted_follow_joint_trajectory(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle)
    {
        // RCLCPP_INFO(node_->get_logger(), "Current controller: " + current_controller);
        // // If we are using the Torque or the GravityCompensator controller, we set up the PID.
        // if ((current_controller == "Torque") || (current_controller == "GravityCompensator")){
        //     std::cout << "Current controller: " << current_controller << std::endl;
        //     std::cout << "In order to move to a desired configuration, the PD or PID controller is needed.";
        //     setController("JointPositionPID");
        // }
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&AllegroRos::execute_follow_joint_trajectory, this, std::placeholders::_1), goal_handle}.detach();
    }

    //! \brief execute function to move at the desired speed using ros to control the time
    void AllegroRos::execute_follow_joint_trajectory(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle)
    {
        AllegroRos::Vector current_conf, current_vel, current_acc, current_torque, current_desired_pos_vec;
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
        auto & feedback_header = feedback->header;
        auto & feedback_joint_names = feedback->joint_names;
        auto & feedback_current_configuration = feedback->actual;
        auto & feedback_desired_current_configuration = feedback->desired;
        auto & feedback_error_current_configuration = feedback->error;
        
        // Moving to the initial position in a controlled manner at 50% of the maximum speed
        RCLCPP_INFO(node_->get_logger(), "Going to initial Trajectory point");
        for (int ii=0; ii < n_fingers * finger_n_dofs; ii ++){
            current_conf(ii) =  goal->trajectory.points[0].positions[ii];
        }
        moveToConf(current_conf, 0.5);

        // std::stringstream ss;
        // ss << " Initial configuration: ";
        // for (int ii=0; ii < n_fingers * finger_n_dofs; ii ++){
        //     ss << current_conf(ii) << ", ";
        // }
        // RCLCPP_INFO(node_->get_logger(), ss.str().c_str());



        RCLCPP_INFO(node_->get_logger(), "Waiting... ");
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*100000)));

        RCLCPP_INFO(node_->get_logger(), "Executing goal");
        // Fill in the joint names of the feedback
        // // Option fill it with predefined names
        // for (char i = '0'; i <= '3'; ++i) {
        //     for (char j = '0'; j <= '3'; ++j) {
        //         feedback_joint_names.push_back("j" + std::string(1, i) + std::string(1, j));
        //     }
        // }
        // Option 2. Same names as the goal trajectory
        feedback_joint_names = goal->trajectory.joint_names;
        int index = 0;
        // Set the size of the position, velocity and torque
        feedback_current_configuration.positions.resize(n_fingers * finger_n_dofs);
        feedback_current_configuration.velocities.resize(n_fingers * finger_n_dofs);
        feedback_current_configuration.effort.resize(n_fingers * finger_n_dofs);
        feedback_desired_current_configuration.positions.resize(n_fingers * finger_n_dofs);
        feedback_error_current_configuration.positions.resize(n_fingers * finger_n_dofs);
        auto result = std::make_shared<FollowJointTrajectory::Result>();

        
        // Calculate the trajectory to arrive to the final configuration
        mutex_current_trajectory.lock();
            num_points =  goal->trajectory.points.size();
        mutex_current_trajectory.unlock();
        // Pass the goal trajectory to std::vector<AllegroHandDriver::Vector> format
        std::vector<AllegroHandDriver::Vector> trajectory;
        for (long unsigned int i=0; i < num_points; i++){
            AllegroRos::Vector configuration;
            for (int ii=0; ii < 16; ii ++){
                configuration(ii) = goal->trajectory.points[i].positions[ii];
            }
            trajectory.push_back(configuration);
        }
         // Store trajectory in the global variable current trajectory
        mutex_current_trajectory.lock();
            current_trajectory = trajectory;
        mutex_current_trajectory.unlock();

        this->moving = true;
        setFingersInitialMovementStatus(current_trajectory.front(), current_trajectory.back());
        this->paused_index_stored_trajectory.fill(-1);
        // For each configuration in the trajectory we have:
        for (unsigned int i = 0; (i < num_points) && rclcpp::ok(); ++i) {
            // Check if there is a cancel request
            if (goal_handle->is_canceling()) {
                this->moving = false;
                mutex_finger_moving.lock();
                    this->finger_moving.fill(false);
                mutex_finger_moving.unlock();
                goal_handle->canceled(result);
                RCLCPP_INFO(node_->get_logger(), "FollowJointTrajectory action canceled");
                stored_trajectory.clear();
                return;
            }
            // Store the next configuration in the trajectory
            std::array<double, n_fingers * finger_n_dofs> current_desired_pos;
            mutex_current_trajectory.lock();
                current_desired_pos_vec = current_trajectory[i];
            mutex_current_trajectory.unlock();
            for (int ii=0; ii < n_fingers * finger_n_dofs; ii ++){
                current_desired_pos[ii] =  current_desired_pos_vec(ii);
            }

            // Move hand to the desired configuration
            if(setDesiredConfiguration(current_desired_pos)){
                // Store the number of trajectory we are in
                mutex_current_trajectory.lock();
                    num_current_configuration = i;
                mutex_current_trajectory.unlock();
                // Get current State
                getCurrentState(current_conf, current_vel, current_acc, current_torque);
                // Populate the feedback with the actual current state
                for (int ii=0; ii < n_fingers * finger_n_dofs; ii ++){
                    // Populate the feedback header:
                    feedback_header.stamp = rclcpp::Clock().now();
                    feedback_header.frame_id =  std::to_string(index);
                    // Populate the feedback position, velocity and torque
                    feedback_current_configuration.positions[ii] = current_conf(ii);
                    feedback_current_configuration.velocities[ii] = current_vel(ii);
                    feedback_current_configuration.effort[ii] = current_torque(ii);

                    feedback_desired_current_configuration.positions[ii] = current_desired_pos[ii];

                    feedback_error_current_configuration.positions[ii] = abs(current_desired_pos[ii] - current_conf(ii));
                }
                index ++;
            }
            else{
                RCLCPP_INFO(node_->get_logger(), "Error in setDesiredConfiguration()");  
                return;
            }
            // Publish feedback
            goal_handle->publish_feedback(feedback);
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*1000)));
        }
        RCLCPP_INFO(node_->get_logger(), "Waiting... ");
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*500000)));
        this->moving = false;
        mutex_finger_moving.lock();
            this->finger_moving.fill(false);
        mutex_finger_moving.unlock();
        // Check if goal is done
        if (rclcpp::ok()) {
            // Define maximum diferences for each joint and for them all
            double max_difference_threshold = 0.05; // rad
            double max_additive_difference_threshold = 0.2; //rad
            bool all_within_threshold = true;
            double additive_difference = 0.0;  
            // Read the current hand configuration
            getCurrentState(current_conf);
            std::array<double, n_fingers * finger_n_dofs> joint_difference;
            std::vector<double> goal_configuration;
            for(int ii = 0; ii < (n_fingers * finger_n_dofs); ii++){
               goal_configuration.push_back(goal->trajectory.points[goal->trajectory.points.size()-1].positions[ii]);
            }
            // Calculate the difference between the desired goal configuration and the final hand configuration
            for (size_t q = 0; q < n_fingers * finger_n_dofs; q++) {
                joint_difference[q] = abs(goal_configuration[q] - current_conf(q));
                // joint_difference[q] = abs(goal->trajectory[num_points-1].hand_configuration[q] - current_conf(q));
                additive_difference += joint_difference[q];
                if (joint_difference[q] > max_difference_threshold) {
                    all_within_threshold = false;
                }
                RCLCPP_INFO(node_->get_logger(), "Joint %zu difference: %f", q, joint_difference[q]);
            }
            RCLCPP_INFO(node_->get_logger(), "Total additive difference: %f", additive_difference);
            // If the difference is lower than the maximum stablished thresholds success is true, false otherwise.
            if (all_within_threshold && additive_difference <= max_additive_difference_threshold) {
                result->error_string = "FollowJointTrajectory Finished successfully";
                RCLCPP_INFO(node_->get_logger(), "FollowJointTrajectory finished successfully!");
                result->SUCCESSFUL;
            } else {
                result->error_string = "FollowJointTrajectory Finished unsuccessfully";
                RCLCPP_INFO(node_->get_logger(), "FollowJointTrajectory finished unsuccessfully");
            }
            goal_handle->succeed(result);
        }
        stored_trajectory.clear();
    }
/* Move along a trajectory at a desired max speed. ONLY SLOWS DOWN THE TRAJECTORY, IT DOES NOT SPEED IT UP*/
    //! \brief Callback function for handling goal of MoveTrajectoryMaxVel. 
    //! We will reject if the size of the configurations do not match the number of joints of the hand or if there are less than 2 configurations in the trajectory
    rclcpp_action::GoalResponse AllegroRos::handle_goal_move_trajectory_vel(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveTrajectoryMaxVel::Goal> goal)
    {
        RCLCPP_INFO(node_->get_logger(), "Received goal request for the hand %s", this->hand_name.c_str());
        (void)uuid;
        // Check if there are at least two configurations in the trajectory
        if (goal->trajectory.size() < 2)
        {
            RCLCPP_WARN(node_->get_logger(), "Trajectory must contain at least two hand configurations.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Check that each configuration has 16 joint values
        for (const auto &hand_configuration : goal->trajectory)
        {
            if (hand_configuration.hand_configuration.size() != (this->n_fingers * this->finger_n_dofs))
            {
                RCLCPP_WARN(node_->get_logger(), "Each configuration must contain %u joint values. Size is not valid.", this->n_fingers * this->finger_n_dofs);
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse AllegroRos::handle_cancel_move_trajectory_vel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveTrajectoryMaxVel>> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void AllegroRos::handle_accepted_move_trajectory_vel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveTrajectoryMaxVel>> goal_handle)
    {
        // If we are using the Torque or the GravityCompensator controller, we set up the PID.
        if ((current_controller == "Torque") || (current_controller == "GravityCompensator")){
            std::cout << "Current controller: " << current_controller << std::endl;
            std::cout << "In order to move to a desired configuration, the PD or PID controller is needed.";
            setController("JointPositionPID");
        }
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&AllegroRos::execute_move_trajectory_vel, this, std::placeholders::_1), goal_handle}.detach();
    }

    //! \brief execute function to move at the desired speed using ros to control the time
    void AllegroRos::execute_move_trajectory_vel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveTrajectoryMaxVel>> goal_handle)
    {
        AllegroRos::Vector current_conf, current_vel, current_acc, current_torque, current_desired_pos_vec;
        std::vector<AllegroHandDriver::Vector> trajectory_new;
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveTrajectoryMaxVel::Feedback>();
        auto & feedback_current_configuration = feedback->current_configuration;
        auto & feedback_desired_current_configuration = feedback->desired_current_configuration;
        
        // Moving to the initial position in a controlled manner at 50% of the maximum speed
        RCLCPP_INFO(node_->get_logger(), "Going to initial Trajectory point");
        for (int ii=0; ii < n_fingers * finger_n_dofs; ii ++){
            current_conf(ii) =  goal->trajectory[0].hand_configuration[ii];
        }
        moveToConf(current_conf, goal->max_velocity);

        RCLCPP_INFO(node_->get_logger(), "Waiting... ");
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*100000)));

        // Pass the trajectory to std::vector<AllegroHandDriver::Vector> format
        for (long unsigned int i=0; i < goal->trajectory.size(); i++){
            AllegroRos::Vector configuration;
            for (int ii=0; ii < 16; ii ++){
                configuration(ii) = goal->trajectory[i].hand_configuration[ii];
            }
            trajectory_new.push_back(configuration);
        }
        // Calculate the new trajectory considering the max_velocity
        calculateTrajectory(trajectory_new, goal->max_velocity);
        mutex_current_trajectory.lock();
            // Store the number of configurations that the new trajectory has
            num_points = trajectory_new.size();
            // Store trajectory in the global variable current trajectory
            current_trajectory = trajectory_new;
        mutex_current_trajectory.unlock();
        RCLCPP_INFO(node_->get_logger(), "Executing goal");
        // Fill in the joint names
        int index = 0;
        for (char i = '0'; i <= '3'; ++i) {
            for (char j = '0'; j <= '3'; ++j) {
                feedback_current_configuration.name.push_back("j" + std::string(1, i) + std::string(1, j));
                feedback_desired_current_configuration.name.push_back("j" + std::string(1, i) + std::string(1, j));
                ++index;
            }
        }
        index = 0;
        // Set the size of the position, velocity and torque
        feedback_current_configuration.position.resize(n_fingers * finger_n_dofs);
        feedback_current_configuration.velocity.resize(n_fingers * finger_n_dofs);
        feedback_current_configuration.effort.resize(n_fingers * finger_n_dofs);
        feedback_desired_current_configuration.position.resize(n_fingers * finger_n_dofs);
        auto result = std::make_shared<MoveTrajectoryMaxVel::Result>();
        this->moving = true;
        setFingersInitialMovementStatus(current_trajectory.front(), current_trajectory.back());
        this->paused_index_stored_trajectory.fill(-1);

        // For each configuration in the trajectory we have:
        for (unsigned int i = 0; (i < num_points) && rclcpp::ok(); ++i) {
            // Check if there is a cancel request
            if (goal_handle->is_canceling()) {
                this->moving = false;
                mutex_finger_moving.lock();
                    this->finger_moving.fill(false);
                mutex_finger_moving.unlock();
                result->status = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(node_->get_logger(), "MoveConf action canceled");
                stored_trajectory.clear();
                return;
            }

           // Store the next configuration in the trajectory
            std::array<double, n_fingers * finger_n_dofs> current_desired_pos;
            mutex_current_trajectory.lock();
                current_desired_pos_vec = current_trajectory[i];
            mutex_current_trajectory.unlock();
            for (int ii=0; ii < n_fingers * finger_n_dofs; ii ++){
                current_desired_pos[ii] =  current_desired_pos_vec(ii);
            }

            // Move hand to the desired configuration
            if(setDesiredConfiguration(current_desired_pos)){
                // Store the number of trajectory we are in
                mutex_current_trajectory.lock();
                    num_current_configuration = i;
                mutex_current_trajectory.unlock();
                // Get current State
                getCurrentState(current_conf, current_vel, current_acc, current_torque);
                // Populate the feedback with the actual current state
                for (int ii=0; ii < n_fingers * finger_n_dofs; ii ++){
                    // Populate the feedback header:
                    feedback_current_configuration.header.stamp = rclcpp::Clock().now();
                    feedback_current_configuration.header.frame_id =  std::to_string(index);
                    // Populate the feedback position, velocity and torque
                    feedback_current_configuration.position[ii] = current_conf(ii);
                    feedback_current_configuration.velocity[ii] = current_vel(ii);
                    feedback_current_configuration.effort[ii] = current_torque(ii);

                    feedback_desired_current_configuration.header.stamp = rclcpp::Clock().now();
                    feedback_desired_current_configuration.header.frame_id =  std::to_string(index);
                    feedback_desired_current_configuration.position[ii] = current_desired_pos[ii];
                }
                index ++;
            }
            else{
                RCLCPP_INFO(node_->get_logger(), "Error in setDesiredConfiguration()");  
                return;
            }
            // Publish feedback
            goal_handle->publish_feedback(feedback);
            // RCLCPP_INFO(node_->get_logger(), "Publish feedback");
            // Time delay.
            // loop_rate.sleep();
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*1000)));
        }
        RCLCPP_INFO(node_->get_logger(), "Waiting... ");
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*500000)));
        this->moving = false;
        mutex_finger_moving.lock();
            this->finger_moving.fill(false);
        mutex_finger_moving.unlock();
        // Check if goal is done
        if (rclcpp::ok()) {
            // Define maximum diferences for each joint and for them all
            double max_difference_threshold = 0.05; // rad
            double max_additive_difference_threshold = 0.2; //rad
            bool all_within_threshold = true;
            double additive_difference = 0.0;  
            // Read the current hand configuration
            getCurrentState(current_conf);
            std::array<double, n_fingers * finger_n_dofs> joint_difference;
            // Calculate the difference between the desired goal configuration and the final hand configuration
            for (size_t q = 0; q < n_fingers * finger_n_dofs; q++) {
                joint_difference[q] = abs(goal->trajectory[goal->trajectory.size()-1].hand_configuration[q] - current_conf(q));
                additive_difference += joint_difference[q];
                if (joint_difference[q] > max_difference_threshold) {
                    all_within_threshold = false;
                }
                RCLCPP_INFO(node_->get_logger(), "Joint %zu difference: %f", q, joint_difference[q]);
            }
            RCLCPP_INFO(node_->get_logger(), "Total additive difference: %f", additive_difference);
            // If the difference is lower than the maximum stablished thresholds success is true, false otherwise.
            if (all_within_threshold && additive_difference <= max_additive_difference_threshold) {
                result->status = true;
                RCLCPP_INFO(node_->get_logger(), "MoveConf finished successfully!");
            } else {
                result->status = false;
                RCLCPP_INFO(node_->get_logger(), "MoveConf finished");
            }
            goal_handle->succeed(result);
            RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
        }
        stored_trajectory.clear();
    }

/* Move to a given configuration at a speed based on the desired percentage speed */
    //! \brief Callback function for handling goal of MoveConfigFinger. 
    //! We will reject if the size of the configurations do not match the number of joints of the hand
    rclcpp_action::GoalResponse AllegroRos::handle_goal_move_config_finger(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveConfigFinger::Goal> goal)
    {
        RCLCPP_INFO(node_->get_logger(), "Received goal request for the hand %s", this->hand_name.c_str());
        (void)uuid;
    
        // If the number of joints of the trajectory is incorrect
        if (goal->final_configuration.size() != (this->finger_n_dofs)) {
            RCLCPP_WARN(node_->get_logger(), "Received joint configuration size is not valid. Size: %zu", goal->final_configuration.size());
            return rclcpp_action::GoalResponse::REJECT;
        }
        // If the finger_id is incorrect
        if(goal->finger_id > n_fingers){
            RCLCPP_WARN(node_->get_logger(), "Received finger index is not correct");
            return rclcpp_action::GoalResponse::REJECT;
        }
        // If we are already moving the goal gets rejected
        if (this->moving) {
            RCLCPP_WARN(node_->get_logger(), "MoveConfigFinger action cannot be executed, because there is another movement in place.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse AllegroRos::handle_cancel_move_config_finger(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveConfigFinger>> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void AllegroRos::handle_accepted_move_config_finger(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveConfigFinger>> goal_handle)
    {
        // If we are using the Torque or the GravityCompensator controller, we set up the PID.
        if ((current_controller == "Torque") || (current_controller == "GravityCompensator")){
            std::cout << "Current controller: " << current_controller << std::endl;
            std::cout << "In order to move to a desired configuration, the PD or PID controller is needed.";
        setController("JointPositionPID");
        }
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&AllegroRos::execute_move_config_finger, this, std::placeholders::_1), goal_handle}.detach();
    }

    //! \brief execute function to move at the desired speed using ros to control the time
    void AllegroRos::execute_move_config_finger(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveConfigFinger>> goal_handle){
        RCLCPP_INFO(node_->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveConfigFinger::Feedback>();
        auto & feedback_current_configuration = feedback->current_configuration;
        // Fill in the joint names of the finger
        int index = 0;
        for (char j = '0'; j <= '3'; ++j) {
            feedback_current_configuration.name.push_back("j" + std::string(1, static_cast<char>(goal->finger_id)) + std::string(1, j));
            ++index;
        }
        index = 0;
        // Set the size of the position, velocity and torque
        feedback_current_configuration.position.resize(finger_n_dofs);
        feedback_current_configuration.velocity.resize(finger_n_dofs);
        feedback_current_configuration.effort.resize(finger_n_dofs);
        auto result = std::make_shared<MoveConfigFinger::Result>();
        
        std::vector<AllegroHandDriver::Vector> trajectory;
        AllegroRos::Vector final_hand_configuration, current_desired_pos_vec, current_conf, current_vel, current_acc, current_torque;

        // We populate store the current joint values
        getCurrentState(final_hand_configuration);
        // We populate the desired final configuration finger
        int i = 0;
        for(int ii = goal->finger_id*n_fingers; ii < (goal->finger_id*n_fingers + finger_n_dofs); ii++){
            final_hand_configuration(ii) = goal->final_configuration[i];
            i++;
        }
        std::vector<double> final_hand_configuration_vector;
        for(int ii = 0; ii < (n_fingers * finger_n_dofs); ii++){
            final_hand_configuration_vector.push_back(final_hand_configuration(ii));
        }

        // Calculate the trajectory to arrive to the final configuration
        if(calculateTrajectory(final_hand_configuration_vector, goal->percentage_velocity, trajectory)){
            // Store trajectory in the global variable current trajectory
            mutex_current_trajectory.lock();
                num_points = trajectory.size();
                current_trajectory = trajectory;
            mutex_current_trajectory.unlock();
            this->moving = true;
            mutex_finger_moving.lock();
                this->finger_moving.fill(false);
                this->finger_moving[goal->finger_id] = true;
            mutex_finger_moving.unlock();
            this->paused_index_stored_trajectory.fill(-1);

            // For each configuration in the trajectory we have:
            for (unsigned int i = 0; (i < num_points) && rclcpp::ok(); ++i) {
                // std::cout << "point: "<< i << " / " << num_points << std::endl;
                // Check if there is a cancel request
                if (goal_handle->is_canceling()) {
                    this->moving = false;
                    mutex_finger_moving.lock();
                        this->finger_moving.fill(false);
                    mutex_finger_moving.unlock();
                    result->status = false;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(node_->get_logger(), "MoveConfFinger action canceled");
                    stored_trajectory.clear();
                    return;
                }

                // Move the hand to the ith configuration
                std::array<double, n_fingers * finger_n_dofs> current_desired_pos;
                mutex_current_trajectory.lock();
                    current_desired_pos_vec = current_trajectory[i];
                mutex_current_trajectory.unlock();
                for (int ii=0; ii < n_fingers * finger_n_dofs; ii ++){
                    current_desired_pos[ii] =  current_desired_pos_vec(ii);
                }

                if(setDesiredConfiguration(current_desired_pos)){
                    // Store the number of trajectory we are in
                    mutex_current_trajectory.lock();
                        num_current_configuration = i;
                    mutex_current_trajectory.unlock();
                    // Get current State
                    getCurrentState(current_conf, current_vel, current_acc, current_torque);
                    int index_feedback = 0;
                    for(int ii = goal->finger_id*n_fingers; ii < (goal->finger_id*n_fingers + finger_n_dofs); ii++){
                        // Populate the feedback header:
                        feedback_current_configuration.header.stamp = rclcpp::Clock().now();
                        feedback_current_configuration.header.frame_id =  std::to_string(index);
                        // Populate the feedback position, velocity and torque
                        feedback_current_configuration.position[index_feedback] = current_conf(ii);
                        feedback_current_configuration.velocity[index_feedback] = current_vel(ii);
                        feedback_current_configuration.effort[index_feedback] = current_torque(ii);
                        index_feedback ++;
                    }
                    index ++;
                }
                else{
                    RCLCPP_INFO(node_->get_logger(), "Error in setDesiredConfiguration()");  
                    return;
                }
                // Publish feedback
                goal_handle->publish_feedback(feedback);
                // RCLCPP_INFO(node_->get_logger(), "Publish feedback");
                // Time delay.
                // loop_rate.sleep();
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*1000)));
            }
            RCLCPP_INFO(node_->get_logger(), "Waiting... ");
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*500000)));
            this->moving = false;
            mutex_finger_moving.lock();
                this->finger_moving.fill(false);
            mutex_finger_moving.unlock();
            // Check if goal is done
            if (rclcpp::ok()) {
                // Define maximum diferences for each joint and for them all
                double max_difference_threshold = 0.05; // rad
                double max_additive_difference_threshold = 0.5; //rad
                bool all_within_threshold = true;
                double additive_difference = 0.0;  
                // Read the current hand configuration
                getCurrentState(current_conf);
                std::array<double, n_fingers * finger_n_dofs> joint_difference;
                // Calculate the difference between the desired goal configuration and the final hand configuration
                size_t i = 0;
                for(int q = goal->finger_id*n_fingers; q < (goal->finger_id*n_fingers + finger_n_dofs); q++){
                    joint_difference[i] = abs(goal->final_configuration[i] - current_conf(q));
                    additive_difference += joint_difference[i];
                    if (joint_difference[i] > max_difference_threshold) {
                        all_within_threshold = false;
                    }
                    RCLCPP_INFO(node_->get_logger(), "Joint %zu difference: %f", i, joint_difference[i]);
                    i ++;
                }
                RCLCPP_INFO(node_->get_logger(), "Total additive difference: %f", additive_difference);
                // If the difference is lower than the maximum stablished thresholds success is true, false otherwise.
                if (all_within_threshold && additive_difference <= max_additive_difference_threshold) {
                    result->status = true;
                    RCLCPP_INFO(node_->get_logger(), "MoveConfFinger finished successfully!");
                } else {
                    result->status = false;
                    RCLCPP_INFO(node_->get_logger(), "MoveConfFinger finished");
                }
                goal_handle->succeed(result);
                RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
            }
            stored_trajectory.clear();
        }
        else{
            // Error when computing the trajectory
            RCLCPP_WARN(node_->get_logger(), "Error in MoveConfFinger when computing the trajectory");
        }
    }

/* Move one fingertip to the desired pose at the desired velocity percentage */
    //! \brief Callback function for handling goal of MoveFingertipPose. 
    //! We will reject if the size of the configurations do not match the number of joints of the hand
    rclcpp_action::GoalResponse AllegroRos::handle_goal_move_fingertip_pose(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveFingertipPose::Goal> goal)
    {
        RCLCPP_INFO(node_->get_logger(), "Received goal request for the hand %s", this->hand_name.c_str());
        (void)uuid;
    
        // Check if the number of euler_angles is correct
        if (goal->euler_angles.size() != (3)) {
            RCLCPP_WARN(node_->get_logger(), "Received ZYZ Euler Angles size is not valid. Size: %zu", goal->euler_angles.size());
            return rclcpp_action::GoalResponse::REJECT;
        }
        // If the finger_id is incorrect
        if(goal->finger_id > n_fingers){
            RCLCPP_WARN(node_->get_logger(), "Received finger index is not correct");
            return rclcpp_action::GoalResponse::REJECT;
        }
        // If we are already moving the goal gets rejected
        if (this->moving) {
            RCLCPP_WARN(node_->get_logger(), "MoveFingertipPose action cannot be executed, because there is another movement in place.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse AllegroRos::handle_cancel_move_fingertip_pose(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveFingertipPose>> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void AllegroRos::handle_accepted_move_fingertip_pose(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveFingertipPose>> goal_handle)
    {
        // If we are using the Torque or the GravityCompensator controller, we set up the PID.
        if ((current_controller == "Torque") || (current_controller == "GravityCompensator")){
            std::cout << "Current controller: " << current_controller << std::endl;
            std::cout << "In order to move to a desired configuration, the PD or PID controller is needed.";
        setController("JointPositionPID");
        }
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&AllegroRos::execute_move_fingertip_pose, this, std::placeholders::_1), goal_handle}.detach();
    }

    //! \brief execute function to move at the desired speed using ros to control the time
    void AllegroRos::execute_move_fingertip_pose(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveFingertipPose>> goal_handle){
        RCLCPP_INFO(node_->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveFingertipPose::Feedback>();
        auto & feedback_current_configuration = feedback->current_configuration;
        // Fill in the joint names
        int index = 0;
        for (char j = '0'; j <= '3'; ++j) {
            feedback_current_configuration.name.push_back("j" + std::string(1, static_cast<char>(goal->finger_id)) + std::string(1, j));
            ++index;
        }
        index = 0;
        // Set the size of the position, velocity and torque
        feedback_current_configuration.position.resize(finger_n_dofs);
        feedback_current_configuration.velocity.resize(finger_n_dofs);
        feedback_current_configuration.effort.resize(finger_n_dofs);
        auto result = std::make_shared<MoveFingertipPose::Result>();

        std::vector<AllegroHandDriver::Vector> trajectory;
        AllegroRos::Vector current_desired_pos_vec, current_conf, current_vel, current_acc, current_torque;
        
            // // ALTERNATIVE: Do the IK of the finger and then calculate the trajectory
            // // Calculate the current joint values of the finger
            // std::vector<double> q_values_finger, q_values_hand;
            // std::vector<double> desired_position, euler_angles;
            // desired_position.push_back(goal->coordinates.x);
            // desired_position.push_back(goal->coordinates.y);
            // desired_position.push_back(goal->coordinates.z);
            // euler_angles.push_back(goal->euler_angles[0]);
            // euler_angles.push_back(goal->euler_angles[1]);
            // euler_angles.push_back(goal->euler_angles[2]);
            // computeIKFinger(false, goal->finger_id, desired_position, euler_angles, q_values_finger);

            // // Get the current hand configuration
            // getCurrentState(current_conf);
            // // Change the final values of the finger
            // int i = 0;
            // for(int ii = goal->finger_id*n_fingers; ii < (goal->finger_id*n_fingers + finger_n_dofs); ii++){
            //     current_conf(ii)= q_values_finger[i];
            //     i ++;
            // }
            // checkAndAjustJointConfigurationValues(current_conf);

            // for(int ii = 0; ii < n_fingers * finger_n_dofs; ii++){
            //     q_values_hand.push_back(current_conf(ii));
            //     RCLCPP_INFO(node_->get_logger(), "q_%d: %f", ii, q_values_hand[ii]);
            // }
            // // Calculate the trajectory to arrive to the final configuration
            // if(calculateTrajectory(q_values_hand, goal->percentage_velocity, trajectory)){
                
        // Compute FK of the current hand configuration
        Eigen::Matrix<double,3,4> eul_angles_solved,translation_solved;
        computeFK(false, eul_angles_solved, translation_solved);
        
        // Store the current coordinates and orientations of all the fingertips
        std::vector<double> desired_positions, euler_angles;
        for (int col = 0; col < translation_solved.cols(); ++col) {
            for (int row = 0; row < translation_solved.rows(); ++row) {
                desired_positions.push_back(translation_solved(row, col));
                euler_angles.push_back(eul_angles_solved(row, col));
            }
        }
        // Store the desired coordinates and orientations of the fingertip
        desired_positions[goal->finger_id*n_fingers] = goal->coordinates.x;
        desired_positions[goal->finger_id*n_fingers+1] = goal->coordinates.y;
        desired_positions[goal->finger_id*n_fingers+2] = goal->coordinates.z;
        euler_angles[goal->finger_id*n_fingers] = goal->euler_angles[0];
        euler_angles[goal->finger_id*n_fingers+1] = goal->euler_angles[1];
        euler_angles[goal->finger_id*n_fingers+2] = goal->euler_angles[2];

        // Calculate the trajectory to arrive to the final configuration
        if(calculateTrajectoryFromFingertipPoses(desired_positions, euler_angles, goal->percentage_velocity, trajectory)){
        
            // Store trajectory in the global variable current trajectory
            mutex_current_trajectory.lock();
                current_trajectory = trajectory;
                num_points = trajectory.size();
            mutex_current_trajectory.unlock();
            this->moving = true;
            mutex_finger_moving.lock();
                this->finger_moving.fill(false);
                this->finger_moving[goal->finger_id] = true;
            mutex_finger_moving.unlock();
            this->paused_index_stored_trajectory.fill(-1);

            // For each configuration in the trajectory we have:
            for (unsigned int i = 0; (i < num_points) && rclcpp::ok(); ++i) {
                // std::cout << "point: "<< i << " / " << num_points << std::endl;
                // Check if there is a cancel request
                if (goal_handle->is_canceling()) {
                    this->moving = false;
                    mutex_finger_moving.lock();
                        this->finger_moving.fill(false);
                    mutex_finger_moving.unlock();
                    result->status = false;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(node_->get_logger(), "MoveFingerTipPoses action canceled");
                    stored_trajectory.clear();
                    return;
                }

                // Move the hand to the ith configuration
                std::array<double, n_fingers * finger_n_dofs> current_desired_pos;
                mutex_current_trajectory.lock();
                    current_desired_pos_vec = current_trajectory[i];
                mutex_current_trajectory.unlock();
                for (int ii=0; ii < n_fingers * finger_n_dofs; ii ++){
                    current_desired_pos[ii] =  current_desired_pos_vec(ii);
                }

                if(setDesiredConfiguration(current_desired_pos)){
                    // Store the number of trajectory we are in
                    mutex_current_trajectory.lock();
                        num_current_configuration = i;
                    mutex_current_trajectory.unlock();
                    // Get current State
                    getCurrentState(current_conf, current_vel, current_acc, current_torque);
                    // Populate the feedback with the actual current state
                    int index_feedback = 0;
                    for(int ii = goal->finger_id*n_fingers; ii < (goal->finger_id*n_fingers + finger_n_dofs); ii++){
                        // Populate the feedback header:
                        feedback_current_configuration.header.stamp = rclcpp::Clock().now();
                        feedback_current_configuration.header.frame_id =  std::to_string(index);
                        // Populate the feedback position, velocity and torque
                        feedback_current_configuration.position[index_feedback] = current_conf(ii);
                        feedback_current_configuration.velocity[index_feedback] = current_vel(ii);
                        feedback_current_configuration.effort[index_feedback] = current_torque(ii);
                        index_feedback ++;
                    }
                    index ++;
                }
                else{
                    RCLCPP_INFO(node_->get_logger(), "Error in setDesiredConfiguration()");  
                    return;
                }
                // Publish feedback
                goal_handle->publish_feedback(feedback);
                // RCLCPP_INFO(node_->get_logger(), "Publish feedback");
                // Time delay.
                // loop_rate.sleep();
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*1000)));
            }
            RCLCPP_INFO(node_->get_logger(), "Waiting... ");
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*500000)));            // Check if goal is done
            this->moving = false;
            mutex_finger_moving.lock();
                this->finger_moving.fill(false);
            mutex_finger_moving.unlock();
            if (rclcpp::ok()) {
                // Define maximum diferences for each joint and for them all
                double max_difference_position_threshold = 0.005;
                double max_difference_orientation_threshold = 0.05;
                double max_additive_difference_position_threshold = 0.05; 
                double max_additive_difference_orientation_threshold = 0.1; 
                bool all_within_threshold = true;
                double additive_difference_position = 0.0;  
                double additive_difference_orientation = 0.0;  

                Eigen::Matrix<double,3,4> eul_angles_solved;
                Eigen::Matrix<double,3,4> translation_solved;

                if(computeFK(false, eul_angles_solved, translation_solved)){
                    // Read the current hand configuration
                    std::array<double, 3> position_difference;
                    std::array<double, 3> orientation_difference;
                    // Calculate the difference between the desired goal poses and the final hand poses
                    size_t i = 0;
                    for(int ii = goal->finger_id*n_fingers; ii < (goal->finger_id*n_fingers + finger_n_dofs); ii++){
                        position_difference[i] = abs(desired_positions[ii] - translation_solved(ii));
                        orientation_difference[i] = abs(euler_angles[ii] - eul_angles_solved(ii));
                            // // // ALTERNATIVE: Do the IK of the finger and then calculate the trajectory
                            // position_difference[i] = abs(desired_position[i] - translation_solved(ii));
                            // orientation_difference[i] = abs(euler_angles[i] - eul_angles_solved(ii));
                        additive_difference_position += position_difference[i];
                        additive_difference_orientation += orientation_difference[i];
                        if (position_difference[i] > max_difference_position_threshold) {
                            all_within_threshold = false;
                        }
                        if (orientation_difference[i] > max_difference_orientation_threshold) {
                            all_within_threshold = false;
                        }
                        // RCLCPP_INFO(node_->get_logger(), "Position des %ld : %f", i, desired_positions[ii]);
                        // RCLCPP_INFO(node_->get_logger(), "Position cur %ld : %f", i, translation_solved(ii));
                        // RCLCPP_INFO(node_->get_logger(), "Position diff %ld : %f", i, position_difference[i]);
                        // RCLCPP_INFO(node_->get_logger(), "Orientation des %ld : %f", i, euler_angles[ii]);
                        // RCLCPP_INFO(node_->get_logger(), "Orientation cur %ld : %f", i, eul_angles_solved(ii));
                        // RCLCPP_INFO(node_->get_logger(), "Orientation diff %ld : %f", i, orientation_difference[i]);
                        i ++;
                    }
                    RCLCPP_INFO(node_->get_logger(), "Total additive position difference: %f", additive_difference_position);
                    RCLCPP_INFO(node_->get_logger(), "Total additive orientation difference: %f", additive_difference_orientation);
                    // If the difference is lower than the maximum stablished thresholds success is true, false otherwise.
                    if (all_within_threshold && (additive_difference_position <= max_additive_difference_position_threshold) && (additive_difference_orientation<=max_additive_difference_orientation_threshold)) {
                        result->status = true;
                        RCLCPP_INFO(node_->get_logger(), "MoveFingerTipPoses finished successfully!");
                    } else {
                        result->status = false;
                        RCLCPP_INFO(node_->get_logger(), "MoveFingerTipPoses finished");
                    }
                }
                else{
                    result->status = false;
                    RCLCPP_INFO(node_->get_logger(), "MoveFingerTipPoses finished Error when computing final FK");
                }
                goal_handle->succeed(result);
                RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
            }
            stored_trajectory.clear();
        }
        else{
            // Error when computing the trajectory
            RCLCPP_WARN(node_->get_logger(), "Error in MoveFingerTipPoses when computing the trajectory");
        }
    }