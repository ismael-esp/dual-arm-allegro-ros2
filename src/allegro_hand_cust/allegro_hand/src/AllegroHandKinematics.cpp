
#include <allegro_hand/AllegroHand.h>

/**
Functions of the AllegroHand class related to kinematics:
- Jacobian
- Forward Kinematics (FK)
- Inverse Kinematics (IK)
**/


/* Functions related to the Jacobian */

//! \brief ComputeJacobian including virtual links
//! \param joints Joint values to compute jacobian
//! \return resulting jacobian matrix in JacobianKDL
void AllegroHand::ComputeJacobian(const std::vector<double> &joints, const std::vector<KDL::Frame> &contactpoints) {
    AllegroController->ComputeJacobian( joints, contactpoints);
}

//! \brief ComputeJacobian including virtual links
//! \param joints Joint values to compute jacobian
//! \return resulting jacobian matrix in JacobianKDL
void AllegroHand::ComputeJacobian(const std::vector<KDL::Frame> &contactpoints, AllegroHandDriver::Vector q_current) {
    AllegroController->ComputeJacobian(contactpoints, q_current);
}

void AllegroHand::ComputeJacobian(const std::vector<double> &joints, const std::vector<double> &cog_x, const std::vector<double> &cog_y) {
    AllegroController->ComputeJacobian(joints, cog_x, cog_y);
}

void AllegroHand::getRealJacobianKDL(std::array<Eigen::MatrixXd,n_fingers> &realJacobianKDL){
    AllegroController->getRealJacobianKDL(realJacobianKDL);
}   
// void AllegroHand::getRealJacobianKDL(std::array<Eigen::MatrixXd,n_fingers> &realJacobianKDL){
//     mutex.lock();
//     realJacobianKDL = AllegroController->RealJacobianKDL;
//     mutex.unlock();
// }

Eigen::MatrixXd AllegroHand::getRealJacobianKDLAtIndex(int index) {
    Eigen::MatrixXd jacobian_matrix;
        AllegroController->getRealJacobianKDLAtIndex(index, jacobian_matrix);
    return jacobian_matrix;
    // if (index >= 0 && index < n_fingers) {
    //     mutexKDL.lock();
    //     return AllegroController->RealJacobianKDL(index, jacobian_matrix);
    //     mutexKDL.unlock();
    // } else {
    //     // Handle invalid index.
    //     std::cout << "Invalid index of the RealJacobianKDL" << std::endl;
    //     return Eigen::MatrixXd();
    // }
}

/* End functions related to the Jacobian */


/** Functions related to IK and FK **/

/* Inverse Kinematics functions */

// bool AllegroHand::computeIKFinger(bool print_values, int finger_number, std::vector<double> desired_position, std::vector<double> desired_normal){
//     std::vector<double> q_values_finger;
//     bool success;
//     if(print_values){
//         std::cout << "Finger " << finger_number <<std::endl;
//         std::cout << "The desired position is: " << desired_position[0] <<", "<< desired_position[1] <<", " << desired_position[2] << std::endl;
//         std::cout << "The desired orientation in ZYZ eul angles is: " << desired_normal[0] <<", "<< desired_normal[1] <<", " << desired_normal[2] << std::endl;
//     }
//     if (finger_number == 3){
//         success =  thumb_finger->IkineNormal(desired_normal, desired_position);
//         q_values_finger = thumb_finger->getJointAngles();
//     }
//     if (finger_number == 0){
//         success =  index_finger->IkineNormal(desired_normal, desired_position);
//         q_values_finger = index_finger->getJointAngles();
//     }
//     else if (finger_number == 1){
//         success =  middle_finger->IkineNormal(desired_normal, desired_position);
//         q_values_finger = middle_finger->getJointAngles();
//     }
//     else if (finger_number == 2){
//         success =  anular_finger->IkineNormal(desired_normal, desired_position);
//         q_values_finger = anular_finger->getJointAngles();
//     }
//     else{
//         std::cout << "Error in computeIKFinger(): Unknown finger " << finger_number <<std::endl;
//     }
//     if(success && print_values){
//         std::cout << "The q values found from IK, are: " <<std::endl;
//         printVector(q_values_finger);
//         std::cout << std::endl;
//     }
//     return success;
// }

//! \brief  Computes the inverse kinematics of one fingertip of IMA (Index, Middle, Anular) and Thumb
//! \param  print_values true: print values on console.
//! \param  finger_number 0:Index, 1:Middle, 2:Anular, 3:Thumb
//! \param  desired_position Position (x,y,z) of the tip of the finger
//! \param  desired_orientation Orientation in ZYZ euler anles of the tip of the finger
//! \param  q_values_finger configuration values in rad of the finger
bool AllegroHand::computeIKFinger(bool print_values, int finger_number, std::vector<double> desired_position, std::vector<double> desired_orientation, std::vector<double> &q_values_finger){
    Eigen::Matrix<double, 2, 4> solutions_IK_IMA;
    std::vector<std::vector<double>> solutions_IK_Thumb;
    bool success;
    if(print_values){
        std::cout << "Finger " << finger_number <<std::endl;
        std::cout << "The desired position is: " << desired_position[0] <<", "<< desired_position[1] <<", " << desired_position[2] << std::endl;
        std::cout << "The desired orientation in ZYZ eul angles is: " << desired_orientation[0] <<", "<< desired_orientation[1] <<", " << desired_orientation[2] << std::endl;
    }
   	if(finger_number == 3){ //Thumb
        // solutions_IK_Thumb = thumb_finger->IkineThumb(desired_orientation, desired_position);
        success =  thumb_finger->IkineThumb(desired_orientation, desired_position, solutions_IK_Thumb);
        q_values_finger = thumb_finger->getJointAngles();
    }
    else { //IMA
        if (finger_number == 0){
            //solutions_IK_IMA = index_finger->Ikine(desired_orientation, desired_position);
            success =  index_finger->Ikine(desired_orientation, desired_position, solutions_IK_IMA);
            q_values_finger = index_finger->getJointAngles();
        }
        else if (finger_number == 1){
            // solutions_IK_IMA = middle_finger->Ikine(desired_orientation, desired_position);
            success =  middle_finger->Ikine(desired_orientation, desired_position, solutions_IK_IMA);
            q_values_finger = middle_finger->getJointAngles();
        }
        else if (finger_number == 2){
            success =  anular_finger->Ikine(desired_orientation, desired_position, solutions_IK_IMA);
            // solutions_IK_IMA = anular_finger->Ikine(desired_orientation, desired_position);
            q_values_finger = anular_finger->getJointAngles();
        }
        else{
            std::cout << "Error in computeIKFinger(): Unknown finger " << finger_number <<std::endl;
        }
        if(print_values){
            std::cout << "The joint values that were found are: " << solutions_IK_IMA <<std::endl;
        }
    }
    if(success && print_values){
        std::cout << "The q values found from IK, are: " <<std::endl;
        printVector(q_values_finger);
        std::cout << std::endl;
    }
    return success;
}

//! \brief  Computes the inverse kinematics of the whole hand
//! \param print_values true: print values on console.
//! \param  desired_position_hand Position (x,y,z) of the tip of all the fingertips. 4 fingers * 3 coordinates = 12 values. Order: index, middle, anular, thumb
//! \param  desired_orientation_hand Orientation in ZYZ euler anles of the tip of all the fingertips.  4 fingers * 3 angles = 12 values. Order: index, middle, anular, thumb
//! \param  q_values_hand configuration values in rad 
bool AllegroHand::computeIKHand(bool print_values, std::vector<double> desired_position_hand, std::vector<double> desired_orientation_hand, std::vector<double> &q_values_hand){
    std::vector<double> q_values_finger(4);
    std::vector<double> desired_position_finger(3);
    std::vector<double> desired_orientation_finger(3);
    bool success = true;
    bool success_one_finger = true;
    // Check that the size of the position and orientation is 12. 
    if (desired_position_hand.size() != 12) {
        std::cerr << "Error computeIKHand: Desired position vector does not contain 12 elements." << std::endl;
        success = false;
    }
    else if (desired_orientation_hand.size() != 12) {
        std::cerr << "Error computeIKHand: Desired orientation vector does not contain 12 elements." << std::endl;
        success = false;
    }
    else{
        // We divide the position and orientation vectors and calculate the q values of each finger
        for (unsigned int finger_id = 0; finger_id < 4; finger_id ++){
            desired_position_finger[0] = desired_position_hand[finger_id*3];
            desired_position_finger[1] = desired_position_hand[finger_id*3+1];
            desired_position_finger[2] = desired_position_hand[finger_id*3+2];
            desired_orientation_finger[0] = desired_orientation_hand[finger_id*3];
            desired_orientation_finger[1] = desired_orientation_hand[finger_id*3+1];
            desired_orientation_finger[2] = desired_orientation_hand[finger_id*3+2];
            success_one_finger = computeIKFinger(print_values, finger_id, desired_position_finger, desired_orientation_finger, q_values_finger);
            q_values_hand[finger_id*4] = q_values_finger[0];
            q_values_hand[finger_id*4+1] = q_values_finger[1];
            q_values_hand[finger_id*4+2] = q_values_finger[2];
            q_values_hand[finger_id*4+3] = q_values_finger[3];
            success = success && success_one_finger;
        }
    }
    return success;
}

//! \brief Executes the inverse kinematics. The fingers move to the desired position
//! \param print_values true: print values on console.
//! \param desired_position_hand Position (x,y,z) of the tip of all the fingertips. 4 fingers * 3 coordinates = 12 values. Order: index, middle, anular, thumb
//! \param desired_orientation_hand Orientation in ZYZ euler anles of the tip of all the fingertips.  4 fingers * 3 angles = 12 values. Order: index, middle, anular, thumb
void AllegroHand::executeIK(bool print_values, std::vector<double> desired_position, std::vector<double> desired_orientation){
    std::vector<double> q_values_hand(n_fingers * finger_n_dofs);
    AllegroHandDriver::Vector q_values, q_values_ik;
    
    if(print_values){ 
        std::cout <<std::endl<< "** Calculating the IK **" <<std::endl;
    }
    if(computeIKHand(print_values, desired_position, desired_orientation, q_values_hand)){
        for (std::size_t i = 0; i < q_values_hand.size(); ++i) {
            q_values_ik(i) = q_values_hand[i]; 
        }
        if(!is_left){
            getConfigsForTheOppositeHand(q_values_ik, q_values);
        }
        else{
            q_values = q_values_ik;
        }
        if(checkAndAjustJointConfigurationValues(q_values)){
            if(print_values){
                std::cout << "Moving to the q values obtained with the IK" <<std::endl;
            }
            moveToConf(q_values);
        }
    }
    else{
        std::cout << "Error computing the Inverse Kinematics" << std::endl;
    }
}

/* Forward Kinematics functions */

//! \brief Forward kinematics of a fingertip
//! \param print_values true: print values on console.
//! \param finger_number 0:Index, 1:Middle, 2:Anular, 3:Thumb
//! \param q_values_finger configuration to compute the IK of the fingertip
//! \param eul_angles_solved Orientation in euler angles of the fingertip
//! \param translation_solved Position in X,Y,Z coordinates of the fingertip
bool AllegroHand::computeFKFinger(bool print_values, unsigned int finger_number, std::vector<double> q_values_finger, Eigen::Matrix<double,3,1> &eul_angles_solved, Eigen::Matrix<double,3,1> &translation_solved){
    bool success;
	Eigen::Affine3d fk_mat;
	Eigen::Affine3d TF_from_w2bfinger, tot_mat;
    Eigen::Quaterniond new_quat_obtained;
    if(print_values){
        std::cout << "Finger " << finger_number <<std::endl;
        std::cout << "q values: " << q_values_finger[0] << ", "<< q_values_finger[1] << ", "<< q_values_finger[2] << ", "<< q_values_finger[3] << std::endl;
    }
    if(finger_number == 3){ //Thumb
        success = thumb_finger->Fkine(q_values_finger, fk_mat);
        // fk_mat = thumb_finger->Fkine(q_values_finger);
        TF_from_w2bfinger = thumb_finger->getTF_w2b() ;
        tot_mat = TF_from_w2bfinger*fk_mat*thumb_finger->getTF_tcp2tip();
    }
    else if (finger_number == 0){
        success = index_finger->Fkine(q_values_finger, fk_mat);
        // fk_mat = index_finger->Fkine(q_values_finger);
        TF_from_w2bfinger = index_finger->getTF_w2b() ;
        tot_mat = TF_from_w2bfinger*fk_mat*index_finger->getTF_tcp2tip();
    }
    else if (finger_number == 1){
        success = middle_finger->Fkine(q_values_finger, fk_mat);
        // fk_mat = middle_finger->Fkine(q_values_finger);
        TF_from_w2bfinger = middle_finger->getTF_w2b() ;
        tot_mat = TF_from_w2bfinger*fk_mat*middle_finger->getTF_tcp2tip();
    }
    else if (finger_number == 2){
        success = anular_finger->Fkine(q_values_finger, fk_mat);
        // fk_mat = anular_finger->Fkine(q_values_finger);
        TF_from_w2bfinger = anular_finger->getTF_w2b() ;
        tot_mat = TF_from_w2bfinger*fk_mat*anular_finger->getTF_tcp2tip();
    }
    else{
        std::cout << "Error in computeFKFinger(): Unknown finger " << finger_number <<std::endl;
        success = false;
    }
    //std::cout << "TF_from_w2bfinger is: " << TF_from_w2bfinger <<std::endl;
    if(success){
        new_quat_obtained = obtainQuaternion(tot_mat);
        eul_angles_solved = obtainEulAngles(tot_mat);
        translation_solved = obtainTranslation(tot_mat);
        if(print_values){
            std::cout << "The FK matrix is: " <<std::endl<< tot_mat.matrix() << std::endl;
            std::cout << "The equivalent quaternion element 'w' is: " << new_quat_obtained.w() <<std::endl;
            std::cout << "The equivalent quaternion element 'vec' is: " << std::endl << new_quat_obtained.vec() <<std::endl;
            std::cout << "The equivalent in euler angles (ZYZ) is: " << std::endl << eul_angles_solved <<std::endl;
            std::cout << "The tanslation vector is: " << std::endl << translation_solved <<std::endl;
            std::cout << std::endl;
        }
    }
    else{
        std::cout << std::endl << "Error computing the Forward Kinematics. \nPlease make sure that the provided joint values are within the limits." << std::endl;
    }
    return success;
}

//! \brief Forward kinematics of all the fingertips
//! \param print_values true: print values FK of each fingertip on the console.
//! \param  q_values_hand configuration to compute the IK of the fingertips
//! \param eul_angles_solved Orientation in euler angles of the fingertips
//! \param translation_solved Position in X,Y,Z coordinates of the fingertips
bool AllegroHand::computeFK(bool print_values, std::vector<double> q_values_hand, Eigen::Matrix<double,3,4> &eul_angles_solved, Eigen::Matrix<double,3,4> &translation_solved){
    bool success = false;
    bool success_one_finger;
    std::vector<double> q_values_finger(4);
    Eigen::Matrix<double,3,1> euler_angles_finger;
    Eigen::Matrix<double,3,1> position_finger;
    // Check that the size of the position and orientation is 12. 
    if (q_values_hand.size() != n_fingers * finger_n_dofs) {
        std::cerr << "Error computeFK: Desired joint position vector does not contain "<< n_fingers * finger_n_dofs <<" elements." << std::endl;
    }
    else {
    // else if(checkJointConfigurationValuesWithinLimits(q_values_hand)){
        if(!is_left){
            q_values_hand[0] = -q_values_hand[0];
            q_values_hand[4] = -q_values_hand[4];
            q_values_hand[8] = -q_values_hand[8];
        }
        success = true;
        // We divide the position and orientation vectors and calculate the q values of each finger
        for (unsigned int finger_id = 0; finger_id < 4; finger_id ++){
            q_values_finger[0] = q_values_hand[finger_id*4];
            q_values_finger[1] = q_values_hand[finger_id*4+1];
            q_values_finger[2] = q_values_hand[finger_id*4+2];
            q_values_finger[3] = q_values_hand[finger_id*4+3];
            success_one_finger = computeFKFinger(print_values, finger_id, q_values_finger, euler_angles_finger, position_finger);
            eul_angles_solved.col(finger_id) = euler_angles_finger;
            translation_solved.col(finger_id) = position_finger;
            success = success && success_one_finger;
        }
    }
    if(!success){
        std::cout << "*** Error computing FK ***";
    }
    return success;
}

//! \brief Forward kinematics of all the fingertips of the current configuration
//! \param print_values true: print values FK of each fingertip on the console.
//! \param eul_angles_solved Orientation in euler angles of the fingertips
//! \param translation_solved Position in X,Y,Z coordinates of the fingertips
bool AllegroHand::computeFK(bool print_values, Eigen::Matrix<double,3,4> &eul_angles_solved, Eigen::Matrix<double,3,4> &translation_solved){
    bool success;
    std::vector<double> q_values_hand(n_fingers * finger_n_dofs);
    AllegroHandDriver::Vector q_current_;
    mutex.lock();
        q_current_ = q_current;
    mutex.unlock();
    for (std::size_t i = 0; i < q_values_hand.size(); ++i) {
        q_values_hand[i] = q_current_(i); 
        // std::cout << "q value (" << i << "): "<< q_values_hand[i];
    }
    // std::cout <<std::endl;
    success = computeFK(print_values, q_values_hand, eul_angles_solved, translation_solved);
    return success;
}


/* Function related to type I motions*/
void AllegroHand::calculeMotion1Finger(unsigned int finger_number, std::vector<double> desired_pos, std::vector<double> desired_orntn, std::vector<std::vector<double>> &motion1_sols_finger){
    std::cout << "Finger " << finger_number <<std::endl;
	std::cout << "The desired position is: " << desired_pos[0] <<", "<< desired_pos[1] <<", " << desired_pos[2] << std::endl;
	std::cout << "The desired orientation in ZYZ eul angles is: " << desired_orntn[0] <<", "<< desired_orntn[1] <<", " << desired_orntn[2] << std::endl;

    if (finger_number == 0){
        motion1_sols_finger = index_finger->obtainMotion1Sols(desired_orntn, desired_pos);
    }
    else if (finger_number == 1){
        motion1_sols_finger = middle_finger->obtainMotion1Sols(desired_orntn, desired_pos);
    }
    else if (finger_number == 2){
        motion1_sols_finger = anular_finger->obtainMotion1Sols(desired_orntn, desired_pos);
    }
    else{
        std::cout << "Error in calculeMotion1Finger(): Unknown finger " << finger_number <<std::endl;
    }
	
    std::cout << "motion1_sols_finger: " <<std::endl;
	for(auto i = 0u; i<motion1_sols_finger.size(); i++)
	{
		std::vector<double> vect_aux;
		vect_aux = motion1_sols_finger[i];
		for(auto j = 0u; j<vect_aux.size(); j++)
		{
			std::cout <<vect_aux[j]<< " ";
		}
		std::cout << std::endl;
	}
}

void AllegroHand::calculeMotion2Finger(unsigned int finger_number, std::vector<double> desired_pos, std::vector<double> desired_orntn, std::vector<std::vector<double>> &motion2_sols_finger){
    std::cout << "Finger " << finger_number <<std::endl;
	std::cout << "The desired position is: " << desired_pos[0] <<", "<< desired_pos[1] <<", " << desired_pos[2] << std::endl;
	std::cout << "The desired orientation in ZYZ eul angles is: " << desired_orntn[0] <<", "<< desired_orntn[1] <<", " << desired_orntn[2] << std::endl;

    if (finger_number == 0){
        motion2_sols_finger = index_finger->obtainMotion2Sols(desired_orntn, desired_pos);
    }
    else if (finger_number == 1){
        motion2_sols_finger = middle_finger->obtainMotion2Sols(desired_orntn, desired_pos);
    }
    else if (finger_number == 2){
        motion2_sols_finger = anular_finger->obtainMotion2Sols(desired_orntn, desired_pos);
    }
    else{
        std::cout << "Error in calculeMotion2Finger(): Unknown finger " << finger_number <<std::endl;
    }
	
    std::cout << "motion2_sols_finger: " <<std::endl;
	for(auto i = 0u; i<motion2_sols_finger.size(); i++)
	{
		std::vector<double> vect_aux;
		vect_aux = motion2_sols_finger[i];
		for(auto j = 0u; j<vect_aux.size(); j++)
		{
			std::cout <<vect_aux[j]<< " ";
		}
		std::cout << std::endl;
	}
}

void AllegroHand::calculeMotionHand(unsigned int which_motion, std::vector<double> desired_pos, std::vector<double> desired_orntn){
    std::vector<std::vector<double>> motion1_sols;
    std::vector<std::vector<double>> motion_sols_finger_I;
    std::vector<std::vector<double>> motion_sols_finger_M;
    std::vector<std::vector<double>> motion_sols_finger_A;
    unsigned int num_coordinates = 3;
    if(which_motion == 1){
        calculeMotion1Finger(0, std::vector<double>(desired_orntn.begin(), desired_orntn.begin() + num_coordinates), std::vector<double>(desired_pos.begin(), desired_pos.begin() + num_coordinates), motion_sols_finger_I);
        calculeMotion1Finger(1, std::vector<double>(desired_orntn.begin()+ num_coordinates, desired_orntn.begin() + 2*num_coordinates), std::vector<double>(desired_pos.begin()+ num_coordinates, desired_pos.begin() + 2*num_coordinates), motion_sols_finger_M);
        calculeMotion1Finger(2, std::vector<double>(desired_orntn.begin()+ 2*num_coordinates, desired_orntn.begin() + 3*num_coordinates), std::vector<double>(desired_pos.begin()+ 2*num_coordinates, desired_pos.begin() + 3*num_coordinates), motion_sols_finger_A);
    }
    else if (which_motion == 2){
        calculeMotion2Finger(0, std::vector<double>(desired_orntn.begin(), desired_orntn.begin() + num_coordinates), std::vector<double>(desired_pos.begin(), desired_pos.begin() + num_coordinates), motion_sols_finger_I);
        calculeMotion2Finger(1, std::vector<double>(desired_orntn.begin()+ num_coordinates, desired_orntn.begin() + 2*num_coordinates), std::vector<double>(desired_pos.begin()+ num_coordinates, desired_pos.begin() + 2*num_coordinates), motion_sols_finger_M);
        calculeMotion2Finger(2, std::vector<double>(desired_orntn.begin()+ 2*num_coordinates, desired_orntn.begin() + 3*num_coordinates), std::vector<double>(desired_pos.begin()+ 2*num_coordinates, desired_pos.begin() + 3*num_coordinates), motion_sols_finger_A);

    }
    else{
        std::cout << "Error: Unknown desired motion" << std::endl;
        return;
    }
    unsigned int size_motions = std::min({motion_sols_finger_I.size(), motion_sols_finger_M.size(), motion_sols_finger_A.size()});
    for(unsigned int num_motions = 0; num_motions<size_motions; num_motions++){
       std::vector<double> row;
        // Append values from motion_sols_finger_I
        row.insert(row.end(), motion_sols_finger_I[num_motions].begin(), motion_sols_finger_I[num_motions].end());
        // Append values from motion_sols_finger_M
        row.insert(row.end(), motion_sols_finger_M[num_motions].begin(), motion_sols_finger_M[num_motions].end());
        // Append values from motion_sols_finger_A
        row.insert(row.end(), motion_sols_finger_A[num_motions].begin(), motion_sols_finger_A[num_motions].end());
        
        // Thumb not calculated thus minimum selected:
        row.insert(row.end(), {q_min[12],q_min[13],q_min[14],q_min[15]});
  
        // Append the row to motion1_sols
        motion1_sols.push_back(row);
    }
    
     std::cout << " Motion 1 sols values: ";
     for (const auto& row : motion1_sols) {
        for (const auto& val : row) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }

    AllegroHandDriver::Vector q_values;

    for (const auto& row : motion1_sols) {
        for (std::size_t i = 0; i < row.size(); ++i) {
                std::cout << row[i] << " ";
                q_values(i) = row[i];
        }
         std::cout << std::endl;
        if(checkAndAjustJointConfigurationValues(q_values)){
            std::cout << "Moving to the q values of the motion 1" <<std::endl;
            
            moveToConf(q_values);
            // std::cout << "Waiting " << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*10000)));
        }
    }

}