
#include <allegro_hand/AllegroHand.h>

/**
Functions of the AllegroHand class related to the joint limits
**/

//! \brief Get the joint limits and print them
void AllegroHand::getJointLimits(){
    for (unsigned int i = 0; i < n_fingers * finger_n_dofs; ++i) {
       std::cout << joint_prefix << "joint_" << i <<"_0: q_min:" << q_min[i] << ", q_max:" << q_max[i] << std::endl;
    }
}
//! \brief Get the joint limits and print them
//! \param q_min_ Joint  minimum value in radians
//! \param q_max_ Joint maximum value in radians
void AllegroHand::getJointLimits(std::vector<double> &q_min_, std::vector<double> &q_max_){
    // Update the values with the current ones.
    q_min_ = q_min;
    q_max_ = q_max;
    // Print the values
    if(!use_safety_margin_joint_limits){
        for (unsigned int i = 0; i < n_fingers * finger_n_dofs; ++i) {
            std::cout << joint_prefix << "joint_" << i <<"_0: q_min:" << q_min[i] << ", q_max:" << q_max[i] << std::endl;
        }
    }
    else{
        for (unsigned int i = 0; i < n_fingers * finger_n_dofs; ++i) {
            std::cout << joint_prefix << "joint_" << i << "_0: " << std::endl;
            std::cout << "       urdf:               q_min:" << q_min_urdf_unsaturated[i] << ", q_max:" << q_max_urdf_unsaturated[i] << std::endl;
            std::cout << "       with safety margin: q_min:" << q_min[i] << ", q_max:" << q_max[i] << std::endl;
            std::cout << "       safety margin:      q_min:" << safety_margin_joint_limits_values[i] << ", q_max:" << safety_margin_joint_limits_values[i] << std::endl;
        }
    }
}

//! \brief Update the values of the joint limits
void AllegroHand::updateJointLimits(){
    std::cout << "Updating joint limits..." << std::endl;
    std::vector<double> q_min_, q_max_;
    // If we want to use safety margin
    if(use_safety_margin_joint_limits){
        for (unsigned int i = 0; i < n_fingers * finger_n_dofs; ++i) {            
            q_min_.push_back(q_min_urdf_unsaturated[i] + safety_margin_joint_limits_values[i]);
            q_max_.push_back(q_max_urdf_unsaturated[i] - safety_margin_joint_limits_values[i]);
        }
    }
    else{
        for (unsigned int i = 0; i < n_fingers * finger_n_dofs; ++i) {
            q_min_.push_back(q_min_urdf_unsaturated[i]);
            q_max_.push_back(q_max_urdf_unsaturated[i]);
        }
    }
    checkNewJointLimitsWithinJointLimits(q_min_, q_max_);
    q_min = q_min_;
    q_max = q_max_;
    std::cout << "Joint limits have been updated" << std::endl;
}

//! \brief Update joint limits URDF to avoid saturated values regarding the real joint limits
void AllegroHand::updateJointLimitsURDF(){
    // Set the same values
    q_min_urdf_unsaturated = q_min_urdf;
    q_max_urdf_unsaturated = q_max_urdf;
    // If there exists a file containing the actual joint limits
    if(readJointLimits()){
        std::cout << "Avoiding joint limits from the URDF saturation with the real joint limits values" << std::endl;
        for (unsigned int i = 0; i < n_fingers * finger_n_dofs; ++i) {
            std::cout << joint_prefix << "joint_" << i <<"_0: q_min:" << q_min_urdf_unsaturated[i] << ", q_max:" << q_max_urdf[i];
            // std::cout << std::endl;
            // std::cout << joint_prefix << "joint_" << i <<"_0: q_min_real:" << q_min_real[i] << ", q_max_real:" << q_max_real[i];
            if(q_min_urdf[i] < q_min_real[i]){
                q_min_urdf_unsaturated[i] = q_min_real[i];
                std::cout << " <<<  q_min saturated: " << q_min_urdf_unsaturated[i];
            }
            if(q_max_urdf[i] > q_max_real[i]){
                q_max_urdf_unsaturated[i] = q_max_real[i];
                std::cout << " <<<  q_max saturated: " << q_max_urdf_unsaturated[i];
            }
            std::cout << std::endl;
        }
    }
}
//! \brief Check that the joint limit values are within the limits of the actual hand
// These limits have been obtained using examples/calculate_min_max_joints.cpp file that stores a yaml file in conf/joints_limits
void AllegroHand::checkNewJointLimitsWithinJointLimits(std::vector<double> &q_min_, std::vector<double> &q_max_){
    std::cout << "Avoiding new joint limits saturation with joint limits of urdf unsaturated" << std::endl;
    for (unsigned int i = 0; i < n_fingers * finger_n_dofs; ++i) {
        std::cout << joint_prefix << "joint_" << i <<"_0: q_min:" << q_min_[i] << ", q_max:" << q_max_[i];
        if(q_min_[i] < q_min_urdf_unsaturated[i]){
            q_min_[i] = q_min_urdf_unsaturated[i];
            std::cout << " <<<  q_min saturated: " << q_min_[i];
        }
        if(q_max_[i] > q_max_urdf_unsaturated[i]){
            q_max_[i] = q_max_urdf_unsaturated[i];
            std::cout << " <<<  q_max saturated: " << q_max_[i];
        }
        std::cout << std::endl;
    }
}

//! \brief Read joint limits from a yaml file in conf/joints_limits
bool AllegroHand::readJointLimits(){
    YAML::Node config;
    std::string joint_limits_file_path = directory_conf_folder + "/joints_limits/" + hand_name + ".yaml";
    // Check that the file exists:
    std::ifstream fin(joint_limits_file_path);
    if (!fin) {
        std::cout << "This is not an important issue (readJointLimits): ";
        std::cout << "Unable to open file " << joint_limits_file_path << std::endl;
        return false;
    }
    else{
        try{
            config = YAML::LoadFile(joint_limits_file_path);
            for (const auto& finger : config) {
                for (const auto& joint : finger.second) {
                    q_min_real.push_back(joint.second["min_value"].as<double>());
                    q_max_real.push_back(joint.second["max_value"].as<double>());
                }
            }
            std::cout << "Real Joint Limit Values from conf/joints_limits folder: " << std::endl;
            for (unsigned int i = 0; i < n_fingers * finger_n_dofs; ++i) {
                std::cout << joint_prefix << "joint_" << i <<"_0: q_min:" << q_min_real[i] << ", q_max:" << q_max_real[i] << std::endl;
            }
            return true;
        }catch(const YAML::ParserException& ex) {
            std::cout <<"Error in readJointLimits. File: " << joint_limits_file_path << std::endl;
            std::cout << ex.what() << std::endl;
            return  false;
        }
    }
}

//! \brief Get the safety margin of the joint limits
//! \param safety_margin_joint_limits_values_ Safety margin for the joint limit in radiants of each joint.
bool AllegroHand::getJointLimitsSafetyMargin(std::vector<double> &safety_margin_joint_limits_values_){
    safety_margin_joint_limits_values_ = safety_margin_joint_limits_values;
    // Print the values
    if(!use_safety_margin_joint_limits){
        std::cout << "The safety margin for the joint limits is not being used" << std::endl;
        for (unsigned int i = 0; i < n_fingers * finger_n_dofs; ++i) {
            std::cout << joint_prefix << "joint_" << i <<"_0: q_min:" << q_min[i] << ", q_max:" << q_max[i] << std::endl;
        }
        return false;
    }
    else{
        for (unsigned int i = 0; i < n_fingers * finger_n_dofs; ++i) {
            std::cout << joint_prefix << "joint_" << i << "_0: " << std::endl;
            std::cout << "       urdf:               q_min:" << q_min_urdf_unsaturated[i] << ", q_max:" << q_max_urdf_unsaturated[i] << std::endl;
            std::cout << "       with safety margin: q_min:" << q_min[i] << ", q_max:" << q_max[i] << std::endl;
            std::cout << "       safety margin:      q_min:" << safety_margin_joint_limits_values[i] << ", q_max:" << safety_margin_joint_limits_values[i] << std::endl;
        }
    }
    return true;
}

//! \brief  Set Joint Limits Safety Margin. Set a percentage for them all
//! \param safety_margin_percentage. Percentage (between 0 and 1) of safety margin. Percentage of the range defined in the urdf
void AllegroHand::setJointLimitsSafetyMargin(double safety_margin_percentage){
    use_safety_margin_joint_limits = true;
    for (unsigned int i = 0; i < n_fingers * finger_n_dofs; ++i) {
        safety_margin_joint_limits_values[i] = (-q_min_urdf_unsaturated[i]+q_max_urdf_unsaturated[i]) * safety_margin_percentage;
    }
    updateJointLimits();
}

//! \brief  Set Joint Limits Safety Margin. Set a specific value for them all
//! \param safety_margin_joint_limits_values_ Safety margin for the joint limit in radiants of each joint
void AllegroHand::setJointLimitsSafetyMargin(std::vector<double> safety_margin_joint_limits_values_){
    use_safety_margin_joint_limits = true;
    for (unsigned int i = 0; i < n_fingers * finger_n_dofs; ++i) {
        if(safety_margin_joint_limits_values_[i] >= abs(q_min_urdf_unsaturated[i])){
            std::cout << "Error: in joint " << i << "safety margin >= q_min_urdf" << std::endl;
            safety_margin_joint_limits_values[i] = 0;
        }
        else if(safety_margin_joint_limits_values_[i] >= q_max_urdf_unsaturated[i]){
            std::cout << "Error: in joint " << i << "safety margin >= q_max_urdf" << std::endl;
            safety_margin_joint_limits_values[i] = 0;
        }
        else{
            safety_margin_joint_limits_values[i] = safety_margin_joint_limits_values_[i];
        }
        // std::cout << joint_prefix << "joint_" << i <<"_0: q_min:" << q_min[i] << ", q_max:" << q_max[i] << std::endl;
    }
    updateJointLimits();
}

void AllegroHand::useSafetyMargin(bool use_safety_margin_joint_limits_){
    use_safety_margin_joint_limits = use_safety_margin_joint_limits_;
    updateJointLimits();
}

/* End related to joint limits*/