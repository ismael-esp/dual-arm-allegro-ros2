#ifndef ALLEGROHANDCONTROLLERPID_H_
#define ALLEGROHANDCONTROLLERPID_H_

#include "AllegroHandController.h"

class AllegroHandControllerJointPositionPID: public AllegroHandController {
public:
    // Default Constructor
    AllegroHandControllerJointPositionPID(std::string controller_file_path, std::array<KDL::Chain,n_fingers> finger_chain_, std::array<KDL::Chain, n_fingers> chain_, double torqueConv_, std::vector<double> torque_max_, Eigen::Vector3d filtCoeffsPos_, Eigen::Vector3d filtCoeffsVel_);
   
    //! Control function. To compute the joint torque using the PID controller
    void control() override;



private:
    //Control variables
    Vector kp_PID, ki_PID, kd_PID;
    Vector Proportional, Integral, Derivative;
    // Read the controller file and save kp and kd of each joint
    bool readControllerConfigurationFile(std::string& controller_file_path);
    // Read the controller file and extract kp and kd of each joint
    bool readControllerParams(std::string& yaml_file, std::vector<double>& _kp, std::vector<double>& _ki, std::vector<double>& _kd);
    // Save kp and kd of each joint
    void readKgain(YAML::Node& config, std::string _kgain, std::vector<double>& _k);
    // Set desired k (kp or kd) of each joint
    void setKgain(Vector &K, const std::vector<double> &parameters);
    // Time related variables
    std::chrono::system_clock::time_point t_start_loop, t_end_loop;
    std::chrono::duration<double, std::micro> loop_time_duration;
};

#endif // AHAND_H_
