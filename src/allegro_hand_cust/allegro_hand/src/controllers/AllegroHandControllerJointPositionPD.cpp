#include <allegro_hand/controllers/AllegroHandControllerJointPositionPD.h>


//! \brief Default Constructor
AllegroHandControllerJointPositionPD::AllegroHandControllerJointPositionPD(std::string controller_file_path, std::array<KDL::Chain,n_fingers> finger_chain_, std::array<KDL::Chain, n_fingers> chain_, double torqueConv_, std::vector<double> torque_max_, Eigen::Vector3d filtCoeffsPos_, Eigen::Vector3d filtCoeffsVel_) 
: AllegroHandController(finger_chain_, chain_, torqueConv_, torque_max_, filtCoeffsPos_, filtCoeffsVel_) 
{
    // Read the controller file to extract kp and kd of each joint
    if(readControllerConfigurationFile(controller_file_path)){
        std::cout << "Control yaml files have been read succesfully!" << std::endl;
    }
     // Start timer
    t_start_loop  = std::chrono::system_clock::now();
}

//! \brief Compute the torque according to a PD controller
void AllegroHandControllerJointPositionPD::control() {
    Vector torque_desired;
    //Save previous iteration values
    q_prev_filt = q_filt;
    qdot_prev = qdot;
    qdot_prev_filt = qdot_filt;
    // qddot_prev = qddot;

    // Low-pass filtering
    q_filt = filtCoeffsPos(0)* q_filt +
            filtCoeffsPos(1)* q_prev +
            filtCoeffsPos(2)* q_current;

    AllegroHandController::roundValue(q_filt);

    qdot = (q_filt - q_prev_filt) / dt;

    qdot_filt = filtCoeffsVel(0)* qdot_filt +
            filtCoeffsVel(1)* qdot_prev +
            filtCoeffsVel(2)* qdot;

    qddot = (qdot_filt-qdot_prev_filt)/dt;

    // Compute postion error
    error_q = q_desired - q_filt;

    // Compute torque to compesate gravity effects
    AllegroHandController::computeGravityCompensationTorque(q_filt, GravityCompensation);

    // Discrete PID control
    Proportional = kp_PD.cwiseProduct(error_q);
    Derivative = kd_PD.cwiseProduct(qdot_filt);

    u_k = Proportional + Derivative;

    torque_desired = (1.0 / torqueConv) * u_k;
    tau_sat = torque_desired + GravityCompensation;

    //Limit torque
    for (unsigned int i = 0; i <n_fingers*finger_n_dofs; ++i) {
        // Upper limit
        if (tau_sat(i) > torque_max[i]) {
            tau_sat(i) = torque_max[i];
            // Lower limit
        } else if (tau_sat(i) < -torque_max[i]) {
            tau_sat(i) = -torque_max[i];
        }
    }

    //Update
    mutex.lock();
    torque = tau_sat;
    vel = qdot_filt;
    acc = qddot;
    mutex.unlock();

    t_end_loop = std::chrono::system_clock::now();
    loop_time_duration = t_end_loop - t_start_loop;
    dt = loop_time_duration.count() / 1e6;
    if(dt <= 0.002){
        dt = 0.003; // When starting the controller the dt parameter is too low resulting in a D term unusualy high that does not lead to stability
    }
    t_start_loop  = std::chrono::system_clock::now();

}

bool AllegroHandControllerJointPositionPD::readControllerConfigurationFile(std::string& controller_file_path)
{
    std::vector<double> _kp, _kd, _ki;
    bool success;
    success =  readControllerParams(controller_file_path, _kp, _kd);
    setKgain(kp_PD,_kp);
    setKgain(kd_PD,_kd);
    return success;
}

bool AllegroHandControllerJointPositionPD::readControllerParams(std::string& yaml_file, std::vector<double>& _kp, std::vector<double>& _kd)
{
    YAML::Node config;
    std::cout <<"File: " << yaml_file << std::endl;
    try{
        config = YAML::LoadFile(yaml_file);
        readKgain(config, std::string("kp"), _kp);
        readKgain(config, std::string("kd"), _kd);
        return true;
    }catch(const YAML::ParserException& ex) {
        std::cout <<"Error in readControllerParams. File: " << yaml_file << std::endl;
        std::cout << ex.what() << std::endl;
        return  false;
    }
}

void AllegroHandControllerJointPositionPD::readKgain(YAML::Node& config, std::string _kgain, std::vector<double>& _k)
{
    _k.resize(16);
    _k[0] = config[_kgain]["j00"].as<double>();
    _k[1] = config[_kgain]["j01"].as<double>();
    _k[2] = config[_kgain]["j02"].as<double>();
    _k[3] = config[_kgain]["j03"].as<double>();

    _k[4] = config[_kgain]["j10"].as<double>();
    _k[5] = config[_kgain]["j11"].as<double>();
    _k[6] = config[_kgain]["j12"].as<double>();
    _k[7] = config[_kgain]["j13"].as<double>();

    _k[8] = config[_kgain]["j20"].as<double>();
    _k[9] = config[_kgain]["j21"].as<double>();
    _k[10] = config[_kgain]["j22"].as<double>();
    _k[11] = config[_kgain]["j23"].as<double>();

    _k[12] = config[_kgain]["j30"].as<double>();
    _k[13] = config[_kgain]["j31"].as<double>();
    _k[14] = config[_kgain]["j32"].as<double>();
    _k[15] = config[_kgain]["j33"].as<double>();
}

void AllegroHandControllerJointPositionPD::setKgain(Vector &K_i, const std::vector<double> &parameters){
    assert (parameters.size()==n_fingers*finger_n_dofs);
    std::cout << "Gains: ";
    for(auto i = 0; i < n_fingers*finger_n_dofs; ++i){
        K_i(i) = parameters[i];
        std::cout << K_i(i) << " ";
    }
    std::cout << std::endl;
}