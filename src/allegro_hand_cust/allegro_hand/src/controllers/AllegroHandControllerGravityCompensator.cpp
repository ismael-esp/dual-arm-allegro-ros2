#include <allegro_hand/controllers/AllegroHandControllerGravityCompensator.h>

//! \brief Default Constructor
AllegroHandControllerGravityCompensator::AllegroHandControllerGravityCompensator(std::array<KDL::Chain,n_fingers> finger_chain_, std::array<KDL::Chain, n_fingers> chain_, double torqueConv_, std::vector<double> torque_max_, Eigen::Vector3d filtCoeffsPos_, Eigen::Vector3d filtCoeffsVel_) 
: AllegroHandController(finger_chain_, chain_, torqueConv_, torque_max_, filtCoeffsPos_, filtCoeffsVel_) 
{
    
}

//! \brief Compute the torque in order to compensate the gravity
void AllegroHandControllerGravityCompensator::control() {
     
    // Low-pass filtering
    q_filt = filtCoeffsPos(0)* q_filt +
            filtCoeffsPos(1)* q_prev +
            filtCoeffsPos(2)* q_current;

    AllegroHandController::roundValue(q_filt);

    AllegroHandController::computeGravityCompensationTorque(q_filt, GravityCompensation);

    mutex.lock();
        torque = GravityCompensation;
    mutex.unlock();
}