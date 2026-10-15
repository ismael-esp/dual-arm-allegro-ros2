#include <allegro_hand/controllers/AllegroHandControllerTorque.h>

//! \brief Default Constructor
AllegroHandControllerTorque::AllegroHandControllerTorque(std::array<KDL::Chain,n_fingers> finger_chain_, std::array<KDL::Chain, n_fingers> chain_, double torqueConv_, std::vector<double> torque_max_, Eigen::Vector3d filtCoeffsPos_, Eigen::Vector3d filtCoeffsVel_) 
: AllegroHandController(finger_chain_, chain_, torqueConv_, torque_max_, filtCoeffsPos_, filtCoeffsVel_) 
{
    torque_desired = Vector::Zero();
}

//! \brief Empty function that allows to set the desired torque without applying any controller
void AllegroHandControllerTorque::control() {
    // Update torque variable
    mutex.lock();
    torque = torque_desired;
    mutex.unlock();
}