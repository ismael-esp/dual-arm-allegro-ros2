#ifndef ALLEGROHANDCONTROLLERGRAVITYCOMPENSATOR_H_
#define ALLEGROHANDCONTROLLERGRAVITYCOMPENSATOR_H_

#include "AllegroHandController.h"

class AllegroHandControllerGravityCompensator: public AllegroHandController {
public:
    // Default Constructor
    AllegroHandControllerGravityCompensator(std::array<KDL::Chain,n_fingers> finger_chain_, std::array<KDL::Chain, n_fingers> chain_, double torqueConv_, std::vector<double> torque_max_, Eigen::Vector3d filtCoeffsPos_, Eigen::Vector3d filtCoeffsVel_);
   
    //! Control function. To compute the joint torque using the gravity compensator controller
    void control() override;
};

#endif // AHAND_H_
