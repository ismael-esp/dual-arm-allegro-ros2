#ifndef ALLEGROHANDCONTROLLERTORQUE_H_
#define ALLEGROHANDCONTROLLERTORQUE_H_

#include "AllegroHandController.h"

class AllegroHandControllerTorque: public AllegroHandController {
public:
    // Default Constructor
    AllegroHandControllerTorque(std::array<KDL::Chain,n_fingers> finger_chain_, std::array<KDL::Chain, n_fingers> chain_, double torqueConv_, std::vector<double> torque_max_, Eigen::Vector3d filtCoeffsPos_, Eigen::Vector3d filtCoeffsVel_);
   
    //! Control function. Torque control does not apply any control. Allows to set a desired torque.
    void control() override;
};

#endif // AHAND_H_
