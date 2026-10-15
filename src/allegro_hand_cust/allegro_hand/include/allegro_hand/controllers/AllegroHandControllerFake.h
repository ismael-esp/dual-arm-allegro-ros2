#ifndef ALLEGROHANDCONTROLLERFAKE_H_
#define ALLEGROHANDCONTROLLERFAKE_H_

#include "AllegroHandController.h"

class AllegroHandControllerFake: public AllegroHandController {
public:
    // Default Constructor
    AllegroHandControllerFake(std::array<KDL::Chain,n_fingers> finger_chain_, std::array<KDL::Chain, n_fingers> chain_, double torqueConv_, std::vector<double> torque_max_, Eigen::Vector3d filtCoeffsPos_, Eigen::Vector3d filtCoeffsVel_);
   
    //! Control function. Torque control does not apply any control. Allows to set a desired torque.
    void control() override;

private:
    // Time related variables
    std::chrono::system_clock::time_point t_start_loop, t_end_loop;
    std::chrono::duration<double, std::micro> loop_time_duration;
};

#endif // AHAND_H_
