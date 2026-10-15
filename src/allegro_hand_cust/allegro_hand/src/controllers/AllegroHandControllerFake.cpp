#include <allegro_hand/controllers/AllegroHandControllerFake.h>


//! \brief Default Constructor
AllegroHandControllerFake::AllegroHandControllerFake(std::array<KDL::Chain,n_fingers> finger_chain_, std::array<KDL::Chain, n_fingers> chain_, double torqueConv_, std::vector<double> torque_max_, Eigen::Vector3d filtCoeffsPos_, Eigen::Vector3d filtCoeffsVel_) 
: AllegroHandController(finger_chain_, chain_, torqueConv_, torque_max_, filtCoeffsPos_, filtCoeffsVel_) 
{
    // Start timer
    dt = 0.002;
}

//! \brief Loop to compute torques every 4 ms in a separated thread
//! WARNING: It's a fake controller for simulation
void AllegroHandControllerFake::control() {
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

    AllegroHandController::computeGravityCompensationTorque(q_filt, GravityCompensation);
    
    mutex.lock();
        // tau vector is the gravity compensation torques
        torque = GravityCompensation;
        // update the state in a fakemode
        vel = qdot_filt;
        acc = qddot;
    mutex.unlock();
}