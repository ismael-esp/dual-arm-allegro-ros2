#ifndef ALLEGROHANDCONTROLLER_H_
#define ALLEGROHANDCONTROLLER_H_

#include "../dynamics_AH.h"
// #include "CAN_AH.h"

#include <kdl/chain.hpp>
#include <thread>
#include <mutex>

#include <urdf/model.h>
#include <kdl/frames_io.hpp>
#include "../kdl_parser/kdl_parser.hpp"

#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <list>
#include <yaml-cpp/yaml.h>



#define DEGREES_TO_RADIANS(angle) angle*M_PI / 180
#define RADIANS_TO_DEGREES(angle) angle*180 / M_PI

class AllegroHandController {
public:

    static constexpr int n_fingers = 4;
    static constexpr int finger_n_dofs = 4;

    typedef Eigen::Matrix<double, n_fingers*finger_n_dofs, 1> Vector;

    //! Initializes the Controller class
    AllegroHandController(std::array<KDL::Chain,n_fingers> finger_chain_, std::array<KDL::Chain, n_fingers> chain_, double torqueConv_, std::vector<double> torque_max_, Eigen::Vector3d filtCoeffsPos_, Eigen::Vector3d filtCoeffsVel_);
    virtual ~AllegroHandController() {}

    // Set and Get Gravity Vector
    void setGravityVector(KDL::Vector &gravity_);
    void getGravityVector(KDL::Vector &gravity_);
    void getGravityComp(Vector &gc_);

    // Get velocity and acceleration of the real hand
    inline void getCurrentVel(Vector &vel_){vel_ = vel;}
    inline void getCurrentAcc(Vector &acc_){acc_ = acc;}

    // Jacobian
    void ComputeJacobian(const std::vector<double> &joints, const std::vector<double> &cog_x, const std::vector<double> &cog_y);
    void ComputeJacobian(const std::vector<double> &joints, const std::vector<KDL::Frame> &contactpoints);
    void ComputeJacobian(const std::vector<KDL::Frame> &contactpoints,  Vector q_current);
    void ComputeFingerFrames(const std::vector<double> &joints, const std::vector<double> &cog_x, const std::vector<double> &cog_y);
    inline void getFingerFrames(std::array<std::vector<KDL::Frame>, n_fingers> &fingerFrames){ mutex.lock(); fingerFrames = FingersFrames;  mutex.unlock();}

    // Accessing to the KDL Chain
    // return copy
    // with finger you choose which finger do you want to obtain
    // virtual, you get the KDL::Chain with virtual links (mass = 0)
    // is false, you get the real chain without virtual links
    KDL::Chain getKDLChain(size_t finger, bool virt);
    std::array<Eigen::MatrixXd,n_fingers> RealJacobianKDL;
    std::array<Eigen::MatrixXd,n_fingers> RealJacobianTransposeKDL;
    std::array<std::vector<KDL::Frame>,n_fingers> FingersFrames;

    // Execute controller. Updates the values, executes the control and updates the torque accordingly
    void executeController(Vector q_prev, Vector q_current, Vector &torque, Vector q_desired);
    // void executeController(Vector q_prev, Vector q_current, Vector &torque, Vector q_desired, Vector torque_desired);
    //! Control function: To compute the joint torque depending on the selected controller
    // Basic controller = Torque controller
    virtual void control() {torque = torque_desired;} 

    void getRealJacobianKDL(std::array<Eigen::MatrixXd,n_fingers> &realJacobianKDL);
    void getRealJacobianKDLAtIndex(int index, Eigen::MatrixXd &realJacobianKDL_);

protected:

    void roundValue(Vector &val);

    //!KDL chain with virtual joints for FK
    std::array<KDL::Chain,n_fingers> chain;
    //!KDL chain without virtual joints for IK
    std::array<KDL::Chain,n_fingers> finger_chain;

    std::array<KDL::ChainJntToJacSolver*, n_fingers> RealJacobianSolver;

    std::array<KDL::ChainFkSolverPos_recursive*,n_fingers> FKSolver;

    std::mutex mutex;
    std::mutex mutexKDL;

    KDL::Twist ag;
    std::array<KDL::Wrench,n_fingers> w;

    KDL::Vector gravity;

    //Hand limits
    std::vector<double> torque_max;

    //Joint friction
    Vector mu_s, mu_v;

    //Current state
    Vector vel, acc, torque;
    Vector G_compensation;

    Vector integral;

   //Controller parameters
    Vector q_prev, q_current, q_desired, torque_desired;
    Vector q_filt, q_prev_filt;
    Vector qdot, qdot_prev;
    Vector qdot_filt, qdot_prev_filt;
    Vector qddot;
    Vector q_des;
    Vector error_q, error_prev;
    double torqueConv;
    Vector tau_sat;
    Vector u_k;
    double dt;
    Vector GravityCompensation;
    Eigen::Vector3d filtCoeffsPos, filtCoeffsVel;
    // Gravity compensation Parameters
    KDL::Twist acctwist;

    bool setAccelerationTwist(const KDL::Twist &ag_);
    bool setFingertipExternalWrench(const std::array<KDL::Wrench,n_fingers> &w_);
    void computeGravityCompensationTorque(Vector q, Vector &gc);

    // Update values to calculate the torque according to the controller
    void setValues(Vector q_prev_, Vector q_current_, Vector q_desired_, Vector torque_desired_);

};

#endif // AHAND_H_
