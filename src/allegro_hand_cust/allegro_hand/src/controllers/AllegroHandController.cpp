/***************************************************************************
 *   Copyright (C) 2021 by Andres Montano                                  *
 *   andres.felipe.montano@upc.edu                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 ***************************************************************************/

#include <allegro_hand/controllers/AllegroHandController.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
#include <filesystem>
#include <iterator>
#include <yaml-cpp/exceptions.h>
#include <Eigen/QR>
#include <ctime>
#include <iostream>   // std::cout, std::ios
#include <sstream>    // std::ostringstream
#include <fstream>    //std::ofstream

#include <boost/signals2/signal.hpp> // Signal library
#include <boost/asio.hpp>


//! \brief Round a vector keeping 3 decimals
//! \param val Vector to round
void AllegroHandController::roundValue(AllegroHandController::Vector &val) {
    double tval;
    for (auto i = 0; i < 16; ++i) {
        val(i) *= 1000.;
        tval = trunc(val(i));
        tval /= 1000.;
        val(i) = tval;
    }
}

//! \brief Constructor Default constructor
//! \param model Model of the hand {F,L,R}
//! \param channel
//! \return resulting jacobian matrix in JacobianKDL

AllegroHandController::AllegroHandController(
    std::array<KDL::Chain, n_fingers> finger_chain_, 
    std::array<KDL::Chain, n_fingers> chain_,
    double torqueConv_,
    std::vector<double> torque_max_,
    Eigen::Vector3d filtCoeffsPos_, 
    Eigen::Vector3d filtCoeffsVel_)
    : chain(chain_),   
      finger_chain(finger_chain_),  
      gravity(KDL::Vector(0., 0., -9.80665)),  
      torque_max(torque_max_),
      torqueConv(torqueConv_),
      dt(0.0030), 
      filtCoeffsPos(filtCoeffsPos_), 
      filtCoeffsVel(filtCoeffsVel_),  
      q_filt(Vector::Zero()),  
      qdot(Vector::Zero()),   
      qdot_filt(Vector::Zero()), 
      error_q(Vector::Zero()), 
      vel(Vector::Zero()),  
      acc(Vector::Zero()), 
      torque(Vector::Zero()),  
      G_compensation(Vector::Zero()),  
      integral(Vector::Zero()),  
      q_prev(Vector::Zero()),  
      q_current(Vector::Zero()), 
      q_desired(Vector::Zero()), 
      torque_desired(Vector::Zero()),
      q_prev_filt(Vector::Zero()), 
      qdot_prev(Vector::Zero()),  
      qdot_prev_filt(Vector::Zero()),
      qddot(Vector::Zero()), 
      q_des(Vector::Zero()), 
      error_prev(Vector::Zero()), 
      tau_sat(Vector::Zero()), 
      u_k(Vector::Zero()), 
      GravityCompensation(Vector::Zero())  
{
}


void AllegroHandController::setValues(AllegroHandController::Vector q_prev_, AllegroHandController::Vector q_current_, AllegroHandController::Vector q_desired_, AllegroHandController::Vector torque_desired_){
    mutex.lock();
    q_prev = q_prev_;
    q_current = q_current_;
    q_desired = q_desired_;
    torque_desired = torque_desired_;
    mutex.unlock();
}

void AllegroHandController::executeController(AllegroHandController::Vector q_prev_, AllegroHandController::Vector q_current_, AllegroHandController::Vector &torque_, AllegroHandController::Vector q_desired_){
    setValues(q_prev_, q_current_, q_desired_, torque_);
    control();
    // Store the calculated torque
    mutex.lock();
    torque_ = torque;
    mutex.unlock();    
}

// void AllegroHandController::executeController(AllegroHandController::Vector q_prev_, AllegroHandController::Vector q_current_, AllegroHandController::Vector &torque_, AllegroHandController::Vector q_desired_, AllegroHandController::Vector torque_desired_){
//     setValues(q_prev_, q_current_, q_desired_, torque_desired_);
//     control();
//     // Store the calculated torque
//     mutex.lock();
//     torque_ = torque;
//     mutex.unlock();    
// }

void AllegroHandController::getGravityVector(KDL::Vector &gravity_){
    mutex.lock();
    gravity_ = gravity;
    mutex.unlock();
}

void AllegroHandController::setGravityVector(KDL::Vector &gravity_){
    mutex.lock();
    gravity = gravity_;
    mutex.unlock();
}

//! \brief Gets the torque to compensate gravity effects
//! \param gc_ reference variable to store torque value
void AllegroHandController::getGravityComp(Vector &gc_) {
    mutex.lock();
    gc_ = GravityCompensation;
    mutex.unlock();
}

//! \brief ComputeJacobian including virtual links
//! \param joints Joint values to compute jacobian
//! \return resulting jacobian matrix in JacobianKDL
void AllegroHandController::ComputeJacobian(const std::vector<double> &joints, const std::vector<double> &cog_x, const std::vector<double> &cog_y) {
    KDL::Jacobian J(4);
    KDL::JntArray q(4);
    KDL::Vector originJoint;

    std::vector<KDL::Frame> fingerFrames;

    for (size_t nf = 0; nf < n_fingers; ++nf) {

        KDL::Chain virtualChain = finger_chain.at(nf);

        //ToDo add extra segment depending on cog_x and cog_y to figerChain
        if (nf != 3)
            originJoint = KDL::Vector(0.01335, (cog_x[nf] * 0.001) - 0.0057, 0.0408+((cog_y[nf] * -0.001) + 0.0137));
        else
            originJoint = KDL::Vector(0.01335, (cog_x[nf] * 0.001) - 0.0057, 0.0564+((cog_y[nf] * -0.001) + 0.0137));

        // Rotation and translation. z goes outside the sensor
        KDL::Frame contactPointFrame(KDL::Rotation(KDL::Rotation::Quaternion( 0, -0.7071068, 0, 0.7071068)),originJoint);

        KDL::Segment virtualSegment (KDL::Joint(KDL::Joint::None), contactPointFrame);

        virtualChain.addSegment(virtualSegment);

        //Resize adding a new frame.
        fingerFrames.resize(virtualChain.getNrOfSegments());

        // Solver for fingers jacobian
        RealJacobianSolver[nf] = new KDL::ChainJntToJacSolver(virtualChain);

        // Solver for fingers forward kinematics
        FKSolver[nf] = new KDL::ChainFkSolverPos_recursive(virtualChain);

        // fill q
        for (size_t dof = 0; dof < finger_n_dofs; ++dof) {
            q(dof) = joints[nf * finger_n_dofs + dof];
        }

        // computes jacobian from finger base to fingertip center point
        RealJacobianSolver[nf]->JntToJac(q, J);
        mutexKDL.lock();
            RealJacobianKDL[nf] = J.data;
            RealJacobianTransposeKDL[nf] = RealJacobianKDL[nf].transpose();
        mutexKDL.unlock();
        FKSolver[nf]->JntToCart(q, fingerFrames);

        mutexKDL.lock();
            FingersFrames[nf] = fingerFrames;
        mutexKDL.unlock();

    }
}

//! \brief ComputeJacobian including virtual links
//! \param joints Joint values to compute jacobian
//! \return resulting jacobian matrix in JacobianKDL
void AllegroHandController::ComputeJacobian(const std::vector<double> &joints, const std::vector<KDL::Frame> &contactpoints) {
    KDL::Jacobian J(4);
    KDL::JntArray q(4);
    KDL::Vector originJoint;

    std::vector<KDL::Frame> fingerFrames;

    for (size_t nf = 0; nf < n_fingers; ++nf) {

        KDL::Chain virtualChain = finger_chain.at(nf);

        KDL::Segment virtualSegment (KDL::Joint(KDL::Joint::None), contactpoints[nf]);

        virtualChain.addSegment(virtualSegment);

        fingerFrames.resize(virtualChain.getNrOfSegments());

        // Solver for fingers jacobian
        RealJacobianSolver[nf] = new KDL::ChainJntToJacSolver(virtualChain);

        // Solver for fingers forward kinematics
        FKSolver[nf] = new KDL::ChainFkSolverPos_recursive(virtualChain);

        // fill q
        for (size_t dof = 0; dof < finger_n_dofs; ++dof) {
            q(dof) = joints[nf * finger_n_dofs + dof];
        }

        // computes jacobian from finger base to fingertip center point
        RealJacobianSolver[nf]->JntToJac(q, J);
        mutexKDL.lock();
            RealJacobianKDL[nf] = J.data;
            RealJacobianTransposeKDL[nf] = RealJacobianKDL[nf].transpose();
        mutexKDL.unlock();

        FKSolver[nf]->JntToCart(q, fingerFrames);

        mutexKDL.lock();
            FingersFrames[nf] = fingerFrames;
        mutexKDL.unlock();

    }
}

//! \brief ComputeJacobian including virtual links
//! \param joints Joint values to compute jacobian
//! \return resulting jacobian matrix in JacobianKDL
void AllegroHandController::ComputeJacobian(const std::vector<KDL::Frame> &contactpoints, Vector q_current) {
    KDL::Jacobian J(4);
    KDL::JntArray q(4);
    KDL::Vector originJoint;

    std::vector<KDL::Frame> fingerFrames;

    for (size_t nf = 0; nf < n_fingers; ++nf) {

        KDL::Chain virtualChain = finger_chain.at(nf);

        KDL::Segment virtualSegment (KDL::Joint(KDL::Joint::None), contactpoints[nf]);

        virtualChain.addSegment(virtualSegment);

        fingerFrames.resize(virtualChain.getNrOfSegments());

        // Solver for fingers jacobian
        RealJacobianSolver[nf] = new KDL::ChainJntToJacSolver(virtualChain);

        // Solver for fingers forward kinematics
        FKSolver[nf] = new KDL::ChainFkSolverPos_recursive(virtualChain);

        // fill q
        for (size_t dof = 0; dof < finger_n_dofs; ++dof) {
            q(dof) = q_current(nf * finger_n_dofs + dof);
        }

        // computes jacobian from finger base to fingertip center point
        RealJacobianSolver[nf]->JntToJac(q, J);
        mutexKDL.lock();
            RealJacobianKDL[nf] = J.data;
            RealJacobianTransposeKDL[nf] = RealJacobianKDL[nf].transpose();
        mutexKDL.unlock();
        FKSolver[nf]->JntToCart(q, fingerFrames);

        mutexKDL.lock();
            FingersFrames[nf] = fingerFrames;
        mutexKDL.unlock();

    }
}

//! \brief ComputeJacobian including virtual links
//! \param cog_x, cog_y comes from the values of the weiss sensor.
//! \param joints Joint values to compute jacobian
//! \return resulting jacobian matrix in JacobianKDL

void AllegroHandController::ComputeFingerFrames(const std::vector<double> &joints, const std::vector<double> &cog_x, const std::vector<double> &cog_y) {
    KDL::JntArray q(4);
    KDL::Vector originJoint;

    std::vector<KDL::Frame> fingerFrames;

    for (size_t nf = 0; nf < n_fingers; ++nf) {

        KDL::Chain virtualChain = finger_chain.at(nf);

        //ToDo add extra segment depending on cog_x and cog_y to figerChain
        if (nf != 3)
            originJoint = KDL::Vector(0.01335, (cog_x[nf] * 0.001) - 0.0057, 0.0408+((cog_y[nf] * -0.001) + 0.0137));
        else
            originJoint = KDL::Vector(0.01335, (cog_x[nf] * 0.001) - 0.0057, 0.0564+((cog_y[nf] * -0.001) + 0.0137));

        // Rotation and translation. z goes outside the sensor
        KDL::Frame contactPointFrame(KDL::Rotation(KDL::Rotation::Quaternion( 0, -0.7071068, 0, 0.7071068)),originJoint);

        KDL::Segment virtualSegment (KDL::Joint(KDL::Joint::None), contactPointFrame);

        virtualChain.addSegment(virtualSegment);

        fingerFrames.resize(virtualChain.getNrOfSegments());

        // Solver for fingers forward kinematics
        FKSolver[nf] = new KDL::ChainFkSolverPos_recursive(virtualChain);

        // fill q
        for (size_t dof = 0; dof < finger_n_dofs; ++dof) {
            q(dof) = joints[nf * finger_n_dofs + dof];
        }

        FKSolver[nf]->JntToCart(q, fingerFrames);

        mutex.lock();
            FingersFrames[nf] = fingerFrames;
        mutex.unlock();

    }
}

bool AllegroHandController::setAccelerationTwist(const KDL::Twist &ag_) {
    for (unsigned int i = 0; i < 6; ++i) {
        if (!isfinite(ag_(i))) return false;
    }
    mutex.lock();
    ag = ag_;
    mutex.unlock();

    return true;
}

bool AllegroHandController::setFingertipExternalWrench(
        const std::array<KDL::Wrench, n_fingers> &w_) {
    for (auto wrench : w_) {
        for (unsigned int i = 0; i < 6; ++i) {
            if (!isfinite(wrench(i))) return false;
        }
    }
    mutex.lock();
    w = w_;
    mutex.unlock();

    return true;
}

void AllegroHandController::computeGravityCompensationTorque(Vector q, Vector &gc){

    // Variables to compute gravity compensation torque
    KDL::JntArray gtorque(finger_n_dofs);

    std::vector<KDL::JntArray> qKDL(4);
    std::vector<KDL::JntArray> qdotKDL(4);
    std::vector<KDL::JntArray> qddotKDL(4);
    KDL::Wrenches wKDL(5);


    //ToDo: external set Twist to inlcude inertial effects
    acctwist = -KDL::Twist(gravity, KDL::Vector::Zero());
    setAccelerationTwist(acctwist);

    std::array<KDL::Wrench, finger_n_dofs + 1> w_ext;
    std::array<KDL::Wrench, n_fingers> w_;

    for (size_t i = 0; i < n_fingers; ++i) {
        qKDL[i].resize(4);
        qdotKDL[i].resize(4);
        qddotKDL[i].resize(4);
    }

    // Compute torque to compesate gravity effects
    for (size_t i = 0; i < n_fingers; ++i) {
        KDL::ChainIdSolver_RNE solverID(chain.at(i), gravity);

        // Compute torque
        qKDL[i].data = q.segment<finger_n_dofs>(finger_n_dofs * i);
        wKDL.back() = w_.at(i);

        solverID.CartToJnt(qKDL[i], qdotKDL[i], qddotKDL[i], wKDL, gtorque);

        gc.segment<finger_n_dofs>(finger_n_dofs * i) = gtorque.data;
    }

    // Bound Gravity compensation torque
    for (unsigned int i = 0; i < n_fingers * finger_n_dofs; ++i) {
        if (gc(i) > torque_max[i]) {
            gc(i) = torque_max[i];

        } else if (gc(i) < -torque_max[i]) {
            gc(i) = -torque_max[i];
        }
    }

}

// Accessing to the KDL Chain
    // Returns a KDL Chain copy
    // with finger you choose which finger do you want to obtain
    // values 0 ... 3
    // virtual, you get the KDL::Chain with virtual links (mass = 0)
    // is false, you get the real chain without virtual links
KDL::Chain AllegroHandController::getKDLChain(size_t finger, bool virt)
{
    KDL::Chain _Chain;
    if(finger > n_fingers){
        std::cerr << "Wrong number of finger" << std::endl;
    }
    if(virt){
        _Chain = finger_chain.at(finger);
    }
    else{
        _Chain = chain.at(finger);
    }
    return _Chain;
}


void AllegroHandController::getRealJacobianKDL(std::array<Eigen::MatrixXd,n_fingers> &realJacobianKDL){
    mutexKDL.lock();
        realJacobianKDL = RealJacobianKDL;
    mutexKDL.unlock();
}

void AllegroHandController::getRealJacobianKDLAtIndex(int index, Eigen::MatrixXd &realJacobianKDL_) {
    if (index >= 0 && index < n_fingers) {
        mutexKDL.lock();
            realJacobianKDL_ = RealJacobianKDL[index];
        mutexKDL.unlock();
    } else {
        // Handle invalid index.
        std::cout << "Invalid index of the RealJacobianKDL" << std::endl;
        realJacobianKDL_ = Eigen::MatrixXd::Zero(4, 4);
    }
}