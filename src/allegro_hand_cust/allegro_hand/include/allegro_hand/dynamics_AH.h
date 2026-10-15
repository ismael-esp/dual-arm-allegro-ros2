#ifndef DYNAMICS_AH_H_
#define DYNAMICS_AH_H_

#include <Eigen/Dense>

#include <kdl/chain.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

//! Recursive Newton-Euler inverse dynamics solver
template <size_t N>
class ChainIdSolver_RNE {
public:
    ChainIdSolver_RNE(const KDL::Chain &chain_): chain(chain_), nj(chain.getNrOfJoints()), ns(chain.getNrOfSegments()) {
        if (nj != N) throw std::invalid_argument("Invalid chain.");

        //std::cout << "Number of joints: " << nj << std::endl;
        //std::cout << "Number of segments: " << ns << std::endl;
    }

    ~ChainIdSolver_RNE(){}

    //! \brief CartToJnt
    //! \param q current joint positions
    //! \param q_dot current joint velocities
    //! \param q_ddot current joint accelerations
    //! \param ag acceleration twist of the root (most likely contains gravity)
    //! \param w_ext external wrecnhes (no gravity) on the segments
    //! \return resulting joint torques
    Eigen::Matrix<double, N, 1> CartToJnt(
            const Eigen::Matrix<double, N, 1> &q,
            const Eigen::Matrix<double, N, 1> &qdot,
            const Eigen::Matrix<double, N, 1> &qddot, const KDL::Twist &ag,
            const std::array<KDL::Wrench, N + 1> &w_ext){
        // Sweep from root to leaf
        //std::cout << "Sweep from root to leaf" << std::endl;
        unsigned int j = 0;
        KDL::Twist vj;
        for (unsigned int i = 0; i < ns; ++i) {
            double q_, qdot_, qddot_;
            if (chain.getSegment(i).getJoint().getType() != KDL::Joint::None) {
                q_ = q(j);
                qdot_ = qdot(j);
                qddot_ = qddot(j);
                j++;
            } else {
                q_ = qdot_ = qddot_ = 0;
            }

            // Calculate segment properties: X, S, vj
            X.at(i) = chain.getSegment(i).pose(q_);
            S.at(i) = X.at(i).M.Inverse(chain.getSegment(i).twist(q_, 1.0));
            vj = X.at(i).M.Inverse(chain.getSegment(i).twist(q_, qdot_));

            // Calculate velocity and acceleration of the segment (in segment coordinates)
            if (i == 0) {
                v.at(i) = vj;
                a.at(i) = X.at(i).Inverse(ag) + S.at(i) * qddot_ + v.at(i) * vj;
            } else {
                v.at(i) = X.at(i).Inverse(v[i - 1]) + vj;
                a.at(i) = X.at(i).Inverse(a[i - 1]) + S.at(i) * qddot_ + v.at(i) * vj;
            }

            // Collect RigidBodyInertia and external forces
            const KDL::RigidBodyInertia &Ii = chain.getSegment(i).getInertia();
            w.at(i) = Ii * a.at(i) + v.at(i) * (Ii * v.at(i)) - w_ext.at(i);
        }

        // Sweep from leaf to root
        //std::cout << "Sweep from leaf to root" << std::endl;
        j = nj - 1;
        Eigen::Matrix<double, N, 1> torque;
        for (int i = ns - 1; i >= 0; --i) {
            if (chain.getSegment(i).getJoint().getType() != KDL::Joint::None)
                torque(j--) = dot(S.at(i), w.at(i));
            if (i != 0) w[i - 1] = w[i - 1] + X.at(i) * w.at(i);
        }

        return torque;
    }

protected:
    const KDL::Chain chain;

    const unsigned int nj;

    const unsigned int ns;

    std::array<KDL::Frame, N + 1> X;

    std::array<KDL::Twist, N + 1> S;

    std::array<KDL::Twist, N + 1> v;

    std::array<KDL::Twist, N + 1> a;

    std::array<KDL::Wrench, N + 1> w;
};

#endif // DYNAMICS_AH_H_
