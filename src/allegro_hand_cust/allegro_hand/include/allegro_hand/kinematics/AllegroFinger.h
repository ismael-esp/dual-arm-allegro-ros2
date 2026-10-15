#ifndef ALLEGROFINGER_H_
#define ALLEGROFINGER_H_

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <vector>


// Rotation part of the transformation from the world coordinates to the base of each finger
const std::vector< std::vector<double> > eul_angles_w2b_fingers = {   { 1.5708, -0.0873,   -1.5708},        //Eul angles transformation for the index finger. Transformation from base_link to link_0_0 according to tf_echo
                                                                      {       0,      0,       0},        //Eul angles transformation for the middle finger. Transformation from base_link to link_4_0 according to xacro file
                                                                      { -1.5708,   -0.0873,    1.5708}};   //Eul angles transformation for the middle finger. Transformation from base_link to link_8_0 according to xacro file

// Translation part of the transformation from the world coordinates to the base of each finger
const std::vector< std::vector<double> > pos_w2b_fingers = {  {0, -0.0435, -0.001542},                 //Eul angles transformation for the middle finger. Transformation from base_link to link_8_0 according to xacro file
                                                              {0, 0, 0.0007},
                                                              {0, 0.0435, -0.001542}};
class AllegroFinger
{
	public:
        //Constructor of the desired Allegro finger providing parameters
        AllegroFinger(unsigned int finger_selection_, std::vector<double> link_lengths, std::vector<double> q_min_limits_, std::vector<double> q_max_limits_);
        //Constructor of the desired Allegro finger with no parameters
        AllegroFinger(unsigned int finger_selection_);
        //Default constructor with no parameters only the link_lengths
		AllegroFinger(std::vector<double> link_lengths);
        //Default constructor with no parameters
		AllegroFinger();
        // Destructor
        virtual ~AllegroFinger();

        //Convert from eul angles to Tranformation matrix
		Eigen::Affine3d eul2Tmat(std::vector<double>  eul_angles, std::vector<double>  pos);

        //Forward kinematics of a finger
		Eigen::Affine3d Fkine(std::vector<double> joint_angles);
        bool Fkine(std::vector<double> joint_angles, Eigen::Affine3d &fk_matrix);

        //Returns current joint angles
        std::vector<double> getJointAngles();

        //Returns the desired transformation frame
        Eigen::Affine3d getTransformationMatrix(int index);

        //Returns the total transformation frame
        Eigen::Affine3d getTotalTransformationMatrix();

        //Returns the transformation from world to the base of the finger
        Eigen::Affine3d getTF_w2b();

        //Returns the transformation from the TCP of the finger model to the finger tip frame in Rviz
        Eigen::Affine3d getTF_tcp2tip();

        //Returns the index row corresponding to the valid configuration of the found solutions
        int index_valid_conf(Eigen::Matrix<double, 2, 4>  feasibility_mat, bool not_feasible);

        //Inverse kinematics of a finger
        virtual Eigen::Matrix<double, 2, 4> Ikine(std::vector<double> goal_eul_ang, std::vector<double> goal_pos);
        virtual bool Ikine(std::vector<double> goal_eul_ang, std::vector<double> goal_pos, Eigen::Matrix<double, 2, 4> &q_values_ikine);
        //Ikine given a normal and a position
        // virtual bool IkineNormal(std::vector<double> goal_normal, std::vector<double> goal_pos);
        //Returns the vector containing the feasible solutions for primitive motion 1
        std::vector<std::vector<double>> obtainMotion1Sols(std::vector<double> goal_eul_ang, std::vector<double> goal_pos);

        //Iterates different orientation alternatives to find solutions of primitive motion 1
        Eigen::Matrix<double, 2, 4> compute_IK_for_motion1(double theta_start_with, double *delta_orientation, Eigen::Matrix<double, 4, 1> A01_vect_to_goal,  double theta1_sol, double reorientation, int *complete_turn);

        //Returns the vector containing the feasible solutions for primitive motion 2
        std::vector<std::vector<double>> obtainMotion2Sols(std::vector<double> goal_eul_ang, std::vector<double> goal_pos);

        //Iterates different position alternatives to find solutions of primitive motion 2
        Eigen::Matrix<double, 2, 4> compute_IK_for_motion2(Eigen::Affine3d new_T_desired, Eigen::Matrix<double, 4, 1> A01_vect_to_goal_new,  double theta1_sol, bool *point_success);

        // Get Euler ZYZ angles from normal vector
        void getEulerZYZAnglesFromNormal(Eigen::Vector3d normal);
	protected:
        //These vectors will have the links length values of all the fingers of the Allegro hand.
        const double link1_length;
        const double link2_length;
        const double link3_length;
        const double link4_length;
        const double link5_length;
        // Which finger. 0: Index, 1:Middle, 2:Anular
		int finger_selection;
		Eigen::Matrix<double, 1, 6> a;
		Eigen::Matrix<double, 1, 6> d;
		Eigen::Matrix<double, 1, 6> t_offs;
		Eigen::Matrix<double, 1, 6> alpha;
        std::vector<double> joint_angles;
		Eigen::Affine3d A01, A12, A23, A34, A45, A05;
		Eigen::Affine3d T_world_base;
        Eigen::Affine3d T_tcp_fingerTip;
        // Joint limits (in rad)
        std::vector<double> q_max_limits = {0.47, 1.61, 1.71, 1.62};
	    std::vector<double> q_min_limits = {-0.47, -0.2, -0.17, -0.23};
        // std::vector<std::vector<double>> motion1_sols;
        

        //Verify that the found IK solution is feasible given the finger angle ranges of motion
		Eigen::Matrix<double, 2, 4> checkFeasibility(Eigen::Matrix<double, 2, 4>  IK_solutions);

        //Verify that the given vector of joint values is feasible given the joint limits of the finger
        virtual bool checkAnglesFeasibility(std::vector<double>  q_to_check);

               //Computes the IK computations considering goal orientation changes to find a mathematically possible solution
        Eigen::Matrix<double, 2, 4> computeIkineIterations(Eigen::Matrix<double, 3, 1> new_eul_angles_,
                                                           double *delta_orientation_,
                                                           Eigen::Matrix<double, 4, 1> A01_vect_to_goal_,
                                                           double theta1_sol_,
                                                           int from_stage_flag,
                                                           double reorientation_,
                                                           int *turn_complete);

        //Computes the second stage of IK. To make it coincident with the projection of the desired Z axis
        Eigen::Matrix<double, 2, 4> computeIkineStage2(Eigen::Matrix<double, 2, 4> arm_solution_, double *delta_orientation_, Eigen::Matrix<double, 4, 1> A01_vect_to_goal_, double reorientation_, double extra_angle_, Eigen::Affine3d T_desired_, double *theta_testing_, int *stage_IK_solver_, int *turn_complete);

        //Iterates different orientation alternatives in case the joint angles found are not feasible (inside the joint limits of the finger)
        Eigen::Matrix<double, 2, 4> computeIkineStage3(bool config_invalid, double last_theta_value, Eigen::Matrix<double, 4, 1> A01_vect_to_goal_, Eigen::Matrix<double, 2, 4> arm_solution_, double reorientation_, int *stage_IK_solver);

        //Verifies if given point is inside the workspace of the finger
        virtual bool checkPointInsideWs(double px, double py, double pz);

        //Check if the complete set of IK solutions is feasible
		bool verifyAllJointValues(Eigen::Matrix<double, 2, 4> feasibility_vec);

        //Update the A matrices of the finger object
        void updateA_mats(std::vector<double> joint_angles);

        // Get euler ZYZ angles from rotation matrix
        Eigen::Vector3d rotationMatrixToEulerZYZ(const Eigen::Matrix3d& R);
        // Normalize vector
        Eigen::Vector3d normalize(const Eigen::Vector3d& vec);

        // Compute Transformation matrix between frame i and frame i-1 according to the DH parameters
        Eigen::Matrix<double, 4, 4> getTFMatrixFromDHParameters(double theta_i, double alpha_i, double a_i, double d_i);

};
#endif // ALLEGROFINGER_H_
