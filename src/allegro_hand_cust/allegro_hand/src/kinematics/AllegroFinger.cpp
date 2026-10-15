#include <allegro_hand/kinematics/AllegroFinger.h>
#include <allegro_hand/kinematics/operations.h>
#include <iostream>

/**
* Brief: Allegro finger class constructor
* Param finger_selection_: Int value corresponding to one of the four Allegro fingers
* Return: Nothing
*/
AllegroFinger::AllegroFinger()
: 	// Predefined link lengths
	link1_length(0.0164),
	link2_length(0.054),
	link3_length(0.0384),
	link4_length(0.0408),
	link5_length(0.01335)
{
	// std::cout << "Initializing the empty parameter constructor of AllegroFinger " <<std::endl;
	this->finger_selection = -1;
}


//Default constructor with no parameters only the link_lengths
AllegroFinger::AllegroFinger(std::vector<double> link_lengths)
  : //Update the constant values of the link lengths
  	link1_length(link_lengths.size() > 0 ? link_lengths[0] : 0.0164),
	link2_length(link_lengths.size() > 1 ? link_lengths[1] : 0.054),
	link3_length(link_lengths.size() > 2 ? link_lengths[2] : 0.0384),
	link4_length(link_lengths.size() > 3 ? link_lengths[3] : 0.0408),
	link5_length(link_lengths.size() > 4 ? link_lengths[4] : 0.01335){

	// std::cout << std::endl << std::endl <<std::endl << "Link lengths: " << link1_length << ", " << link2_length << ", " << link3_length << ", " << link4_length << ", " << link5_length << ", ";
	// std::cout << "Initializing the empty parameter constructor of AllegroFinger " <<std::endl;
	this->finger_selection = -1;
}


AllegroFinger::AllegroFinger(unsigned int finger_selection_, std::vector<double> link_lengths, std::vector<double> q_min_limits_, std::vector<double> q_max_limits_)
  : //Update the constant values of the link lengths
  	link1_length(link_lengths.size() > 0 ? link_lengths[0] : 0.0164),
	link2_length(link_lengths.size() > 1 ? link_lengths[1] : 0.054),
	link3_length(link_lengths.size() > 2 ? link_lengths[2] : 0.0384),
	link4_length(link_lengths.size() > 3 ? link_lengths[3] : 0.0408),
	link5_length(link_lengths.size() > 4 ? link_lengths[4] : 0.01335),
	finger_selection(finger_selection_)
  {
	// std::cout << "Initializing the one-parameter constructor of AllegroFinger " <<std::endl;
	// Update joint limits (in rad)
	if(q_min_limits_.size() == 4){
		q_min_limits = q_min_limits_;
	}
	if(q_max_limits_.size() == 4){
		q_max_limits = q_max_limits_;
	}
	for (int i=0; i<4; i++){
		std::cout << "i: "<< i << " q_min_limit: " << q_min_limits[i] <<", q_max_limits: " << q_max_limits[i] << " " << std::endl;
	}
	// Initialize DH parameters for the requested Allegro finger
	this->a      << 0.0,             link2_length,      link3_length	 , link4_length		, link5_length	   , 0.0;
	this->d      << link1_length	   , 0.0             , 0.0             , 0.0              , 0.0          , 0.0;
	this->t_offs << 0.0                , -M_PI/2         , 0.0             , 0.0              , M_PI/2       , M_PI/2;
	this->alpha  << -M_PI/2            , 0.0             , 0.0             , 0.0              , 0.0          , M_PI/2;

	Eigen::Affine3d rz = Eigen::Affine3d(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0, 0, 1)));
	Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(1, 0, 0)));
	Eigen::Affine3d translation(Eigen::Translation3d(Eigen::Vector3d(link5_length,0,0)));
	A45 = (rz*translation*rz*rx).matrix();																																				//This transformation corresponds to the final displacement of the finger tip with respect to its 4th frame. This is a static transformation

	this->T_world_base = eul2Tmat(eul_angles_w2b_fingers[finger_selection_], pos_w2b_fingers[finger_selection_]);	//Transfomation from world to the finger base
	this->joint_angles={-1,-1,-1,-1};
	// std::cout << "T_world_base is: " <<std::endl<<T_world_base.matrix()<<std::endl;																																					//Initializating the joint angles of the finger to -1

	Eigen::Affine3d ry = Eigen::Affine3d(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(0, 1, 0)));
	this->T_tcp_fingerTip = ry.matrix();																																					//Transfomation from TCP frame to the finger tip frame of the finger as it is shown in Rviz
	T_tcp_fingerTip = (0.001 < T_tcp_fingerTip.matrix().array().abs()).select(T_tcp_fingerTip.matrix(), 0.0f);											//Rounds up/down to zero
	// std::cout << "T_tcp_fingerTip is: " <<std::endl<<T_tcp_fingerTip.matrix()<<std::endl;

}



/**
* Brief: Allegro finger class constructor
* Param finger_selection_: Int value corresponding to one of the four Allegro fingers
* Return: Nothing
*/
AllegroFinger::AllegroFinger(unsigned int finger_selection_):
 	// Predefined link lengths
	link1_length(0.0164),
	link2_length(0.054),
	link3_length(0.0384),
	link4_length(0.0408),
	link5_length(0.01335)
  {
	// std::cout << "Initializing the one-parameter constructor of AllegroFinger " <<std::endl;
	this->finger_selection = finger_selection_;

	// Initialize DH parameters for the requested Allegro finger
	this->a      << 0.0,             link2_length,      link3_length	 , link4_length		, link5_length	   , 0.0;
	this->d      << link1_length	   , 0.0             , 0.0             , 0.0              , 0.0          , 0.0;
	this->t_offs << 0.0                , -M_PI/2         , 0.0             , 0.0              , M_PI/2       , M_PI/2;
	this->alpha  << -M_PI/2            , 0.0             , 0.0             , 0.0              , 0.0          , M_PI/2;

	Eigen::Affine3d rz = Eigen::Affine3d(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0, 0, 1)));
	Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(1, 0, 0)));
	Eigen::Affine3d translation(Eigen::Translation3d(Eigen::Vector3d(link5_length,0,0)));
	A45 = (rz*translation*rz*rx).matrix();																																				//This transformation corresponds to the final displacement of the finger tip with respect to its 4th frame. This is a static transformation

	this->T_world_base = eul2Tmat(eul_angles_w2b_fingers[finger_selection_], pos_w2b_fingers[finger_selection_]);	//Transfomation from world to the finger base
	this->joint_angles={-1,-1,-1,-1};
	// std::cout << "T_world_base is: " <<std::endl<<T_world_base.matrix()<<std::endl;																																					//Initializating the joint angles of the finger to -1

	Eigen::Affine3d ry = Eigen::Affine3d(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(0, 1, 0)));
	this->T_tcp_fingerTip = ry.matrix();																																					//Transfomation from TCP frame to the finger tip frame of the finger as it is shown in Rviz
	T_tcp_fingerTip = (0.001 < T_tcp_fingerTip.matrix().array().abs()).select(T_tcp_fingerTip.matrix(), 0.0f);											//Rounds up/down to zero
	// std::cout << "T_tcp_fingerTip is: " <<std::endl<<T_tcp_fingerTip.matrix()<<std::endl;

}

//! \brief Destructor
AllegroFinger::~AllegroFinger() {
}
/**
* Brief: Converts from euler angles (ZYZ config) to HT matrix (4x4)
* Param eul_angles: Vector containing the euler angles in ZYZ config
* Param pos: Vector containing a cartesian position in space
* Return: Homogeneous transformation matrix (4x4)
*/
Eigen::Affine3d AllegroFinger::eul2Tmat(std::vector<double> eul_angles, std::vector<double> pos)
{
	double s_1, c_1, s_2, c_2, s_3, c_3;
	Eigen::Matrix3d rotation_mat;
	s_1 = sin(eul_angles[0]);
    c_1 = cos(eul_angles[0]);
    s_2 = sin(eul_angles[1]);
    c_2 = cos(eul_angles[1]);
    s_3 = sin(eul_angles[2]);
    c_3 = cos(eul_angles[2]);
	// Building the rotation matrix from euler angles in the ZYZ configuration. Implementation taken from Matlab documentation
	rotation_mat<< 	(c_1*c_2*c_3 - s_1*s_3), (-c_1*c_2*s_3 - s_1*c_3), (c_1*s_2),
					(s_1*c_2*c_3 + c_1*s_3), (-s_1*c_2*s_3 + c_1*c_3), (s_1*s_2),
					(-s_2*c_3), 			 (s_2*s_3),					(c_2);

	Eigen::Matrix4d T_converted;
	T_converted.setIdentity();   																										// Set to Identity to make bottom row of Matrix 0,0,0,1
	Eigen::Vector3d T(pos[0],pos[1],pos[2]);																				// Building the position column vector of the HT matrix
	T_converted.block<3,3>(0,0) = rotation_mat;
	T_converted.block<3,1>(0,3) = T;
	T_converted = (0.001 < T_converted.array().abs()).select(T_converted, 0.0f);		//Rounds up/down to zero

	Eigen::Affine3d res_mat;
	res_mat.matrix() =  T_converted;
	return res_mat;
}

/**
*Brief: Function to compute Transformation matrix between frame i and frame i-1 accorsing to the DH parameters
 */

Eigen::Matrix<double, 4, 4> AllegroFinger::getTFMatrixFromDHParameters(double theta_i, double alpha_i, double a_i, double d_i){
	// Eigen::Affine3d 
	Eigen::Matrix<double, 4, 4> A;
	A <<cos(theta_i), -sin(theta_i)*cos(alpha_i),   sin(theta_i)*sin(alpha_i), a_i*cos(theta_i),
						sin(theta_i),  cos(theta_i)*cos(alpha_i),  -cos(theta_i)*sin(alpha_i), a_i*sin(theta_i),
							0,                sin(alpha_i),      	 	  	cos(alpha_i),          d_i,
							0,         		        0,                 		    0,             		 1;
	return A;
}


/**
* Brief: Computes the forward kinematics (FK) of the finger
* Param joint_angles: Vector containing the joint angles for which compute the FK
* Return: Homogeneous transformation matrix (4x4)
*/
Eigen::Affine3d AllegroFinger::Fkine(std::vector<double> joint_angles)
{
	std::vector<Eigen::Matrix<double, 4, 4>> T_vect(5);
	bool joint_angles_valid = false;
	joint_angles_valid = checkAnglesFeasibility(joint_angles);
	// std::cout << "The joint_angles are: " <<std::endl;
	// for(int i=0 ; i<joint_angles.size(); i++)
	// {
	// 	std::cout <<joint_angles[i]<< " ";
	// }
	// std::cout << std::endl;

	// std::cout << "joint_angles_valid is: " << joint_angles_valid << std::endl;
	if (joint_angles_valid)
	{
		// Computing the HT matrices for each joint of the finger
		for (int i = 0; i<4; i++)
		{
			T_vect[i] <<cos(joint_angles[i]+t_offs(i)), -sin(joint_angles[i]+t_offs(i))*cos(alpha(i)),   sin(joint_angles[i]+t_offs(i))*sin(alpha(i)), a(i)*cos(joint_angles[i]+t_offs(i)),
						sin(joint_angles[i]+t_offs(i)),  cos(joint_angles[i]+t_offs(i))*cos(alpha(i)),  -cos(joint_angles[i]+t_offs(i))*sin(alpha(i)), a(i)*sin(joint_angles[i]+t_offs(i)),
													0,                                  sin(alpha(i)),                            	  	cos(alpha(i)),                                d(i),
													0,                               		        0,                                  		    0,                             		 1;
		}
		A01 = T_vect[0];
		A12 = T_vect[1];
		A23 = T_vect[2];
		A34 = T_vect[3];

		A05 = A01*A12*A23*A34*A45;
		A05 = (0.001 < A05.matrix().array().abs()).select(A05.matrix(), 0.0f);  			//Rounds up/down to zero
	}
	else
	{
		// If given joint angles are out of the working limits of the finger
		std::cout << "Given joint values are out of the physical joint limits of the finger" << std::endl;
		Eigen::Matrix<double, 4, 4> aux;
		aux = Eigen::Matrix<double, 4, 4>::Zero();
		A05.matrix() = aux;
	}
	return A05;
}

bool AllegroFinger::Fkine(std::vector<double> joint_angles, Eigen::Affine3d &fk_matrix)
{
	bool success = true;
	std::vector<Eigen::Matrix<double, 4, 4>> T_vect(5);
	bool joint_angles_valid = false;
	joint_angles_valid = checkAnglesFeasibility(joint_angles);
	// std::cout << "The joint_angles are: " <<std::endl;
	// for(int i=0 ; i<joint_angles.size(); i++)
	// {
	// 	std::cout <<joint_angles[i]<< " ";
	// }
	// std::cout << std::endl;

	// std::cout << "joint_angles_valid is: " << joint_angles_valid << std::endl;
	if (joint_angles_valid)
	{
		// Computing the HT matrices for each joint of the finger
		for (int i = 0; i<4; i++)
		{
			T_vect[i] <<cos(joint_angles[i]+t_offs(i)), -sin(joint_angles[i]+t_offs(i))*cos(alpha(i)),   sin(joint_angles[i]+t_offs(i))*sin(alpha(i)), a(i)*cos(joint_angles[i]+t_offs(i)),
						sin(joint_angles[i]+t_offs(i)),  cos(joint_angles[i]+t_offs(i))*cos(alpha(i)),  -cos(joint_angles[i]+t_offs(i))*sin(alpha(i)), a(i)*sin(joint_angles[i]+t_offs(i)),
													0,                                  sin(alpha(i)),                            	  	cos(alpha(i)),                                d(i),
													0,                               		        0,                                  		    0,                             		 1;
		}
		A01 = T_vect[0];
		A12 = T_vect[1];
		A23 = T_vect[2];
		A34 = T_vect[3];

		A05 = A01*A12*A23*A34*A45;
		fk_matrix = (0.001 < A05.matrix().array().abs()).select(A05.matrix(), 0.0f);  			//Rounds up/down to zero
	}
	else
	{
		// If given joint angles are out of the working limits of the finger
		std::cout << "Given joint values are out of the physical joint limits of the finger" << std::endl;
		Eigen::Matrix<double, 4, 4> aux;
		aux = Eigen::Matrix<double, 4, 4>::Zero();
		fk_matrix.matrix() = aux;
		success = false;
	}
	return success;
}


Eigen::Matrix<double, 2, 4> AllegroFinger::checkFeasibility(Eigen::Matrix<double, 2, 4>  IK_solutions)
{
    Eigen::Matrix<double, 2, 4> res;
    res <<  0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0;

    for (int i=0; i<IK_solutions.rows(); i++)
    {
        for (int j=0; j<IK_solutions.cols(); j++)
        {
			// std::cout << "IK_solutions(" << i << "," << j << ") = " << IK_solutions(i, j) << "   ->   ";
            if(IK_solutions(i,j)<=q_max_limits[j] && IK_solutions(i,j)>=q_min_limits[j]){
                res(i,j) = 1;
				// std::cout << "Feasible(1)" << std::endl;
			}
            else{
                res(i,j) = -1;
				// std::cout << "Non feasible(-1)" << std::endl;
			}
        }
    }
    double aux;
	// std::cout << "Check non feasibles: " << std::endl;
    for (int i=0; i<IK_solutions.rows(); i++)
    {
        for (int j=0; j<IK_solutions.cols(); j++)
        {
            if(res(i,j)==-1)
            {
				// std::cout << "IK_solutions(" << i << "," << j << ") = " << IK_solutions(i, j) << ": ";
                if(IK_solutions(i,j)<0)
                {
					// std::cout << "ep";
					if(IK_solutions(i,j)<-M_PI)
					{
						// std::cout << ", epp";
						aux = IK_solutions(i,j)+2*M_PI;
						// std::cout << ". aux: " << aux << std::endl;
						if(aux<=q_max_limits[j] && aux>=q_min_limits[j])
							res(i,j) = 1;
						else
							res(i,j) = -1;
					}
                }
				// std::cout << std::endl;
                if(IK_solutions(i,j)>0)
                {
					// std::cout << "ups, ";
					if(IK_solutions(i,j)>2*M_PI)
					{
						// std::cout << "upps";
						aux = IK_solutions(i,j)-2*M_PI;
						// std::cout << ". aux: " << aux << std::endl;
						if(aux<=q_max_limits[j] && aux>=q_min_limits[j])
							res(i,j) = 1;
						else
							res(i,j) = -1;
					}
                }
				// std::cout << std::endl;
            }
        }
    }
    return res;
}



bool AllegroFinger::checkAnglesFeasibility(std::vector<double> q_to_check)
{
	std::vector<double> vect_feasibility;
	bool res;

	int index=0;
	// std::cout << "Values in q_to_check: " << std::endl;
    // for (const auto& value : q_to_check) {
    //     std::cout << value << ", ";
    // }
    // std::cout << std::endl;
	// std::cout << "Check angles feasibility: " << std::endl;
    for (double q:q_to_check)
    {
		// std::cout << "Finger index: " << index << " ";
        if(q <= q_max_limits[index] && q >= q_min_limits[index]){
            vect_feasibility.push_back(1);
			// std::cout << "feasible(1)" << std::endl;
		}
        else{
            vect_feasibility.push_back(-1);
			// std::cout << "Non feasible(-1)" << std::endl;
		}
		index++;
    }
	index = 0;
    double aux;
    for (double q:q_to_check)
    {
        if(vect_feasibility[index] == -1)
        {
            if(q < 0)
            {
				if(q < -M_PI)
				{
					aux = q + 2*M_PI;
					if(aux <= q_max_limits[index] && aux >= q_min_limits[index])
						vect_feasibility[index] = 1;
					else
						vect_feasibility[index] = -1;
				}
            }
			else
			{
				if(q > 2*M_PI)
				{
					aux = q - 2*M_PI;
					if(aux <= q_max_limits[index] && aux >= q_min_limits[index])
						vect_feasibility[index] = 1;
					else
						vect_feasibility[index] = -1;
				}
			}
        }
		index++;
    }

	if ( std::find(vect_feasibility.begin(), vect_feasibility.end(), -1) != vect_feasibility.end() )
			res = false;
	else
			res = true;

    return res;
}

/**
* Brief: Get current finger joint angles
* Return: Vector with current finger joint angles
*/
std::vector<double> AllegroFinger::getJointAngles()
{	
	return this->joint_angles;
}

/**
* Brief: Get the desired homogeneous transformation (HT) matrix of the finger
* Param index: Int value corresponding to one of the four HT matrix of the finger
* Return: Homogeneous transformation matrix (4x4)
*/
Eigen::Affine3d AllegroFinger::getTransformationMatrix(int index)
{
	Eigen::Affine3d res;
	switch (index) {
	  case 0:
	    res=A01;
	    break;
	  case 1:
	    res=A12;
	    break;
	  case 2:
	    res=A23;
	    break;
	  case 3:
	    res=A45;
	    break;
	}
	return res;
}

/**
* Brief: Get the total HT matrix of the finger, transformation from base to the finger tip
* Return: Homogeneous transformation matrix (4x4)
*/
Eigen::Affine3d AllegroFinger::getTotalTransformationMatrix()
{
	return A05;
}

/**
* Brief: Get the static HT matrix that relates the world coordinate frame with the base of the finger frame
* Return: Homogeneous transformation matrix (4x4)
*/
Eigen::Affine3d AllegroFinger::getTF_w2b()
{
	return T_world_base;
}

/**
* Brief: Get the static HT matrix that relates the TCP of the finger with its tip frame, as it is shown in Rviz.
* Return: Homogeneous transformation matrix (4x4)
*/
Eigen::Affine3d AllegroFinger::getTF_tcp2tip()
{
	return T_tcp_fingerTip;
}

/**
* Brief: Computes the inverse kinematics (IK) of the finger
* Param goal_eul_ang: Vector containing the goal orientation expressed in eul angles
* Param goal_pos: Vector containing the goal position
* Return: Joint angles solution matrix (2x4)
*/
Eigen::Matrix<double, 2, 4> AllegroFinger::Ikine(std::vector<double> goal_eul_ang, std::vector<double> goal_pos)
{
	const double reorientation = 0.05;				// Reorientation increment in case current orientation is finger-joint-angles unfeasible
	int stage_IK_solver = 0;									// Indicates stage of the IK solution
	double delta_orientation = 0;							// Angle change in case of finger-joint-angles unfeasibility
	int turn_complete = 0;										// Complete turn to find feasible finger joint angles
	Eigen::Matrix<double, 2, 4> arm_solution_from_stage3;

	// Convert the desired goal pose wrt the frame of the base of the finger
	Eigen::Affine3d T_desired_wrt_base_hand_frame, T_desired_wrt_base_finger, T_desired;
	std::vector<double> goal_pos_wrt_base_finger(3);
	T_desired_wrt_base_hand_frame = eul2Tmat(goal_eul_ang, goal_pos);

	// std::cout << "T_world_base is: " <<std::endl<< T_world_base << std::endl;
	// std::cout << "T_desired_wrt_base_hand_frame is: " <<std::endl<< T_desired_wrt_base_hand_frame << std::endl;
	// std::cout << "T_tcp_fingerTip.inverse() is: " <<std::endl<< T_tcp_fingerTip.inverse() << std::endl;
	// std::cout << "T_world_base.inverse() is: " <<std::endl<< T_world_base.inverse() << std::endl;

	T_desired_wrt_base_finger = T_desired_wrt_base_hand_frame*T_tcp_fingerTip.inverse();
	T_desired_wrt_base_finger = T_world_base.inverse()*T_desired_wrt_base_finger;

	T_desired = T_desired_wrt_base_finger;
	// std::cout << "T_desired is: " << T_desired.matrix() << std::endl;
	goal_pos_wrt_base_finger = {T_desired(0,3), T_desired(1,3), T_desired(2,3)};
	// printVector(goal_pos_wrt_base_finger);

	Eigen::Matrix<double, 3, 1> aux_eul_angles;
	// Conversion to eul angles from HT matrix expressed in frame "aux"
	Eigen::Matrix3d rotation_part = T_desired.matrix().block<3,3>(0,0);
	aux_eul_angles = rotation_part.eulerAngles(2,1,2);
	// std::cout << "************ aux_eul_angles is: " << aux_eul_angles << std::endl;
	// printVector(goal_pos_wrt_base_finger);


	bool point_in_ws = true;
	point_in_ws = checkPointInsideWs(goal_pos_wrt_base_finger[0], goal_pos_wrt_base_finger[1], goal_pos_wrt_base_finger[2]);
	// std::cout << "point_in_ws is: "<<point_in_ws<< std::endl;
	if(point_in_ws==1)
	{
		// Goal HT matrix expressed in frame "aux", which resembles frame 1 of the finger
		Eigen::Matrix4d T_desired_in_aux;
		Eigen::Affine3d rz = Eigen::Affine3d(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(0, 0, 1)));
		Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(1, 0, 0)));
		T_desired_in_aux = T_desired*(rx*rz).matrix();
		// std::cout << "T_desired_in_aux is: " << T_desired_in_aux << std::endl;

		double theta, extra_angle;
		Eigen::Matrix<double, 3, 1> new_eul_angles;

		// Conversion to eul angles from HT matrix expressed in frame "aux"
		Eigen::Matrix3d rotation_part = T_desired_in_aux.block<3,3>(0,0);
		new_eul_angles = rotation_part.eulerAngles(2,1,2);
		// std::cout << "new_eul_angles is: " << new_eul_angles << std::endl;

		theta = new_eul_angles[2];																																// Retrieve the last eul angle value as this is expected to indicate the total roation of the last 3 joints of the finger
		theta = theta - M_PI/2;																																		// Subtraction due to the offset turn in Z of the 5th link of the finger
		// std::cout << "theta is: " << theta << std::endl;

		extra_angle = atan2(link5_length,link4_length);       // Angle from the last joint to the tip of the robot. It exists because of the translation offset of the TCP of the finger
		// std::cout << "extra_angle is: " << extra_angle << std::endl;

		double px, py;
		px = goal_pos_wrt_base_finger[0];
		py = goal_pos_wrt_base_finger[1];
		// std::cout << "px is: " << px << std::endl ;
		// std::cout << "py is: " << py << std::endl ;

		// Considering only one solution of theta1 as valid. The angle that is either positive or negative and in absolute value smaller than pi/2.
		double theta1_I, theta1_II, theta1_sol;
		theta1_I = atan2(py,px);
		theta1_II = atan2(-py,-px);
		if (abs(theta1_I) < 0.001) {
			theta1_I = 0;
			theta1_sol = theta1_I;
		}
		else{
			if (abs(theta1_I) >= 0 && abs(theta1_I) < M_PI/2)
				theta1_sol = theta1_I;
		}

		if (abs(theta1_II) < 0.001){
			theta1_II = 0;
			theta1_sol = theta1_II;
		}
		else{
			if (abs(theta1_II) >= 0 && abs(theta1_II) < M_PI/2)
				theta1_sol = theta1_II;
		}
		// std::cout << "theta1_sol is: " << theta1_sol << std::endl ;

		// Update A01 with the obtained theta1 solution
		A01.matrix() <<  cos(theta1_sol+t_offs(0)), -sin(theta1_sol+t_offs(0))*cos(alpha(0)),   sin(theta1_sol+t_offs(0))*sin(alpha(0)), a(0)*cos(theta1_sol+t_offs(0)),
						sin(theta1_sol+t_offs(0)),  cos(theta1_sol+t_offs(0))*cos(alpha(0)),  -cos(theta1_sol+t_offs(0))*sin(alpha(0)), a(0)*sin(theta1_sol+t_offs(0)),
						0,                                                  	sin(alpha(0)),                           	 cos(alpha(0)),                      d(0),
						0,                                                              	0,                                       	 0,                          1;

		A01 = (0.001 < A01.matrix().array().abs()).select(A01.matrix(), 0.0f);     					//Rounds up/down to zero
		// std::cout << "A01 is: " << A01 << std::endl ;

		// Computing translation vector from frame 1 of the finger to the goal frame
		Eigen::Matrix<double, 4, 1> p_e_testing_4_elements, A01_vect_to_goal;
		p_e_testing_4_elements << goal_pos_wrt_base_finger[0], goal_pos_wrt_base_finger[1], goal_pos_wrt_base_finger[2], 1;
		// std::cout << "p_e_testing_4_elements is: " << p_e_testing_4_elements << std::endl ;
		A01_vect_to_goal = A01.inverse()*p_e_testing_4_elements;
		// std::cout << "A01_vect_to_goal is: " << A01_vect_to_goal << std::endl ;

		// Siciliano IK solving method for 3 DOF robots
		double pwx, pwy, c3, subtraction, s3plus, s3minus, theta3_I, theta3_II, s2_I, s2_II, c2_I, c2_II, theta2_I, theta2_II, theta4_I, theta4_II;
		pwx = A01_vect_to_goal[0] - sqrt(pow(this->a[4], 2)+pow(this->a[3], 2))*cos(theta+extra_angle-M_PI/2 );
		pwy = A01_vect_to_goal[1] - sqrt(pow(this->a[4], 2)+pow(this->a[3], 2))*sin(theta+extra_angle-M_PI/2 );
		// Rounding pwx and pwy to zero if these are very small
		if (abs(pwx) < 0.001)
			pwx = 0;
		if (abs(pwy) < 0.001)
			pwy = 0;
		// std::cout << "pwx is: " << pwx << std::endl ;
		// std::cout << "pwy is: " << pwy << std::endl ;
		c3 = (pow(pwx, 2)+pow(pwy, 2)-pow(a[1], 2)-pow(a[2], 2))/(2*a[1]*a[2]);
		subtraction = 1-pow(c3, 2);
		// Rounding subtraction to zero if very small
        if (abs(subtraction) < 0.0025)	// Changed this value to avoid different results with Matlab
			subtraction = 0;
			// std::cout << "subtraction in first stage is: " << subtraction << std::endl;

        Eigen::Matrix<double, 2, 4> arm_solution, ik_solutions_testing_grad;

		if(subtraction<0){																																	// Can be negative due to mathematically unfeasibility orientations or if desired point is out of the finger reachibility space
			// std::cout << "Subtraction was negative. Similar to unreachable space" << std::endl;
			stage_IK_solver = 1;
			delta_orientation = delta_orientation-reorientation;															// Angle change for a next orientation test
			// std::cout << "Before calling the computeIkineIterations function" << std::endl;
			arm_solution = computeIkineIterations(new_eul_angles, &delta_orientation, A01_vect_to_goal, theta1_sol, stage_IK_solver, reorientation, &turn_complete);	// Compute IK with the new orientation
			// std::cout << "----------------***** arm_solution is: " << std::endl << arm_solution << std::endl;
			ik_solutions_testing_grad = arm_solution*180/M_PI;
			// std::cout << "----------------***** ik_solutions_testing_grad from stage 1 is: " << std::endl << ik_solutions_testing_grad << std::endl;
		}
		else
		{
			s3plus = sqrt(subtraction);
			s3minus = -sqrt(subtraction);
			theta3_I = atan2(s3plus,c3);
			theta3_II = atan2(s3minus,c3);

			s2_I = ((a[1]+a[2]*c3)*pwy - a[2]*s3plus*pwx) / (pow(pwx, 2)+pow(pwy, 2));
			s2_II = ((a[1]+a[2]*c3)*pwy - a[2]*s3minus*pwx) / (pow(pwx, 2)+pow(pwy, 2));
			c2_I = ((a[1]+a[2]*c3)*pwx + a[2]*s3plus*pwy) / (pow(pwx, 2)+pow(pwy, 2));
			c2_II = ((a[1]+a[2]*c3)*pwx + a[2]*s3minus*pwy) / (pow(pwx, 2)+pow(pwy, 2));

			theta2_I = atan2(s2_I,c2_I);
			theta2_II = atan2(s2_II,c2_II);

			theta4_I = theta - theta2_I - theta3_I;
			theta4_II = theta - theta2_II - theta3_II;

			arm_solution <<   theta1_sol, theta2_I+M_PI/2, theta3_I, theta4_I-M_PI/2,			// The addition to theta2 is because the computation of IK is performed assuming the finger starts layed down (horizontal)
												theta1_sol, theta2_II+M_PI/2, theta3_II, theta4_II-M_PI/2;	// Subtraction from theta4 is because of the alpha DH parameter of the 6th ficticious link.
			// std::cout << "arm_solution is: " << std::endl << arm_solution << std::endl;

			ik_solutions_testing_grad = arm_solution*180/M_PI;
			// std::cout << "ik_solutions_testing_grad from stage 1 is: " << std::endl << ik_solutions_testing_grad << std::endl;
		}

		// Check joint angles feasibility of the obtained solutions
		Eigen::Matrix<double, 2, 4> check_vect;
		check_vect = checkFeasibility(arm_solution);
		// std::cout << "check_vect is: " << check_vect<< std::endl;
		// Checks if at least one the obtained solutions is feasible in terms of joint angles limits of the finger
		bool joint_val_not_valid;
		joint_val_not_valid = verifyAllJointValues(check_vect);
		// std::cout << " joint_val_not_valid is: " << joint_val_not_valid<< std::endl;

		// Performs second stage of the IK process. This intends to approach as much as possible to the desired orienation
		// std::cout << "----------FROM HERE IS STAGE 2: " << std::endl;
		Eigen::Matrix<double, 2, 4> arm_solution_from_stage2;
		arm_solution_from_stage2 = computeIkineStage2(arm_solution, &delta_orientation, A01_vect_to_goal, reorientation, extra_angle, T_desired, &theta, &stage_IK_solver, &turn_complete);
		// std::cout << "theta_testing after computing stage2 is: " << theta<< std::endl;
		// Check joint angles feasibility of the obtained solutions
		check_vect = checkFeasibility(arm_solution_from_stage2);
		// std::cout << "check_vect in stage2 is: " << check_vect<< std::endl;
		// Checks if at least one the obtained solutions is feasible in terms of joint angles limits of the finger
		joint_val_not_valid = verifyAllJointValues(check_vect);
		// std::cout << " joint_val_not_valid in stage2 is: " << joint_val_not_valid<< std::endl;

		// Performs third stage of the IK process. Once desired orientation has been reached, it checks the feasiblity of joint angles and correct them if necessary
		// std::cout << "----------FROM HERE IS STAGE 3: " << std::endl;
		arm_solution_from_stage3 = computeIkineStage3(joint_val_not_valid, theta, A01_vect_to_goal, arm_solution_from_stage2,reorientation,&stage_IK_solver);
		// joint_angles = arm_solution_from_stage3[]
		// std::cout << "joint state SOLUTIONS OF STAEGE 3 are: " <<arm_solution_from_stage3<< std::endl;
		// Check joint angles feasibility of the obtained solutions
		check_vect = checkFeasibility(arm_solution_from_stage3);
		// std::cout << "check_vect in stage3 is: " << check_vect<< std::endl;
		// std::cout << "verifying all joint values at the end of stage3 is: " << std::endl;
		// Checks if at least one the obtained solutions is feasible in terms of joint angles limits of the finger
		joint_val_not_valid = verifyAllJointValues(check_vect);
		// std::cout << " joint_val_not_valid in stage3 is: " << joint_val_not_valid<< std::endl;
		Eigen::Matrix<double, 1, 4> aux;
		aux = arm_solution_from_stage3.block<1,4>(index_valid_conf(check_vect, joint_val_not_valid),0);		// Retrieves the IK solution corresponding to the feasible values
		// std::cout << " aux after stage3 is: " << aux<< std::endl;
		// Updates joint angles values of this class to those found in the IK process
		joint_angles[0] = aux(0,0);
		joint_angles[1] = aux(0,1);
		joint_angles[2] = aux(0,2);
		joint_angles[3] = aux(0,3);
		updateA_mats(joint_angles);
	}
	else
	{
		std::cout << "GOAL POINT IS OUT OF THE WORKSPACE OF THE FINGER" << std::endl;
		arm_solution_from_stage3 << 0, 0, 0, 0,
									0, 0, 0, 0;
	}

  return arm_solution_from_stage3;
}

bool AllegroFinger::Ikine(std::vector<double> goal_eul_ang, std::vector<double> goal_pos, Eigen::Matrix<double, 2, 4> &q_values_ikine){
	bool success = true;
	const double reorientation = 0.05;				// Reorientation increment in case current orientation is finger-joint-angles unfeasible
	int stage_IK_solver = 0;									// Indicates stage of the IK solution
	double delta_orientation = 0;							// Angle change in case of finger-joint-angles unfeasibility
	int turn_complete = 0;										// Complete turn to find feasible finger joint angles

	// Convert the desired goal pose wrt the frame of the base of the finger
	Eigen::Affine3d T_desired_wrt_base_hand_frame, T_desired_wrt_base_finger, T_desired;
	std::vector<double> goal_pos_wrt_base_finger(3);
	T_desired_wrt_base_hand_frame = eul2Tmat(goal_eul_ang, goal_pos);

	// std::cout << "T_world_base is: " <<std::endl<< T_world_base << std::endl;
	// std::cout << "T_desired_wrt_base_hand_frame is: " <<std::endl<< T_desired_wrt_base_hand_frame << std::endl;
	// std::cout << "T_tcp_fingerTip.inverse() is: " <<std::endl<< T_tcp_fingerTip.inverse() << std::endl;
	// std::cout << "T_world_base.inverse() is: " <<std::endl<< T_world_base.inverse() << std::endl;

	T_desired_wrt_base_finger = T_desired_wrt_base_hand_frame*T_tcp_fingerTip.inverse();
	T_desired_wrt_base_finger = T_world_base.inverse()*T_desired_wrt_base_finger;

	T_desired = T_desired_wrt_base_finger;
	// std::cout << "T_desired is: " << T_desired.matrix() << std::endl;
	goal_pos_wrt_base_finger = {T_desired(0,3), T_desired(1,3), T_desired(2,3)};
	// printVector(goal_pos_wrt_base_finger);

	Eigen::Matrix<double, 3, 1> aux_eul_angles;
	// Conversion to eul angles from HT matrix expressed in frame "aux"
	Eigen::Matrix3d rotation_part = T_desired.matrix().block<3,3>(0,0);
	aux_eul_angles = rotation_part.eulerAngles(2,1,2);
	// std::cout << "************ aux_eul_angles is: " << aux_eul_angles << std::endl;
	// printVector(goal_pos_wrt_base_finger);


	bool point_in_ws = true;
	point_in_ws = checkPointInsideWs(goal_pos_wrt_base_finger[0], goal_pos_wrt_base_finger[1], goal_pos_wrt_base_finger[2]);
	// std::cout << "point_in_ws is: "<<point_in_ws<< std::endl;
	if(point_in_ws==1)
	{
		// Goal HT matrix expressed in frame "aux", which resembles frame 1 of the finger
		Eigen::Matrix4d T_desired_in_aux;
		Eigen::Affine3d rz = Eigen::Affine3d(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(0, 0, 1)));
		Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(1, 0, 0)));
		T_desired_in_aux = T_desired*(rx*rz).matrix();
		// std::cout << "T_desired_in_aux is: " << T_desired_in_aux << std::endl;

		double theta, extra_angle;
		Eigen::Matrix<double, 3, 1> new_eul_angles;

		// Conversion to eul angles from HT matrix expressed in frame "aux"
		Eigen::Matrix3d rotation_part = T_desired_in_aux.block<3,3>(0,0);
		new_eul_angles = rotation_part.eulerAngles(2,1,2);
		// std::cout << "new_eul_angles is: " << new_eul_angles << std::endl;

		theta = new_eul_angles[2];																																// Retrieve the last eul angle value as this is expected to indicate the total roation of the last 3 joints of the finger
		theta = theta - M_PI/2;																																		// Subtraction due to the offset turn in Z of the 5th link of the finger
		// std::cout << "theta is: " << theta << std::endl;

		extra_angle = atan2(link5_length,link4_length);       // Angle from the last joint to the tip of the robot. It exists because of the translation offset of the TCP of the finger
		// std::cout << "extra_angle is: " << extra_angle << std::endl;

		double px, py;
		px = goal_pos_wrt_base_finger[0];
		py = goal_pos_wrt_base_finger[1];
		// std::cout << "px is: " << px << std::endl ;
		// std::cout << "py is: " << py << std::endl ;

		// Considering only one solution of theta1 as valid. The angle that is either positive or negative and in absolute value smaller than pi/2.
		double theta1_I, theta1_II, theta1_sol;
		theta1_I = atan2(py,px);
		theta1_II = atan2(-py,-px);
		if (abs(theta1_I) < 0.001) {
			theta1_I = 0;
			theta1_sol = theta1_I;
		}
		else{
			if (abs(theta1_I) >= 0 && abs(theta1_I) < M_PI/2)
				theta1_sol = theta1_I;
		}

		if (abs(theta1_II) < 0.001){
			theta1_II = 0;
			theta1_sol = theta1_II;
		}
		else{
			if (abs(theta1_II) >= 0 && abs(theta1_II) < M_PI/2)
				theta1_sol = theta1_II;
		}
		// std::cout << "theta1_sol is: " << theta1_sol << std::endl ;

		// Update A01 with the obtained theta1 solution
		A01.matrix() <<  cos(theta1_sol+t_offs(0)), -sin(theta1_sol+t_offs(0))*cos(alpha(0)),   sin(theta1_sol+t_offs(0))*sin(alpha(0)), a(0)*cos(theta1_sol+t_offs(0)),
						sin(theta1_sol+t_offs(0)),  cos(theta1_sol+t_offs(0))*cos(alpha(0)),  -cos(theta1_sol+t_offs(0))*sin(alpha(0)), a(0)*sin(theta1_sol+t_offs(0)),
						0,                                                  	sin(alpha(0)),                           	 cos(alpha(0)),                      d(0),
						0,                                                              	0,                                       	 0,                          1;

		A01 = (0.001 < A01.matrix().array().abs()).select(A01.matrix(), 0.0f);     					//Rounds up/down to zero
		// std::cout << "A01 is: " << A01 << std::endl ;

		// Computing translation vector from frame 1 of the finger to the goal frame
		Eigen::Matrix<double, 4, 1> p_e_testing_4_elements, A01_vect_to_goal;
		p_e_testing_4_elements << goal_pos_wrt_base_finger[0], goal_pos_wrt_base_finger[1], goal_pos_wrt_base_finger[2], 1;
		// std::cout << "p_e_testing_4_elements is: " << p_e_testing_4_elements << std::endl ;
		A01_vect_to_goal = A01.inverse()*p_e_testing_4_elements;
		// std::cout << "A01_vect_to_goal is: " << A01_vect_to_goal << std::endl ;

		// Siciliano IK solving method for 3 DOF robots
		double pwx, pwy, c3, subtraction, s3plus, s3minus, theta3_I, theta3_II, s2_I, s2_II, c2_I, c2_II, theta2_I, theta2_II, theta4_I, theta4_II;
		pwx = A01_vect_to_goal[0] - sqrt(pow(this->a[4], 2)+pow(this->a[3], 2))*cos(theta+extra_angle-M_PI/2 );
		pwy = A01_vect_to_goal[1] - sqrt(pow(this->a[4], 2)+pow(this->a[3], 2))*sin(theta+extra_angle-M_PI/2 );
		// Rounding pwx and pwy to zero if these are very small
		if (abs(pwx) < 0.001)
			pwx = 0;
		if (abs(pwy) < 0.001)
			pwy = 0;
		// std::cout << "pwx is: " << pwx << std::endl ;
		// std::cout << "pwy is: " << pwy << std::endl ;
		c3 = (pow(pwx, 2)+pow(pwy, 2)-pow(a[1], 2)-pow(a[2], 2))/(2*a[1]*a[2]);
		subtraction = 1-pow(c3, 2);
		// Rounding subtraction to zero if very small
        if (abs(subtraction) < 0.0025)	// Changed this value to avoid different results with Matlab
			subtraction = 0;
			// std::cout << "subtraction in first stage is: " << subtraction << std::endl;

        Eigen::Matrix<double, 2, 4> arm_solution, ik_solutions_testing_grad;

		if(subtraction<0){																																	// Can be negative due to mathematically unfeasibility orientations or if desired point is out of the finger reachibility space
			// std::cout << "Subtraction was negative. Similar to unreachable space" << std::endl;
			stage_IK_solver = 1;
			delta_orientation = delta_orientation-reorientation;															// Angle change for a next orientation test
			// std::cout << "Before calling the computeIkineIterations function" << std::endl;
			arm_solution = computeIkineIterations(new_eul_angles, &delta_orientation, A01_vect_to_goal, theta1_sol, stage_IK_solver, reorientation, &turn_complete);	// Compute IK with the new orientation
			// std::cout << "----------------***** arm_solution is: " << std::endl << arm_solution << std::endl;
			ik_solutions_testing_grad = arm_solution*180/M_PI;
			// std::cout << "----------------***** ik_solutions_testing_grad from stage 1 is: " << std::endl << ik_solutions_testing_grad << std::endl;
		}
		else
		{
			s3plus = sqrt(subtraction);
			s3minus = -sqrt(subtraction);
			theta3_I = atan2(s3plus,c3);
			theta3_II = atan2(s3minus,c3);

			s2_I = ((a[1]+a[2]*c3)*pwy - a[2]*s3plus*pwx) / (pow(pwx, 2)+pow(pwy, 2));
			s2_II = ((a[1]+a[2]*c3)*pwy - a[2]*s3minus*pwx) / (pow(pwx, 2)+pow(pwy, 2));
			c2_I = ((a[1]+a[2]*c3)*pwx + a[2]*s3plus*pwy) / (pow(pwx, 2)+pow(pwy, 2));
			c2_II = ((a[1]+a[2]*c3)*pwx + a[2]*s3minus*pwy) / (pow(pwx, 2)+pow(pwy, 2));

			theta2_I = atan2(s2_I,c2_I);
			theta2_II = atan2(s2_II,c2_II);

			theta4_I = theta - theta2_I - theta3_I;
			theta4_II = theta - theta2_II - theta3_II;

			arm_solution <<   theta1_sol, theta2_I+M_PI/2, theta3_I, theta4_I-M_PI/2,			// The addition to theta2 is because the computation of IK is performed assuming the finger starts layed down (horizontal)
												theta1_sol, theta2_II+M_PI/2, theta3_II, theta4_II-M_PI/2;	// Subtraction from theta4 is because of the alpha DH parameter of the 6th ficticious link.
			// std::cout << "arm_solution is: " << std::endl << arm_solution << std::endl;

			ik_solutions_testing_grad = arm_solution*180/M_PI;
			// std::cout << "ik_solutions_testing_grad from stage 1 is: " << std::endl << ik_solutions_testing_grad << std::endl;
		}

		// Check joint angles feasibility of the obtained solutions
		Eigen::Matrix<double, 2, 4> check_vect;
		check_vect = checkFeasibility(arm_solution);
		// std::cout << "check_vect is: " << check_vect<< std::endl;
		// Checks if at least one the obtained solutions is feasible in terms of joint angles limits of the finger
		bool joint_val_not_valid;
		joint_val_not_valid = verifyAllJointValues(check_vect);
		// std::cout << " joint_val_not_valid is: " << joint_val_not_valid<< std::endl;

		// Performs second stage of the IK process. This intends to approach as much as possible to the desired orienation
		// std::cout << "----------FROM HERE IS STAGE 2: " << std::endl;
		Eigen::Matrix<double, 2, 4> arm_solution_from_stage2;
		arm_solution_from_stage2 = computeIkineStage2(arm_solution, &delta_orientation, A01_vect_to_goal, reorientation, extra_angle, T_desired, &theta, &stage_IK_solver, &turn_complete);
		// std::cout << "theta_testing after computing stage2 is: " << theta<< std::endl;
		// Check joint angles feasibility of the obtained solutions
		check_vect = checkFeasibility(arm_solution_from_stage2);
		// std::cout << "check_vect in stage2 is: " << check_vect<< std::endl;
		// Checks if at least one the obtained solutions is feasible in terms of joint angles limits of the finger
		joint_val_not_valid = verifyAllJointValues(check_vect);
		// std::cout << " joint_val_not_valid in stage2 is: " << joint_val_not_valid<< std::endl;

		// Performs third stage of the IK process. Once desired orientation has been reached, it checks the feasiblity of joint angles and correct them if necessary
		// std::cout << "----------FROM HERE IS STAGE 3: " << std::endl;
		q_values_ikine = computeIkineStage3(joint_val_not_valid, theta, A01_vect_to_goal, arm_solution_from_stage2,reorientation,&stage_IK_solver);
		// joint_angles = q_values_ikine[]
		// std::cout << "joint state SOLUTIONS OF STAEGE 3 are: " <<q_values_ikine<< std::endl;
		// Check joint angles feasibility of the obtained solutions
		check_vect = checkFeasibility(q_values_ikine);
		// std::cout << "check_vect in stage3 is: " << check_vect<< std::endl;
		// std::cout << "verifying all joint values at the end of stage3 is: " << std::endl;
		// Checks if at least one the obtained solutions is feasible in terms of joint angles limits of the finger
		joint_val_not_valid = verifyAllJointValues(check_vect);
		// std::cout << " joint_val_not_valid in stage3 is: " << joint_val_not_valid<< std::endl;
		Eigen::Matrix<double, 1, 4> aux;
		aux = q_values_ikine.block<1,4>(index_valid_conf(check_vect, joint_val_not_valid),0);		// Retrieves the IK solution corresponding to the feasible values
		// std::cout << " aux after stage3 is: " << aux<< std::endl;
		// Updates joint angles values of this class to those found in the IK process
		joint_angles[0] = aux(0,0);
		joint_angles[1] = aux(0,1);
		joint_angles[2] = aux(0,2);
		joint_angles[3] = aux(0,3);
		updateA_mats(joint_angles);
	}
	else
	{
		std::cout << "GOAL POINT IS OUT OF THE WORKSPACE OF THE FINGER" << std::endl;
		success = false;
	}
	return success;
}

// //! \brief Compute Ikine based on a point and a normal
// //! \param goal_normal. Normal direction of the finger tip
// //! \param goal_pos. X,Y,Z point coordinates
// //! \param q_values_ikine. Resulting q values
// bool AllegroFinger::IkineNormal(std::vector<double> goal_normal, std::vector<double> goal_pos){
//  // WIP
// 	return true;
// }



/**
* Brief: Computes the IK considering goal orientation changes to find a mathematically possible solution
* Param new_eul_angles_: Vector containing the goal orientation expressed in eul angles
* Param *delta_orientation_: Pointer to delta orientation. Total angle increment for testing alternative orientation
* Param A01_vect_to_goal_: Translation vector from frame 1 of the finger to the goal frame
* Param theta1_sol_: IK solution for the first joint
* Param from_stage_flag: Indicates stage of the IK process we are currently in
* Param reorientation_: Fixed incremental value for alternative orientations
* Param *turn_complete: Flag to indicate a complete turn of alternative orientations is done
* Return: Joint angles solution matrix (2x4)
*/
Eigen::Matrix<double, 2, 4> AllegroFinger::computeIkineIterations(Eigen::Matrix<double, 3, 1> new_eul_angles_, double *delta_orientation_, Eigen::Matrix<double, 4, 1> A01_vect_to_goal_, double theta1_sol_, int from_stage_flag, double reorientation_, int *turn_complete)
{
	*turn_complete=0;
	if(abs(*delta_orientation_)>=2*M_PI)		// If the accumulated angle increment is greater or equal than 2*pi
		*turn_complete = 1;

	Eigen::Matrix<double, 2, 4> IK_values;
	double theta_testing, delta_orientation_aux, extra_angle;
	double dist2, dist3, dist4, dist5;
	dist5 = link5_length;
	dist4 = link4_length;
	dist3 = link3_length;
	dist2 = link2_length;

	theta_testing = new_eul_angles_[2];																																			// Retrieve the last eul angle value as this is expected to indicate the total roation of the last 3 joints of the finger
	theta_testing = theta_testing-M_PI/2;																																		// Subtraction due to the offset turn in Z of the 5th link of the finger
	// std::cout << "theta_testing inside the IkineIterations is: " << theta_testing<< std::endl;

	if(theta_testing != 0){
		if (from_stage_flag==3 && *turn_complete==1)																													// If this function has been called from the stage 3 and if turn is complete
		{
			if(theta_testing<0 && *delta_orientation_>0 ){
				delta_orientation_aux = ((theta_testing > 0) - (theta_testing < 0))*abs(*delta_orientation_);			// Make the accumulated angle increment have the same sign as the theta value, that is, we will add to it
			}																												// If theta value is negative and the accumulated increment is positive
			else{
				delta_orientation_aux = -1*((theta_testing > 0) - (theta_testing < 0))*abs(*delta_orientation_);	// Make the accumulated angle increment have a different sign from the theta value, that is, we will subtract from it
			}
		}
		else{
			if(theta_testing<0 && *delta_orientation_>0 ){
					delta_orientation_aux = ((theta_testing > 0) - (theta_testing < 0))*abs(*delta_orientation_);
			}
			else{
					delta_orientation_aux = -1*((theta_testing > 0) - (theta_testing < 0))*abs(*delta_orientation_);
			}
		}
	}
	else{
		delta_orientation_aux = 0;
	}

	extra_angle = atan2(dist5, dist4);
	// std::cout << "delta_orientation_aux, inside the IkineIterations is: " << delta_orientation_aux<< std::endl;
	theta_testing = theta_testing + delta_orientation_aux;
	// std::cout << "theta_testing after sum, inside the IkineIterations is: " << theta_testing<< std::endl;
	double theta, pwx, pwy, c3, subtraction, s3plus, s3minus, theta3_I, theta3_II, s2_I, s2_II, c2_I, c2_II, theta2_I, theta2_II, theta4_I, theta4_II;
	theta = theta_testing;
	pwx = A01_vect_to_goal_[0] - sqrt(pow(dist5, 2)+pow(dist4, 2))*cos(theta+extra_angle-M_PI/2);
	pwy = A01_vect_to_goal_[1] - sqrt(pow(dist5, 2)+pow(dist4, 2))*sin(theta+extra_angle-M_PI/2);

	// Rounding pwx and pwy to zero if these are very small
	if (abs(pwx) < 0.001)
			pwx = 0;
	if (abs(pwy) < 0.001)
			pwy = 0;

	c3 = (pow(pwx, 2)+pow(pwy, 2)-pow(dist2, 2)-pow(dist3, 2))/(2*dist2*dist3);
	subtraction = 1-pow(c3, 2);
	// Rounding subtraction to zero if very small
	if (abs(subtraction) < 0.0025)
			subtraction = 0;
	// std::cout << "In IkineIterations, subtraction is: " << subtraction << std::endl ;

	if(subtraction<0)
	{
		if (from_stage_flag==1)
		{
			*delta_orientation_ = *delta_orientation_-reorientation_;
		}
		if (from_stage_flag==2)
		{
			*delta_orientation_ = *delta_orientation_+reorientation_;
		}
		if (from_stage_flag==3){
			if(*turn_complete == 0){
				*delta_orientation_ = *delta_orientation_-reorientation_;
			}	
			else{
				*delta_orientation_ = *delta_orientation_+reorientation_;
			}
		}
		// std::cout << "About to call the IkineIterations once again "  << std::endl ;
		IK_values = computeIkineIterations(new_eul_angles_, &*delta_orientation_, A01_vect_to_goal_, theta1_sol_, from_stage_flag, reorientation_, &*turn_complete);
	}
	else
	{
		s3plus = sqrt(subtraction);
		s3minus = -sqrt(subtraction);
		theta3_I = atan2(s3plus,c3);
		theta3_II = atan2(s3minus,c3);

		s2_I = ((dist2+dist3*c3)*pwy - dist3*s3plus*pwx) / (pow(pwx, 2)+pow(pwy, 2));
		s2_II = ((dist2+dist3*c3)*pwy - dist3*s3minus*pwx) / (pow(pwx, 2)+pow(pwy, 2));
		c2_I = ((dist2+dist3*c3)*pwx + dist3*s3plus*pwy) / (pow(pwx, 2)+pow(pwy, 2));
		c2_II = ((dist2+dist3*c3)*pwx + dist3*s3minus*pwy) / (pow(pwx, 2)+pow(pwy, 2));

		theta2_I = atan2(s2_I,c2_I);
		theta2_II = atan2(s2_II,c2_II);

		theta4_I = theta - theta2_I - theta3_I;
		theta4_II = theta - theta2_II - theta3_II;

		Eigen::Matrix<double, 2, 4> arm_solution;
		arm_solution << theta1_sol_, theta2_I+M_PI/2, theta3_I, theta4_I-M_PI/2,
										theta1_sol_, theta2_II+M_PI/2, theta3_II, theta4_II-M_PI/2;
		IK_values = arm_solution;
	}
	return IK_values;
}


Eigen::Matrix<double, 2, 4> AllegroFinger::computeIkineStage2(Eigen::Matrix<double, 2, 4> arm_solution_, double *delta_orientation_, Eigen::Matrix<double, 4, 1> A01_vect_to_goal_, double reorientation_, double extra_angle_, Eigen::Affine3d T_desired_, double *theta_testing, int *stage_IK_solver, int *turn_complete)
{
	Eigen::Affine3d new_A05, new_A01;
  // ***start** Check this part of the code. We want to update the value of A01. Look for an improved version of this.
	double q1, q2, q3, q4;

  q1 = arm_solution_(0,0);
  q2 = arm_solution_(0,1);
  q3 = arm_solution_(0,2);
  q4 = arm_solution_(0,3);
	std::vector<double> q_vect{q1, q2, q3, q4};
//   std::cout << "q_vect is: " << std::endl;
//   for(int i =0; i<q_vect.size(); i++)
//   {
//     std::cout << q_vect[i]<<" ";
//   }
//   std::cout << std::endl;
	std::vector<Eigen::Affine3d>  T_vect(5);

  for (int i = 0; i<4; i++)
  {
		T_vect[i].matrix() <<  cos(q_vect[i]+t_offs(i)), -sin(q_vect[i]+t_offs(i))*cos(alpha[i]),   sin(q_vect[i]+t_offs(i))*sin(alpha[i]), a[i]*cos(q_vect[i]+t_offs(i)),
									sin(q_vect[i]+t_offs(i)),  cos(q_vect[i]+t_offs(i))*cos(alpha[i]),  -cos(q_vect[i]+t_offs(i))*sin(alpha[i]), a[i]*sin(q_vect[i]+t_offs(i)),
									0,                                                  sin(alpha[i]),                            cos(alpha[i]),                          d[i],
									0,                                                              0,                                        0,                             1;
  }
	Eigen::Affine3d A01, A12, A23, A34, A05;
  A01 = T_vect[0];
  A12 = T_vect[1];
  A23 = T_vect[2];
  A34 = T_vect[3];
  A05 = A01*A12*A23*A34*this->A45;
  new_A05 = A05;
  // ***end***
//   std::cout << "new_A05 is: " <<std::endl << new_A05 << std::endl;

  Eigen::Affine3d orntn_eeframe_wrt_endpoint, orntn_eeframe_wrt_A0;
  Eigen::Matrix3d rotation_part_T_desired = T_desired_.matrix().block<3,3>(0,0);
//   cout << "rotation_part_T_desired is: " <<endl << rotation_part_T_desired << endl;
  orntn_eeframe_wrt_A0.matrix().block<3,3>(0,0) = rotation_part_T_desired;
//   cout << "orntn_eeframe_wrt_A0 is: " << endl<<orntn_eeframe_wrt_A0 << endl;
  orntn_eeframe_wrt_A0 = (0.001 < orntn_eeframe_wrt_A0.matrix().array().abs()).select(orntn_eeframe_wrt_A0.matrix(), 0.0f);    // Check how this works
//   std::cout << "orntn_eeframe_wrt_A0 is: " << std::endl<<orntn_eeframe_wrt_A0 << std::endl;
	A05 = (0.001 < A05.matrix().array().abs()).select(A05.matrix(), 0.0f);

  orntn_eeframe_wrt_endpoint = new_A05.inverse()*orntn_eeframe_wrt_A0;
  orntn_eeframe_wrt_endpoint = (0.001 < orntn_eeframe_wrt_endpoint.matrix().array().abs()).select(orntn_eeframe_wrt_endpoint.matrix(), 0.0f);    // Check how this works
//   std::cout << "orntn_eeframe_wrt_endpoint is: " << std::endl<<orntn_eeframe_wrt_endpoint << std::endl;

  *theta_testing = arm_solution_(0,1) + arm_solution_(0,2) + arm_solution_(0,3);                    //Check this eul_angles[2,0]

  // theta_testing = theta_testing - pi/2;
//   std::cout << "theta_testing is: " << *theta_testing<< std::endl;
  // if (abs(theta_testing) < 0.001)
  //   theta_testing = 0;


	bool theta_testing_was_neg = 0;
  *delta_orientation_ = atan2(orntn_eeframe_wrt_endpoint(0,2), orntn_eeframe_wrt_endpoint(2,2));
//   std::cout << "*delta_orientation_ is: " << *delta_orientation_<< std::endl;

  if(*theta_testing< 0)
	{
		*theta_testing = *theta_testing+2*M_PI;
		theta_testing_was_neg = 1;
	}
  else
  {
    if(orntn_eeframe_wrt_endpoint(1,3) > 0 && theta_testing_was_neg==0)
      *delta_orientation_ = -*delta_orientation_;
  }

  *theta_testing = *theta_testing + *delta_orientation_;
//   std::cout << "theta_testing_ is: " << *theta_testing<< std::endl;
	double theta;
  theta = *theta_testing;

	double pwx, pwy, c3, subtraction, s3plus, s3minus, theta3_I, theta3_II, s2_I, s2_II, c2_I, c2_II, theta2_I, theta2_II, theta4_I, theta4_II;
	pwx = A01_vect_to_goal_[0] - sqrt(pow(this->a[4], 2)+pow(this->a[3], 2))*cos(theta+extra_angle_-M_PI/2 );
	pwy = A01_vect_to_goal_[1] - sqrt(pow(this->a[4], 2)+pow(this->a[3], 2))*sin(theta+extra_angle_-M_PI/2 );

  // Rounding pwx and pwy to zero if these are very small
  if (abs(pwx) < 0.001)
      pwx = 0;
  if (abs(pwy) < 0.001)
      pwy = 0;
//   std::cout << "pwx is: " << pwx << std::endl ;
//   std::cout << "pwy is: " << pwy << std::endl ;
  c3 = (pow(pwx, 2)+pow(pwy, 2)-pow(a[1], 2)-pow(a[2], 2))/(2*a[1]*a[2]);
  subtraction = 1-pow(c3, 2);
  // Rounding subtraction to zero if very small
  if (abs(subtraction) < 0.0025)
      subtraction = 0;
//   std::cout << "subtraction is: " << subtraction << std::endl;

	Eigen::Matrix<double, 2, 4> new_arm_solution, new_ik_solutions_testing_grad;
  if(subtraction<0){
    // std::cout << "SUbtraction was negative inside the second stage, calling the IK function " << std::endl;
		Eigen::Matrix<double, 3, 1> theta_start_with;
		theta_start_with << 0.0, 0.0, *theta_testing-*delta_orientation_+M_PI/2;
		*delta_orientation_ = *delta_orientation_+reorientation_;
		*stage_IK_solver = 2;
    new_arm_solution = computeIkineIterations(theta_start_with, &*delta_orientation_, A01_vect_to_goal_, arm_solution_(0,0), *stage_IK_solver, reorientation_, turn_complete);

    // std::cout << "----------------***** arm_solution in SECOND stage is: " << std::endl << new_arm_solution << std::endl;
    new_ik_solutions_testing_grad = new_arm_solution*180/M_PI;
    // std::cout << "----------------***** ik_solutions_testing_grad in SECOND stage is: " << std::endl << new_ik_solutions_testing_grad << std::endl;
  }
  else
  {
    s3plus = sqrt(subtraction);
    s3minus = -sqrt(subtraction);
    theta3_I = atan2(s3plus,c3);
    theta3_II = atan2(s3minus,c3);

    s2_I = ((a[1]+a[2]*c3)*pwy - a[2]*s3plus*pwx) / (pow(pwx, 2)+pow(pwy, 2));
    s2_II = ((a[1]+a[2]*c3)*pwy - a[2]*s3minus*pwx) / (pow(pwx, 2)+pow(pwy, 2));
    c2_I = ((a[1]+a[2]*c3)*pwx + a[2]*s3plus*pwy) / (pow(pwx, 2)+pow(pwy, 2));
    c2_II = ((a[1]+a[2]*c3)*pwx + a[2]*s3minus*pwy) / (pow(pwx, 2)+pow(pwy, 2));

    theta2_I = atan2(s2_I,c2_I);
    theta2_II = atan2(s2_II,c2_II);

    theta4_I = theta - theta2_I - theta3_I;
    theta4_II = theta - theta2_II - theta3_II;

    new_arm_solution <<   arm_solution_(0,0), theta2_I+M_PI/2, theta3_I, theta4_I-M_PI/2,
													arm_solution_(0,0), theta2_II+M_PI/2, theta3_II, theta4_II-M_PI/2;

    new_arm_solution = (0.0001 < new_arm_solution.array().abs()).select(new_arm_solution, 0.0f);    // Check how this works
    // std::cout << "new_arm_solution in stage 2 is: " << std::endl << new_arm_solution << std::endl;

    new_ik_solutions_testing_grad = new_arm_solution*180/M_PI;
    // std::cout << "new_ik_solutions_testing_grad in stage 2 is: " << std::endl << new_ik_solutions_testing_grad << std::endl;
  }

	// Eigen::Matrix<double, 2, 4> check_vect;
	// check_vect = checkFeasibility(new_arm_solution);
	// std::cout << "------------- check_vect is: " << check_vect<< std::endl;
	// bool joint_val_not_valid;
	// joint_val_not_valid = verifyAllJointValues(check_vect);
	// std::cout << "--------*-*-*-*----- joint_val_not_valid is: " << joint_val_not_valid<< std::endl;
	return new_arm_solution;
}

Eigen::Matrix<double, 2, 4> AllegroFinger::computeIkineStage3(bool config_invalid, double last_theta_value, Eigen::Matrix<double, 4, 1> A01_vect_to_goal_, Eigen::Matrix<double, 2, 4> arm_solution_, double reorientation_, int *stage_IK_solver)
{
	double delta_orientation;
	int turn_complete;
	Eigen::Matrix<double, 2, 4> new_arm_solution, new_ik_solutions_testing_grad, check_vect;
	if(config_invalid)
	{
		delta_orientation = 0;
		*stage_IK_solver = 3;
    turn_complete = 0;
		Eigen::Matrix<double, 3, 1> theta_start_with;
		while (config_invalid)
		{
			if (turn_complete == 0)
					delta_orientation = delta_orientation -reorientation_;
			else
					delta_orientation = delta_orientation +reorientation_;
			// std::cout << "delta_orientation is: " << delta_orientation << std::endl;
	 		theta_start_with << 0.0, 0.0, last_theta_value+M_PI/2;
			new_arm_solution = computeIkineIterations(theta_start_with, &delta_orientation, A01_vect_to_goal_, arm_solution_(0,0), 1, reorientation_, &turn_complete);
			check_vect = checkFeasibility(new_arm_solution);
			config_invalid = verifyAllJointValues(check_vect);
		}
		new_ik_solutions_testing_grad = new_arm_solution*180/M_PI;
		// std::cout << "----------------***** FINALLY, IK solutions in rad are: " << std::endl << new_arm_solution << std::endl;
		// std::cout << "----------------***** FINALLY, IK solutions in grad are: " << std::endl << new_ik_solutions_testing_grad << std::endl;
		// std::cout << "----------------***** FINALLY, the check vector is: " << std::endl << check_vect << std::endl;
		return new_arm_solution;
	}
	else
	{
		check_vect = checkFeasibility(arm_solution_);
		// std::cout << "----------------***** FINALLY, IK solutions in rad are: " << std::endl << arm_solution_ << std::endl;
		// std::cout << "----------------***** FINALLY, IK solutions in grad are: " << std::endl << arm_solution_*180/M_PI << std::endl;
		// std::cout << "----------------***** FINALLY, the check vector is: " << std::endl << check_vect << std::endl;
		return arm_solution_;
	}
}



bool AllegroFinger::checkPointInsideWs(double px, double py, double pz)
{
	double eq1 = -1518325.85689361*pow(px, 7) + 122722.676426551*pow(px, 6) + 40973.7684578739*pow(px, 5) -5026.02406095314*pow(px, 4) + 17.5985028583025*pow(px, 3) + 15.0905600760902*pow(px, 2) + 0.0277594154537869*px + 0.00293436840914036;
	double eq2 = 1535515.00991672*pow(px, 7) - 119406.165281822*pow(px, 6) - 43847.7080748202*pow(px, 5) + 5497.36491230401*pow(px, 4) - 47.3354698053248*pow(px, 3) -14.4944949423473*pow(px, 2) - 0.0224844980301482*px - 0.00308159148427317;

	// std::cout << "px is: "<<px<< std::endl;
	// std::cout << "eq1 is: "<<eq1<< std::endl;
	bool XY_feasible = false, XZ_feasible = false;
	bool point_in_ws = 0;
	if(px==0 && py==0 && pz == 0)
	{
		point_in_ws = false;
	}
	else{
		if(px<0)
			XY_feasible = false;
		else
		{
			// std::cout << "(eq1-py) is: "<<eq1-py<< std::endl;
			if( (eq1-py >= 0) && (eq2-py <= 0) )
					XY_feasible = true;
			else
					XY_feasible = false;
		}

		double x_on_XZ_plane = sqrt(pow(px,2)+pow(py,2));
		double eq3 = -7799309.28865717*pow(x_on_XZ_plane,7) + 3476018.91996905*pow(x_on_XZ_plane,6) - 620513.853452737*pow(x_on_XZ_plane,5) + 55808.4375839113*pow(x_on_XZ_plane,4) - 2616.45801852230*pow(x_on_XZ_plane,3) + 54.9619500233725*pow(x_on_XZ_plane,2) - 0.510281883465270*x_on_XZ_plane + 0.151435268565539;
		double eq4 = 21316935.9368227*pow(x_on_XZ_plane,7) - 10022022.2258242*pow(x_on_XZ_plane,6) + 1861568.85786671*pow(x_on_XZ_plane,5) - 171693.905688145*pow(x_on_XZ_plane,4) + 8054.22421125576*pow(x_on_XZ_plane,3) - 165.208468934169*pow(x_on_XZ_plane,2) + 0.258583179480744*x_on_XZ_plane -0.0326717141684319;

		// std::cout << "x_on_XZ_plane is: "<<x_on_XZ_plane<< std::endl;
		// std::cout << "(eq3-pz) is: "<<eq3-pz<< std::endl;
		if((eq3-pz >= 0) && (eq4-pz <= 0) )
				XZ_feasible = true;
		else
				XZ_feasible = false;


		if(XZ_feasible && XY_feasible)
				point_in_ws = true;
		else
				point_in_ws = false;

	}
	return point_in_ws;
}

bool AllegroFinger::verifyAllJointValues(Eigen::Matrix<double, 2, 4> feasibility_vec)
{
    bool res=0, aux=0;
    Eigen::Matrix<double, 1, 4> first_row, second_row;
    first_row = feasibility_vec.block<1,4>(0,0);
    second_row = feasibility_vec.block<1,4>(1,0);
    // std::cout << "first_row is: " << first_row << std::endl;
    // std::cout << "second_row is: " << second_row << std::endl;

    for(int i =0; i<4; i++)
    {
        if(first_row(0,i)==-1)
            res = 1;

    }
    for(int i=0; i<4; i++)
    {
        if(second_row(0,i)==-1)
            aux = 1;
		// std::cout << "res: " << res << ", aux: " << aux << std::endl;
    }
    if(aux==0 && res==1)
        res = 0;
    return res;
}

void AllegroFinger::updateA_mats(std::vector<double> joint_angles)
{
	std::vector<Eigen::Matrix<double, 4, 4>> T_vect(5);


	for (int i = 0; i<4; i++)
	{
		T_vect[i] <<cos(joint_angles[i]+t_offs(i)), -sin(joint_angles[i]+t_offs(i))*cos(alpha(i)),   sin(joint_angles[i]+t_offs(i))*sin(alpha(i)), a(i)*cos(joint_angles[i]+t_offs(i)),
								sin(joint_angles[i]+t_offs(i)),  cos(joint_angles[i]+t_offs(i))*cos(alpha(i)),  -cos(joint_angles[i]+t_offs(i))*sin(alpha(i)), a(i)*sin(joint_angles[i]+t_offs(i)),
															0,                                  sin(alpha(i)),                            	  							cos(alpha(i)),                          d(i),
															0,                               					  0,                                  			        									0,                             		 			1;
	}
	A01 = T_vect[0];
	A12 = T_vect[1];
	A23 = T_vect[2];
	A34 = T_vect[3];

	A05 = A01*A12*A23*A34*A45;
	A05 = (0.001 < A05.matrix().array().abs()).select(A05.matrix(), 0.0f);  			//Rounds up/down to zero

}



int AllegroFinger::index_valid_conf(Eigen::Matrix<double, 2, 4>  feasibility_mat, bool not_feasible)
{
	int index;
	Eigen::Matrix<double, 1, 4> first_row, second_row;
	first_row = feasibility_mat.block<1,4>(0,0);
	second_row  = feasibility_mat.block<1,4>(1,0);
	if(not_feasible)
		index = 0;
	else
	{
		if( (first_row.array() != -1).any() )
			index = 0;
		else
			index = 1;

	}
	return index;

}

std::vector<std::vector<double>> AllegroFinger::obtainMotion1Sols(std::vector<double> goal_eul_ang, std::vector<double> goal_pos)
{
	double delta_orientation = 0;
	int complete_turn = 0;
	bool config_invalid;

	std::vector<std::vector<double>> vect_possible_motion;
	Ikine(goal_eul_ang, goal_pos);

	// Convert the desired goal pose wrt the frame of the base of the finger
	Eigen::Affine3d T_desired_wrt_base_hand_frame, T_desired_wrt_base_finger, T_desired;
	std::vector<double> goal_pos_wrt_base_finger(3);
	T_desired_wrt_base_hand_frame = eul2Tmat(goal_eul_ang, goal_pos);

	T_desired_wrt_base_finger = T_desired_wrt_base_hand_frame*T_tcp_fingerTip.inverse();
	T_desired_wrt_base_finger = T_world_base.inverse()*T_desired_wrt_base_finger;

	T_desired = T_desired_wrt_base_finger;
	// std::cout << "T_desired is: " << T_desired << std::endl;
	goal_pos_wrt_base_finger = {T_desired(0,3), T_desired(1,3), T_desired(2,3)};

	vect_possible_motion.push_back(this->joint_angles);

	// std::cout << "vect_possible_motion is: " <<std::endl;
	// for(int i=0 ; i<vect_possible_motion.size(); i++)
	// {
	// 	std::vector<double> vect_aux;
	// 	vect_aux = vect_possible_motion[i];
	// 	for(int j=0 ; j<vect_aux.size(); j++)
	// 	{
	// 		std::cout <<vect_aux[j]<< " ";
	// 	}
	// }
	// std::cout << std::endl;

    double theta_start_with, reorientation= 0.05;
	theta_start_with = joint_angles[1]+joint_angles[2]+joint_angles[3]+M_PI/2;


	// Computing translation vector from frame 1 of the finger to the goal frame
	Eigen::Matrix<double, 4, 1> p_e_testing_4_elements, A01_vect_to_goal;
	p_e_testing_4_elements << goal_pos_wrt_base_finger[0], goal_pos_wrt_base_finger[1], goal_pos_wrt_base_finger[2], 1;
	A01_vect_to_goal = A01.inverse()*p_e_testing_4_elements;
	Eigen::Matrix<double, 2, 4> IK_joint_values, ik_solutions_testing_grad, check_vect;

    while (complete_turn == 0)
	{
		delta_orientation = delta_orientation - reorientation;
		IK_joint_values = compute_IK_for_motion1(theta_start_with, &delta_orientation, A01_vect_to_goal, joint_angles[0], reorientation, &complete_turn);

		check_vect = checkFeasibility(IK_joint_values);
		config_invalid = verifyAllJointValues(check_vect);
		if(config_invalid==0)
		{
			Eigen::Matrix<double, 1, 4> aux;
			aux = IK_joint_values.block<1,4>(index_valid_conf(check_vect, config_invalid),0);
			std::vector<double> joint_vals_inVect= {aux(0,0), aux(0,1), aux(0,2), aux(0,3)};
            vect_possible_motion.push_back(joint_vals_inVect);
		}
	}
	return vect_possible_motion;
}

Eigen::Matrix<double, 2, 4> AllegroFinger::compute_IK_for_motion1(double theta_start_with, double *delta_orientation, Eigen::Matrix<double, 4, 1> A01_vect_to_goal,  double theta1_sol, double reorientation, int *complete_turn)
{
	*complete_turn=0;
	// double new_delta_orientation, 
	double theta_testing, extra_angle;
	double dist2, dist3, dist4, dist5;
	dist5 = link5_length;
	dist4 = link4_length;
	dist3 = link3_length;
	dist2 = link2_length;

	// new_delta_orientation  = *delta_orientation;
	if(abs(*delta_orientation)>=2*M_PI){
		*complete_turn = 1;
	}
    theta_testing = theta_start_with;
    theta_testing = theta_testing -M_PI/2;
    if(theta_testing != 0)
	{
        if (*complete_turn==1){
            *delta_orientation = -1* (*delta_orientation);
		}
	}
	theta_testing = theta_testing + *delta_orientation;
    extra_angle = atan2(dist5, dist4);


 	double pwx, pwy, c3, subtraction, s3plus, s3minus, theta3_I, theta3_II, s2_I, s2_II, c2_I, c2_II, theta2_I, theta2_II, theta4_I, theta4_II;
	pwx = A01_vect_to_goal[0] - sqrt(pow(dist5, 2)+pow(dist4, 2))*cos(theta_testing+extra_angle-M_PI/2);
	pwy = A01_vect_to_goal[1] - sqrt(pow(dist5, 2)+pow(dist4, 2))*sin(theta_testing+extra_angle-M_PI/2);

	// Rounding pwx and pwy to zero if these are very small
	if (abs(pwx) < 0.001)
		pwx = 0;
	if (abs(pwy) < 0.001)
		pwy = 0;

	c3 = (pow(pwx, 2)+pow(pwy, 2)-pow(dist2, 2)-pow(dist3, 2))/(2*dist2*dist3);
	subtraction = 1-pow(c3, 2);
	// Rounding subtraction to zero if very small
	if (abs(subtraction) < 0.0025)
		subtraction = 0;

	Eigen::Matrix<double, 2, 4> IK_values;
	if(subtraction<0)
	{
		if((*delta_orientation > 0) - (*delta_orientation < 0)==-1)
			*delta_orientation = *delta_orientation-reorientation;
        else
            *delta_orientation = *delta_orientation+reorientation;
		IK_values = compute_IK_for_motion1(theta_start_with, &*delta_orientation, A01_vect_to_goal, theta1_sol, reorientation, &*complete_turn);
	}
	else
	{
		s3plus = sqrt(subtraction);
		s3minus = -sqrt(subtraction);
		theta3_I = atan2(s3plus,c3);
		theta3_II = atan2(s3minus,c3);

		s2_I = ((dist2+dist3*c3)*pwy - dist3*s3plus*pwx) / (pow(pwx, 2)+pow(pwy, 2));
		s2_II = ((dist2+dist3*c3)*pwy - dist3*s3minus*pwx) / (pow(pwx, 2)+pow(pwy, 2));
		c2_I = ((dist2+dist3*c3)*pwx + dist3*s3plus*pwy) / (pow(pwx, 2)+pow(pwy, 2));
		c2_II = ((dist2+dist3*c3)*pwx + dist3*s3minus*pwy) / (pow(pwx, 2)+pow(pwy, 2));

		theta2_I = atan2(s2_I,c2_I);
		theta2_II = atan2(s2_II,c2_II);

		theta4_I = theta_testing - theta2_I - theta3_I;
		theta4_II = theta_testing - theta2_II - theta3_II;

		Eigen::Matrix<double, 2, 4> arm_solution;
		arm_solution << theta1_sol, theta2_I+M_PI/2, theta3_I, theta4_I-M_PI/2,
						theta1_sol, theta2_II+M_PI/2, theta3_II, theta4_II-M_PI/2;
		IK_values = arm_solution;
	}
	return IK_values;
}


std::vector<std::vector<double>> AllegroFinger::obtainMotion2Sols(std::vector<double> goal_eul_ang, std::vector<double> goal_pos)
{
	double delta_transl = 0;
	double trans_increment= 0.005;
	bool point_in_ws = true, point_success = true, config_invalid;


	std::vector<std::vector<double>> vect_possible_motion;
	Ikine(goal_eul_ang, goal_pos);

	vect_possible_motion.push_back(this->joint_angles);

	// double theta_start_with;
	// theta_start_with = joint_angles[1]+joint_angles[2]+joint_angles[3]+M_PI/2;


	Eigen::Matrix<double, 2, 4> IK_joint_values, ik_solutions_testing_grad, check_vect;
	Eigen::Affine3d T_current_pos, new_T_desired, T_trans_mat;
	std::vector<double> new_p_desired(3);

	// Computing translation vector from frame 1 of the finger to the goal frame
	Eigen::Matrix<double, 4, 1> new_p_desired_4_elements, A01_vect_to_goal;

	for(int repeat=0 ; repeat<2; repeat++)
	{
		delta_transl = 0;
		while (point_in_ws == true && point_success == true)
		{
			if(repeat == 0)
				delta_transl = delta_transl - trans_increment;
			else
				delta_transl = delta_transl + trans_increment;

			Eigen::Affine3d T_trans(Eigen::Translation3d(Eigen::Vector3d(delta_transl,0,0)));
			T_trans_mat = T_trans.matrix();
			T_current_pos = Fkine(this->joint_angles);
			new_T_desired = T_current_pos*T_trans_mat;
			// std::cout << "new_T_desired is: "<<std::endl<<new_T_desired<<std::endl;
			new_p_desired = {new_T_desired(0,3), new_T_desired(1,3), new_T_desired(2,3)};
			point_in_ws = checkPointInsideWs(new_p_desired[0], new_p_desired[1], new_p_desired[2]);
			// std::cout << "point_in_ws is: "<<point_in_ws<< std::endl;
			if(point_in_ws==0)
			{
				point_in_ws = 1;
				break;
			}

			new_p_desired_4_elements << new_p_desired[0], new_p_desired[1], new_p_desired[2], 1;
			A01_vect_to_goal = A01.inverse()*new_p_desired_4_elements;
			A01_vect_to_goal = (0.001 < A01_vect_to_goal.array().abs()).select(A01_vect_to_goal, 0.0f);     					//Rounds up/down to zero
			// std::cout << "**** A01_vect_to_goal is: "<<A01_vect_to_goal<<std::endl;

			IK_joint_values = compute_IK_for_motion2(new_T_desired, A01_vect_to_goal, joint_angles[0], &point_success);
			if(point_success == false)
			{
				point_success = true;
				break;
			}
			check_vect = checkFeasibility(IK_joint_values);
			config_invalid = verifyAllJointValues(check_vect);
			if(config_invalid==0)
			{
				Eigen::Matrix<double, 1, 4> aux;
				aux = IK_joint_values.block<1,4>(index_valid_conf(check_vect, config_invalid),0);
				std::vector<double> joint_vals_inVect= {aux(0,0), aux(0,1), aux(0,2), aux(0,3)};
	      		vect_possible_motion.push_back(joint_vals_inVect);
			}
		}
	}
	return vect_possible_motion;
}


Eigen::Matrix<double, 2, 4> AllegroFinger::compute_IK_for_motion2(Eigen::Affine3d new_T_desired, Eigen::Matrix<double, 4, 1> A01_vect_to_goal_new,  double theta1_sol, bool *point_success)
{
	double dist2, dist3, dist4, dist5;
	dist5 = link5_length;
	dist4 = link4_length;
	dist3 = link3_length;
	dist2 = link2_length;

	Eigen::Matrix4d T_desired_in_aux;
	Eigen::Affine3d rz = Eigen::Affine3d(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(0, 0, 1)));
	Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(1, 0, 0)));
	T_desired_in_aux = new_T_desired*(rx*rz).matrix();
	// std::cout << "T_desired_in_aux is: " << T_desired_in_aux << std::endl;

	double theta, extra_angle;

	Eigen::Matrix<double, 3, 1> new_eul_angles;

	// Conversion to eul angles from HT matrix expressed in frame "aux"
	Eigen::Matrix3d rotation_part = T_desired_in_aux.block<3,3>(0,0);
	new_eul_angles = rotation_part.eulerAngles(2,1,2);
	// std::cout << "new_eul_angles is: " << new_eul_angles << std::endl;

	theta = new_eul_angles[2];
	// Added this verification because the angles that C++ finds with "eulerAngles" in some cases are different from the ones obtained in Matlab. Are different but the overall rotation matrix remains the same
	if (theta>0)
		theta = theta+ M_PI/2;
	else
		theta = theta- M_PI/2;


	extra_angle = atan2(dist5, dist4);

	double pwx, pwy, c3, subtraction, s3plus, s3minus, theta3_I, theta3_II, s2_I, s2_II, c2_I, c2_II, theta2_I, theta2_II, theta4_I, theta4_II;
	pwx = A01_vect_to_goal_new[0] - sqrt(pow(dist5, 2)+pow(dist4, 2))*cos(theta+extra_angle-M_PI/2);
	pwy = A01_vect_to_goal_new[1] - sqrt(pow(dist5, 2)+pow(dist4, 2))*sin(theta+extra_angle-M_PI/2);
	// std::cout << "pwx is: " << pwx << std::endl;
	// std::cout << "pwy is: " << pwy << std::endl;
	// Rounding pwx and pwy to zero if these are very small
	if (abs(pwx) < 0.001)
		pwx = 0;
	if (abs(pwy) < 0.001)
		pwy = 0;

	c3 = (pow(pwx, 2)+pow(pwy, 2)-pow(dist2, 2)-pow(dist3, 2))/(2*dist2*dist3);
	subtraction = 1-pow(c3, 2);
	// Rounding subtraction to zero if very small
	if (abs(subtraction) < 0.0025)
		subtraction = 0;

	Eigen::Matrix<double, 2, 4> IK_values;
	// std::cout << "subtraction is: " << subtraction << std::endl;
	if(subtraction<0)
	{
		std::cout << "Unable to reach this point" << std::endl;
		*point_success = false;
    IK_values << -1, -1, -1, -1,
 									-1, -1, -1, -1;
	}
	else
	{
		s3plus = sqrt(subtraction);
		s3minus = -sqrt(subtraction);
		theta3_I = atan2(s3plus,c3);
		theta3_II = atan2(s3minus,c3);

		s2_I = ((dist2+dist3*c3)*pwy - dist3*s3plus*pwx) / (pow(pwx, 2)+pow(pwy, 2));
		s2_II = ((dist2+dist3*c3)*pwy - dist3*s3minus*pwx) / (pow(pwx, 2)+pow(pwy, 2));
		c2_I = ((dist2+dist3*c3)*pwx + dist3*s3plus*pwy) / (pow(pwx, 2)+pow(pwy, 2));
		c2_II = ((dist2+dist3*c3)*pwx + dist3*s3minus*pwy) / (pow(pwx, 2)+pow(pwy, 2));

		theta2_I = atan2(s2_I,c2_I);
		theta2_II = atan2(s2_II,c2_II);

		theta4_I = theta - theta2_I - theta3_I;
		theta4_II = theta - theta2_II - theta3_II;


		IK_values << theta1_sol, theta2_I+M_PI/2, theta3_I, theta4_I-M_PI/2,
										theta1_sol, theta2_II+M_PI/2, theta3_II, theta4_II-M_PI/2;
	}
	return IK_values;
}


//! \brief Function to calculate the euler angles from the normal of the object
void AllegroFinger::getEulerZYZAnglesFromNormal(Eigen::Vector3d normal){
	// Normalize the normal vector
	normal = normalize(normal);
	// Define a vector non colineal with the normal
	Eigen::Vector3d u(1.0, 0.0, 0.0);
    if (normal == u) {
        u = Eigen::Vector3d(0.0, 1.0, 0.0);
    }

	// Calculate an othonormal vector to the normal: i = normal X u
	Eigen::Vector3d i = normalize(normal.cross(u));

	// Calculate the third vector, orthonormal to the previous ones. j = normal x i
	Eigen::Vector3d j = normal.cross(i);

	// Generate a rotation matrix. [i, j, normal]
	Eigen::Matrix3d R;
    R.col(0) = i;
    R.col(1) = j;
    R.col(2) = normal;

	// Convert the rotation matrix to Euler angles (ZYZ)
	Eigen::Vector3d eulerAngles = rotationMatrixToEulerZYZ(R);
	std::cout << "Euler angles (ZYZ): " << eulerAngles.transpose() << std::endl;
}

//! \brief Function to normalize a vector
//! \param vec: Vector to normalize
//! \return Normalized vector
Eigen::Vector3d AllegroFinger::normalize(const Eigen::Vector3d& vec) {
    return vec / vec.norm();
}

//! \brief Calculate the euler angles (ZYZ) from a rotation matrix
//! \param R: Rotation matrix
//! \return Euler ZYZ vector
Eigen::Vector3d AllegroFinger::rotationMatrixToEulerZYZ(const Eigen::Matrix3d& R) {
    Eigen::Vector3d eulerAngles;
    if (R(2, 2) < 1) {
        if (R(2, 2) > -1) {
            eulerAngles[0] = atan2(R(1, 2), R(0, 2));
            eulerAngles[1] = acos(R(2, 2));
            eulerAngles[2] = atan2(R(2, 1), -R(2, 0));
        } else { // R(2,2) == -1
            eulerAngles[0] = -atan2(-R(1, 0), R(1, 1));
            eulerAngles[1] = M_PI;
            eulerAngles[2] = 0;
        }
    } else { // R(2,2) == 1
        eulerAngles[0] = atan2(-R(1, 0), R(1, 1));
        eulerAngles[1] = 0;
        eulerAngles[2] = 0;
    }
    return eulerAngles;
}