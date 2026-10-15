#include <allegro_hand/kinematics/ThumbFinger.h>
#include <allegro_hand/kinematics/AllegroFinger.h>
#include <allegro_hand/kinematics/operations.h>
#include <iostream>


// Intitialize thumb with predefined link lengths values.
ThumbFinger::ThumbFinger():AllegroFinger({0.005, 0.0399+0.0177, 0.0514, 0.0564, 0.01335})
{

	// std::cout <<std::endl << "Link lengths: " << link1_length << ", " << link2_length << ", " << link3_length << ", " << link4_length << ", " << link5_length << ", ";
	// super(finger_selection_);
	// std::cout << "Initializing the thumb finger derived class " <<std::endl;
	// Define joint limits:
	q_max_limits = {1.57, 1.16, 1.64, 1.72};
	q_min_limits = {0.26, -0.1, -0.19, -0.16};
	// Initialize DH parameters for the requested Allegro finger
	this->a      << link1_length				   , 0.0					, link3_length					 , link4_length					  , link5_length				   , 0.0;
	this->d      << 0							   , -link2_length          , 0.0                            , 0.0                            , 0.0                            , 0.0;
	this->t_offs << 0.0                            , -M_PI/2                , -M_PI/2                        , 0.0                            , M_PI/2                         , M_PI/2;
	this->alpha  << -M_PI/2                        , M_PI/2                 , 0.0                            , 0.0                            , 0.0                            , M_PI/2;

	Eigen::Affine3d rz = Eigen::Affine3d(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0, 0, 1)));
	Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(1, 0, 0)));
	Eigen::Affine3d translation(Eigen::Translation3d(Eigen::Vector3d(link5_length,0,0)));
	A45 = (rz*translation*rz*rx).matrix();																																				//This transformation corresponds to the final displacement of the finger tip with respect to its 4th frame. This is a static transformation

	this->T_world_base = eul2Tmat(eul_ang_w2thumbBase, pos_w2b_thumbBase);	//Transfomation from world to the thumb finger base
	std::cout << "T thumb world to base: " <<T_world_base.matrix() << std::endl;
	this->joint_angles={-1,-1,-1,-1};																																							//Initializating the joint angles of the finger to -1

	Eigen::Affine3d ry = Eigen::Affine3d(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(0, 1, 0)));
	this->T_tcp_fingerTip = ry.matrix();			
	std::cout << "T thumb T_tcp_fingerTip: " <<T_tcp_fingerTip.matrix() << std::endl;																																//Transfomation from TCP frame to the finger tip frame of the finger as it is shown in Rviz
    T_tcp_fingerTip = (0.001 < T_tcp_fingerTip.matrix().array().abs()).select(T_tcp_fingerTip.matrix(), 0.0);
	std::cout << "T thumb T_tcp_fingerTip: " <<T_tcp_fingerTip.matrix() << std::endl;

}

    
ThumbFinger::ThumbFinger(std::vector<double> link_lengths, std::vector<double> q_min_limits_, std::vector<double> q_max_limits_):AllegroFinger(link_lengths)
{
		
	// Update the joint limits (in rad)
	if(q_min_limits_.size() == 4){
		q_min_limits = q_min_limits_;
	}
	if(q_max_limits_.size() == 4){
		q_max_limits = q_max_limits_;
	}
	for (int i=0; i<4; i++){
		std::cout << "i: "<< i << " q_min_limit: " << q_min_limits[i] <<", q_max_limits: " << q_max_limits[i] << " " << std::endl;
	}
	std::cout <<std::endl << "Thumb Link lengths: " << link1_length << ", " << link2_length << ", " << link3_length << ", " << link4_length << ", " << link5_length << ", ";
	// super(finger_selection_);
	// std::cout << "Initializing the thumb finger derived class " <<std::endl;

	// Initialize DH parameters for the requested Allegro finger
	this->a      << link1_length				   , 0.0					, link3_length					 , link4_length					  , link5_length				   , 0.0;
	this->d      << 0							   , -link2_length          , 0.0                            , 0.0                            , 0.0                            , 0.0;
	this->t_offs << 0.0                            , -M_PI/2                , -M_PI/2                        , 0.0                            , M_PI/2                         , M_PI/2;
	this->alpha  << -M_PI/2                        , M_PI/2                 , 0.0                            , 0.0                            , 0.0                            , M_PI/2;

	Eigen::Affine3d rz = Eigen::Affine3d(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0, 0, 1)));
	Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(1, 0, 0)));
	Eigen::Affine3d translation(Eigen::Translation3d(Eigen::Vector3d(link5_length,0,0)));
	A45 = (rz*translation*rz*rx).matrix();																																				//This transformation corresponds to the final displacement of the finger tip with respect to its 4th frame. This is a static transformation

	this->T_world_base = eul2Tmat(eul_ang_w2thumbBase, pos_w2b_thumbBase);	//Transfomation from world to the thumb finger base
	// std::cout << "T thumb world to base: " <<T_world_base.matrix() << std::endl;
	this->joint_angles={-1,-1,-1,-1};																																							//Initializating the joint angles of the finger to -1

	Eigen::Affine3d ry = Eigen::Affine3d(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(0, 1, 0)));
	this->T_tcp_fingerTip = ry.matrix();																																					//Transfomation from TCP frame to the finger tip frame of the finger as it is shown in Rviz
	// std::cout << "T thumb T_tcp_fingerTip: " <<T_tcp_fingerTip.matrix() << std::endl;																																//Transfomation from TCP frame to the finger tip frame of the finger as it is shown in Rviz
    T_tcp_fingerTip = (0.001 < T_tcp_fingerTip.matrix().array().abs()).select(T_tcp_fingerTip.matrix(), 0.0);
	// std::cout << "T thumb T_tcp_fingerTip: " <<T_tcp_fingerTip.matrix() << std::endl;
}

/**
* Brief: Computes the inverse kinematics (IK) of the finger
* Param goal_eul_ang: Vector containing the goal orientation expressed in eul angles
* Param goal_pos: Vector containing the goal position
* Return: Joint angles solution matrix (2x4)
*/
std::vector<std::vector<double>> ThumbFinger::IkineThumb(std::vector<double> goal_eul_ang, std::vector<double> goal_pos)
{
    //const double reorientation = 0.05;				// Reorientation increment in case current orientation is finger-joint-angles unfeasible
    //int stage_IK_solver = 0;									// Indicates stage of the IK solution
    //double delta_orientation = 0;							// Angle change in case of finger-joint-angles unfeasibility
    //int turn_complete = 0;										// Complete turn to find feasible finger joint angles
	
	std::vector<std::vector<double>> arm_sols_accumulator, final_arm_solutions;
	std::vector<double> final_vect_joint_values;
	std::vector <Eigen::Vector3d> solutions_eq_syst;
	// Convert the desired goal pose wrt the frame of the base of the finger
	Eigen::Affine3d T_desired_wrt_base_hand_frame, T_desired_wrt_base_finger, T_desired;
	std::vector<double> goal_pos_wrt_base_finger(3);
	T_desired_wrt_base_hand_frame = eul2Tmat(goal_eul_ang, goal_pos);

	// std::cout << "T_world_base is: " <<std::endl<< T_world_base << std::endl;
	// std::cout << "T_desired_wrt_base_hand_frame is: " <<std::endl<< T_desired_wrt_base_hand_frame << std::endl;
	// std::cout << "T_world_base.inverse() is: " <<std::endl<< T_world_base.inverse() << std::endl;

	T_desired_wrt_base_finger = T_desired_wrt_base_hand_frame*T_tcp_fingerTip.inverse();
	T_desired_wrt_base_finger = T_world_base.inverse()*T_desired_wrt_base_finger;
	// std::cout << "T_desired_wrt_base_finger is: " <<std::endl<< T_desired_wrt_base_finger << std::endl;

	T_desired = T_desired_wrt_base_finger;
	// std::cout << "T_desired is: " << T_desired << std::endl;
	goal_pos_wrt_base_finger = {T_desired(0,3), T_desired(1,3), T_desired(2,3)};

	bool point_in_ws = true;
	point_in_ws = checkPointInsideWs(goal_pos_wrt_base_finger[0], goal_pos_wrt_base_finger[1], goal_pos_wrt_base_finger[2]);
	// std::cout << "point_in_ws is: "<<point_in_ws<< std::endl;
	if(point_in_ws==1)
	{
		// std::cout << "The T_desired matrix is: " <<std::endl<< T_desired << std::endl;
		// Goal HT matrix expressed in frame "aux", which resembles frame 1 of the finger
		Eigen::Matrix4d T_desired_in_aux, T_desired_in_aux2;
		Eigen::Affine3d translation1(Eigen::Translation3d(Eigen::Vector3d(0,0,-link5_length)));
		T_desired_in_aux = T_desired*(translation1).matrix();

		Eigen::Affine3d translation2(Eigen::Translation3d(Eigen::Vector3d(link4_length,0,0)));
		Eigen::Affine3d rz = Eigen::Affine3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0, 0, 1)));
		Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(1, 0, 0)));
		T_desired_in_aux2=T_desired_in_aux*(translation2*rz*rx).matrix();
		// std::cout << "T_desired_in_aux2 is: " << std::endl<<T_desired_in_aux2 << std::endl;

		double px, py, pz;
		px = T_desired_in_aux2(0,3);
		py = T_desired_in_aux2(1,3);
		pz = T_desired_in_aux2(2,3);

		///*/*/*/*/*/*//*/*/**/*/*/*/*/*/*/ */*/*/*/*/ HERE PUT THE SOLVER CODE
		solutions_eq_syst=systemEquationsSolver(px, py, pz);
		// std::cout << "Solver solutions are:"<< std::endl;
		// for(unsigned int i = 0; i < solutions_eq_syst.size();i++)
		// {
		// 		std::cout << "q1 = " << solutions_eq_syst[i](0) <<", ";
		// 		std::cout << "q2 = " << solutions_eq_syst[i](1) <<", ";
		// 		std::cout << "q3 = " << solutions_eq_syst[i](2) << std::endl;
		// }
		// double solver_value;
		// std::cout << "size of the solutions solver is: " << solutions_eq_syst.size() << std::endl;

		std::vector<std::vector<double>> partial_q_values(solutions_eq_syst.size()), valid_partial_q_vals;
        for (unsigned int i = 0 ; i < solutions_eq_syst.size() ; i++) {
            partial_q_values[i].resize(3);
        }
		for(unsigned int i = 0; i < solutions_eq_syst.size();i++)
		{
			partial_q_values[i][0] = solutions_eq_syst[i](0);
			partial_q_values[i][1] = solutions_eq_syst[i](1);
			partial_q_values[i][2] = solutions_eq_syst[i](2);
		}

		valid_partial_q_vals = considerValidJointValues(partial_q_values, false);
		// std::cout << "valid_partial_q_vals are:"<< std::endl;
		// for(unsigned int i = 0; i < valid_partial_q_vals.size();i++)
		// {
		// 	printVector(valid_partial_q_vals[i]);
		// }

		std::vector<double> q1_sols_to_test;
		for (std::vector<double> vect:valid_partial_q_vals)
		{
			q1_sols_to_test.push_back(vect[0]);
			// printVector(vect);
		}

		// std::cout << "THe q1_sols_to_test are: " << std::endl;
		// printVector(q1_sols_to_test);

		for(double q1_partial_sol:q1_sols_to_test)
		{
			Eigen::Affine3d b_T_f1, b_T_f1_hat, f1_hat_T_goal;		// Transform from base to frame f1, transform from base to frame f1_hat and transform from frame f1_hat to goal frame
			b_T_f1.matrix() << cos(q1_partial_sol+t_offs(0)), -sin(q1_partial_sol+t_offs(0))*cos(alpha(0)),   sin(q1_partial_sol+t_offs(0))*sin(alpha(0)), a(0)*cos(q1_partial_sol+t_offs(0)),
					sin(q1_partial_sol+t_offs(0)),  cos(q1_partial_sol+t_offs(0))*cos(alpha(0)),  -cos(q1_partial_sol+t_offs(0))*sin(alpha(0)), a(0)*sin(q1_partial_sol+t_offs(0)),
												0,                                  sin(alpha(0)),                            	  	cos(alpha(0)),                          d(0),
												0,                               					  0,                                		0,                             		 			1;
			Eigen::Affine3d rotx = Eigen::Affine3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1, 0, 0)));
			b_T_f1_hat = b_T_f1.matrix()*rotx.matrix();
			f1_hat_T_goal = b_T_f1_hat.matrix().inverse()*T_desired.matrix();
			// std::cout << "f1_hat_T_goal is: " << std::endl<< f1_hat_T_goal<< std::endl;

			double X_f1_hat_T_goal, Y_f1_hat_T_goal, Z_f1_hat_T_goal;	// X, Y and Z coordinates of the transformation that relates f1_hat frame to the goal frame
			X_f1_hat_T_goal = f1_hat_T_goal(0,3);
			Y_f1_hat_T_goal = f1_hat_T_goal(1,3);
			Z_f1_hat_T_goal = f1_hat_T_goal(2,3);

			//------------ Here it begins the computation of the last 3 joint angles
			Eigen::Matrix<double, 4, 4> f1_hat_T_auxf4;
			Eigen::Affine3d rotz = Eigen::Affine3d(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(0, 0, 1)));

			rotx = Eigen::Affine3d(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(1, 0, 0)));
			f1_hat_T_auxf4 = f1_hat_T_goal*(rotx*rotz).matrix();		//T matrix from the f1_hat frame to goal point translated to frame 4
			// std::cout << "f1_hat_T_auxf4 is: " << std::endl<< f1_hat_T_auxf4<< std::endl;

			double theta, extra_angle;
			Eigen::Matrix<double, 3, 1> new_eul_angles;

			// Conversion to eul angles from HT matrix
			Eigen::Matrix3d rotation_part = f1_hat_T_auxf4.block<3,3>(0,0);
			new_eul_angles = rotation_part.eulerAngles(2,1,2);
			// std::cout << "new_eul_angles is: " << std::endl<< new_eul_angles<< std::endl;


			theta = new_eul_angles[2];						// Retrieve the last eul angle value as this is expected to indicate the total roation of the last 3 joints of the finger
			// std::cout << "theta at the beginning is: " << std::endl<< theta<< std::endl;
			// Added this verification because the angles that C++ finds with "eulerAngles" in some cases are different from the ones obtained in Matlab. Are different but the overall rotation matrix remains the same. However, this needs further tests, not all cases were cover
			if (theta<0){
				theta = theta+ M_PI;
			}
			else{
				theta = theta- M_PI;
			}
			theta = theta - M_PI/2;							// The pi/2 addition is due to the offset turn in Z that we do for the 5th link DH parameters.
			extra_angle = atan2(link5_length,link4_length);	// Corresponds to the angle between the links 4 and 5
			// std::cout << "theta at the end is: " << std::endl<< theta<< std::endl;

			double theta2_I, theta2_sol, hipot, L1, c4_temp, subtraction;
			theta2_I = atan2(Y_f1_hat_T_goal,X_f1_hat_T_goal);
			if (abs(theta2_I) < 0.001) {
				theta2_I = 0;
				theta2_sol = theta2_I;
			}
			else{
				if (abs(theta2_I) >= 0 && abs(theta2_I) < M_PI/2)
					theta2_sol = theta2_I;
			}
			theta2_sol=M_PI/2-theta2_sol;      // Given the axis of frame1_hat, to compute theta2_sol we need to subtract from pi/2
			hipot = sqrt(pow(X_f1_hat_T_goal, 2) + pow(Y_f1_hat_T_goal, 2));
			L1 = sqrt(pow(link5_length, 2) + pow(link4_length, 2));

			c4_temp = (pow(hipot, 2) +  pow((Z_f1_hat_T_goal-link2_length), 2) - pow(link3_length,2) - pow(L1,2))/(2*link3_length*L1);
			subtraction = 1-pow(c4_temp,2);
			if (abs(subtraction) < 0.0025)																											// Changed this value to avoid different results with Matlab
				subtraction = 0;

			std::vector<double> arm_solution(4);
			double s4_plus, s4_minus, theta4_I_temp, theta4_II_temp, beta, alpha_I, alpha_II, theta3_I, theta3_II, theta4_I, theta4_II;
			if(subtraction<0){																																	// Can be negative due to mathematically unfeasibility orientations or if desired point is out of the finger reachibility space
				std::cout << "Subtraction was negative. Similar to unreachable space. Ending..." << std::endl;
			}
			else
			{
				s4_plus = sqrt(subtraction);
				s4_minus = -sqrt(subtraction);
				theta4_I_temp = atan2(s4_plus,c4_temp);
				theta4_II_temp = atan2(s4_minus,c4_temp);

				beta = atan2(hipot, (Z_f1_hat_T_goal-link2_length));

				alpha_I = atan2(L1*sin(theta4_I_temp), link3_length + L1*cos(theta4_I_temp))	;
				alpha_II = atan2(L1*sin(theta4_II_temp), link3_length + L1*cos(theta4_II_temp));

				theta3_I = beta -  alpha_I;
				theta3_II = beta -  alpha_II;
				theta4_I = theta4_I_temp - extra_angle;
				theta4_II = theta4_II_temp - extra_angle;

				arm_solution = {q1_partial_sol, theta2_sol, theta3_I, theta4_I};
				// std::cout << "arm_solution at first are: " << std::endl;
				// printVector(arm_solution);

				arm_sols_accumulator.push_back(arm_solution);
				arm_solution = {q1_partial_sol, theta2_sol, theta3_II, theta4_II};
				// std::cout << "arm_solution at second are: " << std::endl;
				// printVector(arm_solution);
				arm_sols_accumulator.push_back(arm_solution);
			}
		}
		// std::cout << "arm_sols_accumulator are:" << std::endl;
		// for (std::vector<double> vect:arm_sols_accumulator)
		// {
		// 	printVector(vect);
		// }

		// std::cout << "Fnally the results are:" << std::endl;
		final_arm_solutions = considerValidJointValues(arm_sols_accumulator, true);


		// for (std::vector<double> vect:final_arm_solutions)
		// {
		// 	printVector(vect);
		// }
	}
	else
	{
		std::cout << "GOAL POINT IS OUT OF THE WORKSPACE OF THE FINGER" << std::endl;
		final_arm_solutions = {{0,0,0,0}};
	}
	bool correct_sol_found = false;

	std::cout << "/*/*/*/*/ final_arm_solutions are:"<< std::endl;
	for(unsigned int i = 0; i < final_arm_solutions.size();i++)
	{
		printVector(final_arm_solutions[i]);
	}
	final_vect_joint_values = analyze_results_quality(final_arm_solutions, T_desired, &correct_sol_found);
	// std::cout << "final_vect_joint_values is" << std::endl;
	// printVector(final_vect_joint_values);
	// std::cout << "correct_sol_found is: " << correct_sol_found<<std::endl;
	joint_angles[0] = final_vect_joint_values[0];
	joint_angles[1] = final_vect_joint_values[1];
	joint_angles[2] = final_vect_joint_values[2];
	joint_angles[3] = final_vect_joint_values[3];
	updateA_mats(joint_angles);
	return final_arm_solutions;
}

bool ThumbFinger::IkineThumb(std::vector<double> goal_eul_ang, std::vector<double> goal_pos, std::vector<std::vector<double>> &q_values_ikine)
{
    //const double reorientation = 0.05;				// Reorientation increment in case current orientation is finger-joint-angles unfeasible
    //int stage_IK_solver = 0;									// Indicates stage of the IK solution
    //double delta_orientation = 0;							// Angle change in case of finger-joint-angles unfeasibility
    //int turn_complete = 0;										// Complete turn to find feasible finger joint angles
	bool success = true;
	std::vector<std::vector<double>> arm_sols_accumulator;
	std::vector<double> final_vect_joint_values;
	std::vector <Eigen::Vector3d> solutions_eq_syst;
	// Convert the desired goal pose wrt the frame of the base of the finger
	Eigen::Affine3d T_desired_wrt_base_hand_frame, T_desired_wrt_base_finger, T_desired;
	std::vector<double> goal_pos_wrt_base_finger(3);
	T_desired_wrt_base_hand_frame = eul2Tmat(goal_eul_ang, goal_pos);

	// std::cout << "T_world_base is: " <<std::endl<< T_world_base << std::endl;
	// std::cout << "T_desired_wrt_base_hand_frame is: " <<std::endl<< T_desired_wrt_base_hand_frame << std::endl;
	// std::cout << "T_world_base.inverse() is: " <<std::endl<< T_world_base.inverse() << std::endl;

	T_desired_wrt_base_finger = T_desired_wrt_base_hand_frame*T_tcp_fingerTip.inverse();
	T_desired_wrt_base_finger = T_world_base.inverse()*T_desired_wrt_base_finger;
	// std::cout << "T_desired_wrt_base_finger is: " <<std::endl<< T_desired_wrt_base_finger << std::endl;

	T_desired = T_desired_wrt_base_finger;
	// std::cout << "T_desired is: " << T_desired << std::endl;
	goal_pos_wrt_base_finger = {T_desired(0,3), T_desired(1,3), T_desired(2,3)};

	bool point_in_ws = true;
	point_in_ws = checkPointInsideWs(goal_pos_wrt_base_finger[0], goal_pos_wrt_base_finger[1], goal_pos_wrt_base_finger[2]);
	// std::cout << "point_in_ws is: "<<point_in_ws<< std::endl;
	if(point_in_ws==1)
	{
		// std::cout << "The T_desired matrix is: " <<std::endl<< T_desired << std::endl;
		// Goal HT matrix expressed in frame "aux", which resembles frame 1 of the finger
		Eigen::Matrix4d T_desired_in_aux, T_desired_in_aux2;
		Eigen::Affine3d translation1(Eigen::Translation3d(Eigen::Vector3d(0,0,-link5_length)));
		T_desired_in_aux = T_desired*(translation1).matrix();

		Eigen::Affine3d translation2(Eigen::Translation3d(Eigen::Vector3d(link4_length,0,0)));
		Eigen::Affine3d rz = Eigen::Affine3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0, 0, 1)));
		Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(1, 0, 0)));
		T_desired_in_aux2=T_desired_in_aux*(translation2*rz*rx).matrix();
		// std::cout << "T_desired_in_aux2 is: " << std::endl<<T_desired_in_aux2 << std::endl;

		double px, py, pz;
		px = T_desired_in_aux2(0,3);
		py = T_desired_in_aux2(1,3);
		pz = T_desired_in_aux2(2,3);

		///*/*/*/*/*/*//*/*/**/*/*/*/*/*/*/ */*/*/*/*/ HERE PUT THE SOLVER CODE
		solutions_eq_syst=systemEquationsSolver(px, py, pz);
		// std::cout << "Solver solutions are:"<< std::endl;
		// for(unsigned int i = 0; i < solutions_eq_syst.size();i++)
		// {
		// 		std::cout << "q1 = " << solutions_eq_syst[i](0) <<", ";
		// 		std::cout << "q2 = " << solutions_eq_syst[i](1) <<", ";
		// 		std::cout << "q3 = " << solutions_eq_syst[i](2) << std::endl;
		// }
		// double solver_value;
		// std::cout << "size of the solutions solver is: " << solutions_eq_syst.size() << std::endl;

		std::vector<std::vector<double>> partial_q_values(solutions_eq_syst.size()), valid_partial_q_vals;
        for (unsigned int i = 0 ; i < solutions_eq_syst.size() ; i++) {
            partial_q_values[i].resize(3);
        }
		for(unsigned int i = 0; i < solutions_eq_syst.size();i++)
		{
			partial_q_values[i][0] = solutions_eq_syst[i](0);
			partial_q_values[i][1] = solutions_eq_syst[i](1);
			partial_q_values[i][2] = solutions_eq_syst[i](2);
		}

		valid_partial_q_vals = considerValidJointValues(partial_q_values, false);
		// std::cout << "valid_partial_q_vals are:"<< std::endl;
		// for(unsigned int i = 0; i < valid_partial_q_vals.size();i++)
		// {
		// 	printVector(valid_partial_q_vals[i]);
		// }

		std::vector<double> q1_sols_to_test;
		for (std::vector<double> vect:valid_partial_q_vals)
		{
			q1_sols_to_test.push_back(vect[0]);
			// printVector(vect);
		}

		// std::cout << "THe q1_sols_to_test are: " << std::endl;
		// printVector(q1_sols_to_test);

		for(double q1_partial_sol:q1_sols_to_test)
		{
			Eigen::Affine3d b_T_f1, b_T_f1_hat, f1_hat_T_goal;		// Transform from base to frame f1, transform from base to frame f1_hat and transform from frame f1_hat to goal frame
			b_T_f1.matrix() << cos(q1_partial_sol+t_offs(0)), -sin(q1_partial_sol+t_offs(0))*cos(alpha(0)),   sin(q1_partial_sol+t_offs(0))*sin(alpha(0)), a(0)*cos(q1_partial_sol+t_offs(0)),
					sin(q1_partial_sol+t_offs(0)),  cos(q1_partial_sol+t_offs(0))*cos(alpha(0)),  -cos(q1_partial_sol+t_offs(0))*sin(alpha(0)), a(0)*sin(q1_partial_sol+t_offs(0)),
												0,                                  sin(alpha(0)),                            	  	cos(alpha(0)),                          d(0),
												0,                               					  0,                                		0,                             		 			1;
			Eigen::Affine3d rotx = Eigen::Affine3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1, 0, 0)));
			b_T_f1_hat = b_T_f1.matrix()*rotx.matrix();
			f1_hat_T_goal = b_T_f1_hat.matrix().inverse()*T_desired.matrix();
			// std::cout << "f1_hat_T_goal is: " << std::endl<< f1_hat_T_goal<< std::endl;

			double X_f1_hat_T_goal, Y_f1_hat_T_goal, Z_f1_hat_T_goal;	// X, Y and Z coordinates of the transformation that relates f1_hat frame to the goal frame
			X_f1_hat_T_goal = f1_hat_T_goal(0,3);
			Y_f1_hat_T_goal = f1_hat_T_goal(1,3);
			Z_f1_hat_T_goal = f1_hat_T_goal(2,3);

			//------------ Here it begins the computation of the last 3 joint angles
			Eigen::Matrix<double, 4, 4> f1_hat_T_auxf4;
			Eigen::Affine3d rotz = Eigen::Affine3d(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(0, 0, 1)));

			rotx = Eigen::Affine3d(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(1, 0, 0)));
			f1_hat_T_auxf4 = f1_hat_T_goal*(rotx*rotz).matrix();		//T matrix from the f1_hat frame to goal point translated to frame 4
			// std::cout << "f1_hat_T_auxf4 is: " << std::endl<< f1_hat_T_auxf4<< std::endl;

			double theta, extra_angle;
			Eigen::Matrix<double, 3, 1> new_eul_angles;

			// Conversion to eul angles from HT matrix
			Eigen::Matrix3d rotation_part = f1_hat_T_auxf4.block<3,3>(0,0);
			new_eul_angles = rotation_part.eulerAngles(2,1,2);
			// std::cout << "new_eul_angles is: " << std::endl<< new_eul_angles<< std::endl;


			theta = new_eul_angles[2];						// Retrieve the last eul angle value as this is expected to indicate the total roation of the last 3 joints of the finger
			// std::cout << "theta at the beginning is: " << std::endl<< theta<< std::endl;
			// Added this verification because the angles that C++ finds with "eulerAngles" in some cases are different from the ones obtained in Matlab. Are different but the overall rotation matrix remains the same. However, this needs further tests, not all cases were cover
			if (theta<0)
				theta = theta+ M_PI;
			else
				theta = theta- M_PI;
			theta = theta - M_PI/2;							// The pi/2 addition is due to the offset turn in Z that we do for the 5th link DH parameters.
			extra_angle = atan2(link5_length,link4_length);	// Corresponds to the angle between the links 4 and 5
			// std::cout << "theta at the end is: " << std::endl<< theta<< std::endl;

			double theta2_I, theta2_sol, hipot, L1, c4_temp, subtraction;
			theta2_I = atan2(Y_f1_hat_T_goal,X_f1_hat_T_goal);
			if (abs(theta2_I) < 0.001) {
				theta2_I = 0;
				theta2_sol = theta2_I;
			}
			else{
				if (abs(theta2_I) >= 0 && abs(theta2_I) < M_PI/2)
					theta2_sol = theta2_I;
			}
			theta2_sol=M_PI/2-theta2_sol;      // Given the axis of frame1_hat, to compute theta2_sol we need to subtract from pi/2
			hipot = sqrt(pow(X_f1_hat_T_goal, 2) + pow(Y_f1_hat_T_goal, 2));
			L1 = sqrt(pow(link5_length, 2) + pow(link4_length, 2));

			c4_temp = (pow(hipot, 2) +  pow((Z_f1_hat_T_goal-link2_length), 2) - pow(link3_length,2) - pow(L1,2))/(2*link3_length*L1);
			subtraction = 1-pow(c4_temp,2);
			if (abs(subtraction) < 0.0025)																											// Changed this value to avoid different results with Matlab
				subtraction = 0;

			std::vector<double> arm_solution(4);
			double s4_plus, s4_minus, theta4_I_temp, theta4_II_temp, beta, alpha_I, alpha_II, theta3_I, theta3_II, theta4_I, theta4_II;
			if(subtraction<0){																																	// Can be negative due to mathematically unfeasibility orientations or if desired point is out of the finger reachibility space
				std::cout << "Subtraction was negative. Similar to unreachable space. Ending..." << std::endl;
			}
			else
			{
				s4_plus = sqrt(subtraction);
				s4_minus = -sqrt(subtraction);
				theta4_I_temp = atan2(s4_plus,c4_temp);
				theta4_II_temp = atan2(s4_minus,c4_temp);

				beta = atan2(hipot, (Z_f1_hat_T_goal-link2_length));

				alpha_I = atan2(L1*sin(theta4_I_temp), link3_length + L1*cos(theta4_I_temp))	;
				alpha_II = atan2(L1*sin(theta4_II_temp), link3_length + L1*cos(theta4_II_temp));

				theta3_I = beta -  alpha_I;
				theta3_II = beta -  alpha_II;
				theta4_I = theta4_I_temp - extra_angle;
				theta4_II = theta4_II_temp - extra_angle;

				arm_solution = {q1_partial_sol, theta2_sol, theta3_I, theta4_I};
				// std::cout << "arm_solution at first are: " << std::endl;
				// printVector(arm_solution);

				arm_sols_accumulator.push_back(arm_solution);
				arm_solution = {q1_partial_sol, theta2_sol, theta3_II, theta4_II};
				// std::cout << "arm_solution at second are: " << std::endl;
				// printVector(arm_solution);
				arm_sols_accumulator.push_back(arm_solution);
			}
		}
		// std::cout << "arm_sols_accumulator are:" << std::endl;
		// for (std::vector<double> vect:arm_sols_accumulator)
		// {
		// 	printVector(vect);
		// }


		// std::cout << "Fnally the results are:" << std::endl;
		q_values_ikine = considerValidJointValues(arm_sols_accumulator, true);

		// std::cout << "q_values_ikine are:" << std::endl;
		// for (std::vector<double> vect:q_values_ikine)
		// {
		// 	printVector(vect);
		// }
	}
	else
	{
		std::cout << "GOAL POINT IS OUT OF THE WORKSPACE OF THE FINGER" << std::endl;
		success = false;
	}
	bool correct_sol_found = false;
	if (q_values_ikine.size()==0){
		success = false;
	}
	else{
		// std::cout << "/*/*/*/*/ final_arm_solutions are:"<< std::endl;
		// std::cout << q_values_ikine.size() << std::endl;
		// for(unsigned int i = 0; i < q_values_ikine.size();i++)
		// {
		// 	printVector(q_values_ikine[i]);
		// }
		final_vect_joint_values = analyze_results_quality(q_values_ikine, T_desired, &correct_sol_found);
		// std::cout << "final_vect_joint_values is" << std::endl;
		// printVector(final_vect_joint_values);
		// std::cout << "correct_sol_found is: " << correct_sol_found<<std::endl;
		joint_angles[0] = final_vect_joint_values[0];
		joint_angles[1] = final_vect_joint_values[1];
		joint_angles[2] = final_vect_joint_values[2];
		joint_angles[3] = final_vect_joint_values[3];
		updateA_mats(joint_angles);
	}
	return success;
}

// //! \brief Compute Ikine based on a point and a normal
// //! \param goal_normal. Normal direction of the finger tip
// //! \param goal_pos. X,Y,Z point coordinates
// //! \param q_values_ikine. Resulting q values
// bool ThumbFinger::IkineNormal(std::vector<double> goal_normal, std::vector<double> goal_pos){
//  // WIP
//  return true;
//  }

std::vector<std::vector<double>> ThumbFinger::considerValidJointValues(std::vector<std::vector<double>> partial_q_values, bool four_elements)
{
	// Limits of the joint (in rad)
	std::vector<std::vector<double>> mat_feasibility, validJointValues;
	std::vector<double> vect_feasibility;
	if(four_elements)
		vect_feasibility = {0,0,0,0};
	else
		vect_feasibility = {0,0,0};

	int index = 0;
	// std::cout << "I will begin this function " << std::endl;
	for (std::vector<double> vec_to_check:partial_q_values)
    {
		// std::cout << "THe vec_to_check are: " << std::endl;
		// printVector(vec_to_check);
		for(double q_to_check:vec_to_check)
		{
			if(q_to_check <= q_max_limits[index] && q_to_check >= q_min_limits[index])
				vect_feasibility[index]=1;
			else
				vect_feasibility[index]=-1;
			// std::cout << "THe vect_feasibility are: " << std::endl;
			// printVector(vect_feasibility);
			index++;
		}
		mat_feasibility.push_back(vect_feasibility);
		index = 0;
    }

	// std::cout << "At the end of first stage mat_feasibility are: " << std::endl;
	// for (std::vector<double> vec_to_print:mat_feasibility)
    // {
	// 	printVector(vec_to_print);
	// }

	index = 0;
	int outter_index = 0;
    double aux;
	for (std::vector<double> vec_to_check:partial_q_values)
    {
		vect_feasibility = mat_feasibility[outter_index];
		// std::cout << "One vect of mat_feas is: " << std::endl;
		// printVector(vect_feasibility);
		for(double q_to_check:vec_to_check)
		{
			if(vect_feasibility[index] == -1)
			{
				if(q_to_check < 0)
				{
					if(q_to_check < -M_PI)
					{
						aux = q_to_check + 2*M_PI;
						if(aux <= q_max_limits[index] && aux >= q_min_limits[index])
							vect_feasibility[index] = 1;
						else
							vect_feasibility[index] = -1;
					}
				}
				else
				{
					if(q_to_check > 2*M_PI)
					{
						aux = q_to_check - 2*M_PI;
						if(aux <= q_max_limits[index] && aux >= q_min_limits[index])
							vect_feasibility[index] = 1;
						else
							vect_feasibility[index] = -1;
					}
				}
				// std::cout << "Iterating in the second stage vect_feasibility are: " << std::endl;
				// printVector(vect_feasibility);
			}
			index++;
		}
		index = 0;
		outter_index++;
    }
	// std::cout << "At the end of second stage mat_feasibility are: " << std::endl;
	// for (std::vector<double> vec_to_print:mat_feasibility)
    // {
	// 	printVector(vec_to_print);
	// }
	index = 0;
	outter_index = 0;

	for (std::vector<double> vec_to_check:partial_q_values)
    {
		vect_feasibility = mat_feasibility[outter_index];
		if ( std::find(vect_feasibility.begin(), vect_feasibility.end(), -1) != vect_feasibility.end() )
		{
			// std::cout << "THere is -1 value, so vect is not valid " << std::endl;
			// // pass;
		}
		else
		{
			// std::cout << "NO -1 value, pushing this vect " << std::endl;
			validJointValues.push_back(vec_to_check);

		}

		outter_index++;
	}
	// std::cout << "The final validJointValues are: " << std::endl;
	// for (std::vector<double> vec_to_print:validJointValues)
    // {
	// 	printVector(vec_to_print);
	// }

    return validJointValues;
}



bool ThumbFinger::checkPointInsideWs(double px, double py, double pz)
{
	double eq1 = -4985698.162763160*pow(px, 7) + 2780723.953785964*pow(px, 6) -606977.1046804322*pow(px, 5) + 65070.94915267395*pow(px, 4) -3498.043549937818*pow(px, 3) + 77.122096006162707*pow(px, 2) + 0.408768111479018*px + 0.056859191141542;
	double eq2 = 15314617.67137061*pow(px, 7) -9397032.647032414*pow(px, 6) + 2312936.236372822*pow(px, 5) - 288620.4156835210*pow(px, 4) + 18780.98737167166*pow(px, 3) - 554.4056439605333*pow(px, 2) + 2.705452287358597*px - 0.041966028585322;

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
		double eq3 = -885635.2760538224*pow(x_on_XZ_plane,7) + 395521.1472916254*pow(x_on_XZ_plane,6) - 65746.69289653355*pow(x_on_XZ_plane,5) + 4638.299355156986*pow(x_on_XZ_plane,4) - 56.099284733917912*pow(x_on_XZ_plane,3) - 17.114385284777349*pow(x_on_XZ_plane,2) + 1.449042220196802*x_on_XZ_plane + 0.067598985016500;
		double eq4 = -49934.48228336617*pow(x_on_XZ_plane,7) + 67942.92044293317*pow(x_on_XZ_plane,6) - 50555.09168222573*pow(x_on_XZ_plane,5) + 15110.82066295201*pow(x_on_XZ_plane,4) - 2095.124224915341*pow(x_on_XZ_plane,3) + 138.3651771699399*pow(x_on_XZ_plane,2) - 3.856029683707503*x_on_XZ_plane + 0.054420046412870;

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

std::vector<double> ThumbFinger::analyze_results_quality(std::vector<std::vector<double>> final_joint_values, Eigen::Affine3d T_desired, bool *correct_sol_found)
{
	// std::cout << std::endl;
	// std::cout << "The analyze_results_quality function"<<std::endl;

	Eigen::Affine3d fkine_sol_found, mat_subtraction;
	Eigen::Matrix<double, 1, 4> aux, aux2;
	// int tot_size;
	int ones_counter=0, winner_index, index=0;
	Eigen::Matrix<double, 1, 1> sum_vect;

    // tot_size = final_joint_values.size();
	// std::vector<double> winner_vect = {100, 100, 100, 100};
	Eigen::Matrix<double, 1, 4> winner_vect;
	winner_vect << 100, 100, 100, 100;
	std::vector<double> res;
	// std::cout << "At the beginning winner_vect is: "<<winner_vect<< std::endl;
	for (std::vector<double> vec_q_values:final_joint_values)
    {
		fkine_sol_found = Fkine(vec_q_values);
		mat_subtraction = (T_desired.matrix() - fkine_sol_found.matrix());
    	mat_subtraction = mat_subtraction.matrix().array().abs();
		// std::cout << "mat_subtraction is: "<<std::endl<<mat_subtraction<< std::endl;

		aux = mat_subtraction.matrix().colwise().sum();
		// std::cout << "aux is: "<<std::endl<<aux<< std::endl;
		for(int i=0; i<4; i++)
		{
			// std::cout << "aux(i), is: "<<aux(i)<< std::endl;
			// std::cout << "winner_vect(i), is: "<<winner_vect(i)<< std::endl;
			if(aux(i) < winner_vect(i))
			{
				aux2(i) = 1;
			}
			else
				aux2(i) = 0;
		}

		// std::cout << "aux2 is: "<<std::endl<<aux2<< std::endl;
		for(int j=0; j<4; j++)
		{
			if(aux2[j]==1)
				ones_counter++;
		}
		if(ones_counter>2)
		{
			winner_vect = aux;
			// std::cout << "Now,  winner_vect is: "<<winner_vect<< std::endl;
			winner_index = index;
		}
		// std::cout << "winner_index is: "<<std::endl<<winner_index<< std::endl;
		index++;
		ones_counter=0;
	}
	sum_vect = winner_vect.rowwise().sum();
	// winner_index = 0;
	// std::cout << "winner_vect.rowwise().sum() is: " << sum_vect<<std::endl;
	if(sum_vect(0,0)<0.04)
	{
		*correct_sol_found = true;
	}
	res = final_joint_values[winner_index];
	return res;
}
