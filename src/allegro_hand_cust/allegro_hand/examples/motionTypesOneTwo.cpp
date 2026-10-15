#include <iostream>
#include <vector>
#include <allegro_hand/kinematics/AllegroFinger.h>
#include <allegro_hand/kinematics/IMAFinger.h>
#include <allegro_hand/kinematics/ThumbFinger.h>
#include <allegro_hand/kinematics/operations.h>

int main()
{
	Eigen::Affine3d fk_mat;
	Eigen::Matrix<double, 2, 4> ik_sols;
	Eigen::Affine3d TF_from_w2bfinger, tot_mat;
	std::vector<double> q_values_finger(4);
	std::vector<double> desired_pos, desired_orntn;
	std::vector<std::vector<double>> motion1_sols, motion2_sols;

	//FOR THE INDEX FINGER-------------------------------------------------------------------------
	IMAFinger index_finger(0);
	// Desired ortn and pos for q_test=[0.25 0.49 0.85 0.72 0 0];
	desired_orntn = {-2.8480,   -2.0365,   -3.0469};	// Orientation wrt the base of the hand
	desired_pos = {0.0897, -0.0243, 0.0422};			// Pos wrt the base of the hand
	std::cout << "#######INDEX FINGER###### " <<std::endl;
	std::cout << "The desired position is: " << desired_pos[0] <<", "<< desired_pos[1] <<", " << desired_pos[2] << std::endl;
	std::cout << "The desired orientation in ZYZ eul angles is: " << desired_orntn[0] <<", "<< desired_orntn[1] <<", " << desired_orntn[2] << std::endl;
	std::cout << "*********MOTION TYPE 1 DEMO***** " <<std::endl;
	motion1_sols = index_finger.obtainMotion1Sols(desired_orntn, desired_pos);

	std::cout << "motion1_sols is: " <<std::endl;
	for(auto i=0u ; i<motion1_sols.size(); i++)
	{
		std::vector<double> vect_aux;
		vect_aux = motion1_sols[i];
		for(auto j=0u ; j<vect_aux.size(); j++)
		{
			std::cout <<vect_aux[j]<< " ";
		}
		std::cout << std::endl;
	}

	std::cout << "*********MOTION TYPE 2 DEMO*****" <<std::endl;
	motion2_sols = index_finger.obtainMotion2Sols(desired_orntn, desired_pos);
	std::cout << "motion2_sols is: " <<std::endl;
	for(auto i=0u ; i<motion2_sols.size(); i++)
	{
		std::vector<double> vect_aux;
		vect_aux = motion2_sols[i];
		for(auto j=0u ; j<vect_aux.size(); j++)
		{
			std::cout <<vect_aux[j]<< " ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;
	std::cout << std::endl;



	//FOR THE MIDDLE FINGER-------------------------------------------------------------------------
	IMAFinger middle_finger(1);
	// Desired ortn and pos for q_test=[0.25 0.49 0.85 0.72 0 0];
	desired_orntn = {-2.8916,  -2.0600,   -3.1416};		// Orientation wrt the base of the hand
	desired_pos = {0.0897, 0.0229, 0.0426};	// Pos wrt the base of the hand
	std::cout << "#######MIDDLE FINGER###### " <<std::endl;
	std::cout << "The desired position is: " << desired_pos[0] <<", "<< desired_pos[1] <<", " << desired_pos[2] << std::endl;
	std::cout << "The desired orientation in ZYZ eul angles is: " << desired_orntn[0] <<", "<< desired_orntn[1] <<", " << desired_orntn[2] << std::endl;
	// std::vector<std::vector<double>> motion1_sols;
	std::cout << "*********MOTION TYPE 1 DEMO***** " <<std::endl;
	motion1_sols = middle_finger.obtainMotion1Sols(desired_orntn, desired_pos);

	std::cout << "motion1_sols is: " <<std::endl;
	for(auto i = 0u ; i<motion1_sols.size(); i++)
	{
		std::vector<double> vect_aux;
		vect_aux = motion1_sols[i];
		for(auto j=0u ; j<vect_aux.size(); j++)
		{
			std::cout <<vect_aux[j]<< " ";
		}
		std::cout << std::endl;
	}

	// std::vector<std::vector<double>> motion2_sols;
	std::cout << "*********MOTION TYPE 2 DEMO*****" <<std::endl;
	motion2_sols = middle_finger.obtainMotion2Sols(desired_orntn, desired_pos);
	std::cout << "motion2_sols is: " <<std::endl;
	for(auto i=0u ; i<motion2_sols.size(); i++)
	{
		std::vector<double> vect_aux;
		vect_aux = motion2_sols[i];
		for(auto j=0u ; j<vect_aux.size(); j++)
		{
			std::cout <<vect_aux[j]<< " ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;
	std::cout << std::endl;



	//FOR THE MIDDLE FINGER-------------------------------------------------------------------------
	IMAFinger anular_finger(2);
	// Desired ortn and pos for q_test=[0.25 0.49 0.85 0.72 0 0];
	desired_orntn = {-2.9380,   -2.0796,    3.0447};		// Orientation wrt the base of the hand
	desired_pos = {0.0897, 0.07, 0.0382};	// Pos wrt the base of the hand
	std::cout << "#######ANULAR FINGER###### " <<std::endl;
	std::cout << "The desired position is: " << desired_pos[0] <<", "<< desired_pos[1] <<", " << desired_pos[2] << std::endl;
	std::cout << "The desired orientation in ZYZ eul angles is: " << desired_orntn[0] <<", "<< desired_orntn[1] <<", " << desired_orntn[2] << std::endl;
	// std::vector<std::vector<double>> motion1_sols;
	std::cout << "*********MOTION TYPE 1 DEMO***** " <<std::endl;
	motion1_sols = anular_finger.obtainMotion1Sols(desired_orntn, desired_pos);

	std::cout << "motion1_sols is: " <<std::endl;
	for(auto i=0u ; i<motion1_sols.size(); i++)
	{
		std::vector<double> vect_aux;
		vect_aux = motion1_sols[i];
		for(auto j=0u ; j<vect_aux.size(); j++)
		{
			std::cout <<vect_aux[j]<< " ";
		}
		std::cout << std::endl;
	}

	std::cout << "*********MOTION TYPE 2 DEMO*****" <<std::endl;
	motion2_sols = anular_finger.obtainMotion2Sols(desired_orntn, desired_pos);
	std::cout << "motion2_sols is: " <<std::endl;
	for(auto i=0u ; i<motion2_sols.size(); i++)
	{
		std::vector<double> vect_aux;
		vect_aux = motion2_sols[i];
		for(auto j=0u ; j<vect_aux.size(); j++)
		{
			std::cout <<vect_aux[j]<< " ";
		}
		std::cout << std::endl;
	}


	return 0;
}
