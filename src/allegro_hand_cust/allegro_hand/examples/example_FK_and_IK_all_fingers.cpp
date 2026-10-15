#include <iostream>
#include <vector>
#include <allegro_hand/kinematics/AllegroFinger.h>
#include <allegro_hand/kinematics/IMAFinger.h>
#include <allegro_hand/kinematics/ThumbFinger.h>
#include <allegro_hand/kinematics/operations.h>

int main()
{
	// std::vector<double> q_test = {0.25, 0.49, 0.84, 0.71};
	std::vector<double> q_test = {0.95, 0.31, 0.68, 0.79};
	Eigen::Affine3d fk_mat;
	Eigen::Matrix<double, 2, 4> ik_sols;
	std::vector<std::vector<double>> ik_sols_thumb;
	Eigen::Affine3d TF_from_w2bfinger, tot_mat;
	std::vector<double> q_values_finger(4);

	//FOR THE THUMB FINGER-------------------------------------------------------------------------
	ThumbFinger thumb_finger;
	std::cout << "#######THUMB FINGER###### " <<std::endl;
	std::cout << "*******DEMO OF IKINE***** " <<std::endl;
	std::vector<double> desired_pos, desired_orntn;
	desired_orntn = { -2.7583, -0.2836,  -1.0117};	// Orientation wrt the base of the hand
	desired_pos = {0.0738,  -0.0506,  0.0100};
	std::cout << "The desired position is: " << desired_pos[0] <<", "<< desired_pos[1] <<", " << desired_pos[2] << std::endl;
	std::cout << "The desired orientation in ZYZ eul angles is: " << desired_orntn[0] <<", "<< desired_orntn[1] <<", " << desired_orntn[2] << std::endl;
	ik_sols_thumb = thumb_finger.IkineThumb(desired_orntn, desired_pos);
	q_values_finger = thumb_finger.getJointAngles();
	std::cout << "The q values found from IK, are: " <<std::endl;
	printVector(q_values_finger);

	std::cout << std::endl;
	std::cout << std::endl;

	std::cout << "*******DEMO OF FKINE***** " <<std::endl;
	fk_mat = thumb_finger.Fkine(q_values_finger);
	std::cout << "Given the angles: " << q_values_finger[0] <<", "<< q_values_finger[1] <<", " << q_values_finger[2] <<", " << q_values_finger[3] << std::endl;
	TF_from_w2bfinger = thumb_finger.getTF_w2b() ;
	// std::cout << "TF_from_w2bfinger is: " << TF_from_w2bfinger <<std::endl;
	tot_mat = TF_from_w2bfinger*fk_mat*thumb_finger.getTF_tcp2tip();
	std::cout << "The FK matrix with the previously found IK angles, is: " <<std::endl<< tot_mat.matrix() << std::endl;
	Eigen::Matrix<double,3,1> eul_angles_solved, translation_solved;
	Eigen::Quaterniond new_quat_obtained;
	new_quat_obtained = obtainQuaternion(tot_mat);
	std::cout << "The equivalent quaternion element 'w' is: " << new_quat_obtained.w() <<std::endl;
	std::cout << "The equivalent quaternion element 'vec' is: " << std::endl << new_quat_obtained.vec() <<std::endl;
	eul_angles_solved = obtainEulAngles(tot_mat);
	std::cout << "The equivalent in euler angles (ZYZ) is: " << std::endl << eul_angles_solved <<std::endl;
	translation_solved = obtainTranslation(tot_mat);
	std::cout << "The tanslation vector is: " << std::endl << translation_solved <<std::endl;
	std::cout << std::endl;
	std::cout << std::endl;

	//FOR THE INDEX FINGER-------------------------------------------------------------------------
	IMAFinger index_finger(0);
	desired_orntn = {-2.8480,   -2.0365,   -3.0469};	// Orientation wrt the base of the hand
	desired_pos = {0.0897, -0.0243, 0.0422};			// Pos wrt the base of the hand
	std::cout << "#######INDEX FINGER###### " <<std::endl;
	std::cout << "*******DEMO OF IKINE***** " <<std::endl;
	std::cout << "The desired position is: " << desired_pos[0] <<", "<< desired_pos[1] <<", " << desired_pos[2] << std::endl;
	std::cout << "The desired orientation in ZYZ eul angles is: " << desired_orntn[0] <<", "<< desired_orntn[1] <<", " << desired_orntn[2] << std::endl;
	ik_sols = index_finger.Ikine(desired_orntn, desired_pos);
	// std::cout << "The joint values that were found are: " << ik_sols <<std::endl;
	q_values_finger = index_finger.getJointAngles();
	std::cout << "The q values found from IK, are: " <<std::endl;
	printVector(q_values_finger);
	std::cout << std::endl;

	std::cout << "*******DEMO OF FKINE***** " <<std::endl;
	fk_mat = index_finger.Fkine(q_values_finger);
	TF_from_w2bfinger = index_finger.getTF_w2b() ;
	// std::cout << "TF_from_w2bfinger is: " << TF_from_w2bfinger <<std::endl;
	tot_mat = TF_from_w2bfinger*fk_mat*index_finger.getTF_tcp2tip();
	std::cout << "The FK matrix with the previously found IK angles, is: " <<std::endl<< tot_mat.matrix() << std::endl;

	new_quat_obtained = obtainQuaternion(tot_mat);
	std::cout << "The equivalent quaternion element 'w' is: " << new_quat_obtained.w() <<std::endl;
	std::cout << "The equivalent quaternion element 'vec' is: " << std::endl << new_quat_obtained.vec() <<std::endl;
	eul_angles_solved = obtainEulAngles(tot_mat);
	std::cout << "The equivalent in euler angles (ZYZ) is: " << std::endl << eul_angles_solved <<std::endl;
	translation_solved = obtainTranslation(tot_mat);
	std::cout << "The tanslation vector is: " << std::endl << translation_solved <<std::endl;
	std::cout << std::endl;
	std::cout << std::endl;

	//FOR THE MIDDLE FINGER-------------------------------------------------------------------------
	IMAFinger middle_finger(1);
	desired_orntn = {-2.8916,  -2.0600,   -3.1416};		// Orientation wrt the base of the hand
	desired_pos = {0.0897, 0.0229, 0.0426};	// Pos wrt the base of the hand
	std::cout << "#######MIDDLE FINGER###### " <<std::endl;
	std::cout << "*******DEMO OF IKINE***** " <<std::endl;
	std::cout << "The desired position is: " << desired_pos[0] <<", "<< desired_pos[1] <<", " << desired_pos[2] << std::endl;
	std::cout << "The desired orientation in ZYZ eul angles is: " << desired_orntn[0] <<", "<< desired_orntn[1] <<", " << desired_orntn[2] << std::endl;
	ik_sols = middle_finger.Ikine(desired_orntn, desired_pos);
	// std::cout << "The joint values that were found are: " << ik_sols <<std::endl;
	q_values_finger = middle_finger.getJointAngles();
	std::cout << "The q values found from IK, are: " <<std::endl;
	printVector(q_values_finger);
	std::cout << std::endl;

	std::cout << "*******DEMO OF FKINE***** " <<std::endl;
	fk_mat = middle_finger.Fkine(q_values_finger);
	TF_from_w2bfinger = middle_finger.getTF_w2b() ;
	// std::cout << "TF_from_w2bfinger is: " << TF_from_w2bfinger <<std::endl;
	tot_mat = TF_from_w2bfinger*fk_mat*middle_finger.getTF_tcp2tip();
	std::cout << "The FK matrix with the previously found IK angles, is: " <<std::endl<< tot_mat.matrix() << std::endl;

	new_quat_obtained = obtainQuaternion(tot_mat);
	std::cout << "The equivalent quaternion element 'w' is: " << new_quat_obtained.w() <<std::endl;
	std::cout << "The equivalent quaternion element 'vec' is: " << std::endl << new_quat_obtained.vec() <<std::endl;
	eul_angles_solved = obtainEulAngles(tot_mat);
	std::cout << "The equivalent in euler angles (ZYZ) is: " << std::endl << eul_angles_solved <<std::endl;
	translation_solved = obtainTranslation(tot_mat);
	std::cout << "The tanslation vector is: " << std::endl << translation_solved <<std::endl;
	std::cout << std::endl;
	std::cout << std::endl;


	// //FOR THE ANULAR FINGER-------------------------------------------------------------------------
	IMAFinger anular_finger(2); 
	desired_orntn = {-2.9380,   -2.0796,    3.0447};		// Orientation wrt the base of the hand
	desired_pos = {0.0897, 0.07, 0.0382};	// Pos wrt the base of the hand
	std::cout << "#######ANULAR FINGER###### " <<std::endl;
	std::cout << "*******DEMO OF IKINE***** " <<std::endl;
	std::cout << "The desired position is: " << desired_pos[0] <<", "<< desired_pos[1] <<", " << desired_pos[2] << std::endl;
	std::cout << "The desired orientation in ZYZ eul angles is: " << desired_orntn[0] <<", "<< desired_orntn[1] <<", " << desired_orntn[2] << std::endl;
	ik_sols = anular_finger.Ikine(desired_orntn, desired_pos);
	// std::cout << "The joint values that were found are: " << ik_sols <<std::endl;
	q_values_finger = anular_finger.getJointAngles();
	std::cout << "q_values_finger after the IK is -------------: " <<std::endl;
	printVector(q_values_finger);
	std::cout << std::endl;

	std::cout << "*******DEMO OF FKINE***** " <<std::endl;
	fk_mat = anular_finger.Fkine(q_values_finger);
	TF_from_w2bfinger = anular_finger.getTF_w2b() ;
	// std::cout << "TF_from_w2bfinger is: " << TF_from_w2bfinger <<std::endl;
	tot_mat = TF_from_w2bfinger*fk_mat*anular_finger.getTF_tcp2tip();
	std::cout << "The FK matrix with the previously found IK angles, is: " <<std::endl<< tot_mat.matrix() << std::endl;

	new_quat_obtained = obtainQuaternion(tot_mat);
	std::cout << "The equivalent quaternion element 'w' is: " << new_quat_obtained.w() <<std::endl;
	std::cout << "The equivalent quaternion element 'vec' is: " << std::endl << new_quat_obtained.vec() <<std::endl;
	eul_angles_solved = obtainEulAngles(tot_mat);
	std::cout << "The equivalent in euler angles (ZYZ) is: " << std::endl << eul_angles_solved <<std::endl;
	translation_solved = obtainTranslation(tot_mat);
	std::cout << "The tanslation vector is: " << std::endl << translation_solved <<std::endl;
	std::cout << std::endl;
	std::cout << std::endl;

	return 0;
}
