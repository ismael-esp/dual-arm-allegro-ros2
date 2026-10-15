/*
This code is an example of the use of the AllegroHand library with the controllers
It allows to:
 - Comunicate with the hand.
 - Read the configurations of the hand.
 - Send the desired torque towards the hand
 - Set desired controller (gravity compensator, PD, PID)
 - Move to predefined configurations
 - Store configurations
 - Coupled closing
*/

#include <iostream>
// Include library to operate hand
#include <allegro_hand/AllegroHand.h>

#include <termios.h>
#include <cstdlib>
#include <boost/date_time/posix_time/posix_time.hpp>

int main(int argc, char** argv) {

    if (argc < 3){
        std::cout << "Syntax: example [path_yaml_file_configuration] [sim] [canport]" << std::endl;
        std::cout << "Please provide a YALM file as input" << std::endl;
        std::cout << "and optionally set if is a simulated hand [sim]" << std::endl;
        std::cout << "also, if the real hand is connected, the canport (ex. 32, 0, 1) [canport]" << std::endl;
        std::cout << "Ex: test_allegro_hand conf/SAH030C035L.yaml sim" << std::endl;
        std::cout << "where run the example in simulation without connection with the real hand" << std::endl;
        return 1;
    }

    AllegroHand *Allegro;
    int option;
    std::string Hand;
    unsigned int Channel =32;

    AllegroHandDriver::Vector pos_des, pos, vel, acc, torque, torques_gravity_compensator;

    std::string chain;

    system("clear");

    char wpath[512];
    getcwd(wpath, 512);
    std::cout << "Current path: " << wpath << std::endl;
    std::string path = wpath;
    std::string pcan = "";
    std::string simulation ("sim");

    std::cout << "Initializing hand object" << std::endl;

    if(argc == 4){
        Channel = atoi(argv[3]);
    }

    if(simulation.compare(argv[2]))
    {
        Allegro = new AllegroHand(argv[1],pcan, Channel);
    }
    else{
        Allegro = new AllegroHand(argv[1]);
    }
    bool stop = false;

    std::cout << "Hand initialized" << std::endl;

    while(!stop){
        std::cout << std::endl;
        std::cout << "   UPC-ALLEGRO HAND APPLICATION" << std::endl;
        std::cout << std::endl;
        std::cout << "Please select one option to continue: " << std::endl;
        std::cout << std::endl;
        std::cout << "<1> Set controller" << std::endl;
        std::cout << "<2> Compute FK" << std::endl;
        std::cout << "<3> Move fingertips to a pose. Compute IK" << std::endl;
        std::cout << "<4> Move to defined configuration" << std::endl;
        std::cout << "<5> Read State" << std::endl;
        std::cout << "<6> Store a configuration" << std::endl;
        std::cout << "<7> Store current configuration" << std::endl;
        std::cout << "<8> Coupled Closing" << std::endl;
        std::cout << "<9> Set direct torque" << std::endl;
        std::cout << "<c> Move to configuration [rad]" << std::endl;
        std::cout << "<C> Move to configuration [º]" << std::endl;
        std::cout << "<G> Get configuration list" << std::endl;
        std::cout << "<Q> Calculate Motions 1" << std::endl;
        std::cout << "<E> Calculate Motions 2" << std::endl;
        std::cout << "<X> Move to conf at desired velocity" << std::endl;
        std::cout << "<Y> Move to conf at desired velocity and store trajectory" << std::endl;
        std::cout << "<T> Move along a trajectory" << std::endl;
        std::cout << "<V> Move along a trajectory with desired velocity [rad/s]" << std::endl;
        std::cout << "<L> Listen hand and store the trajectory" << std::endl;
        std::cout << "<F> Move along a trajectory defined by poses using IK" << std::endl;
        std::cout << "<I> Execute grasp" << std::endl;
        std::cout << "<J> Get Jacobian (KDL)" << std::endl;
        std::cout << "<Z> Get Gravity Compensation Torques" << std::endl;
        std::cout << "Defined Configs: <a>, <b>, <h>, <m>, <o>, <p>, <r>, <s>, <u>, <w>" << std::endl;
        std::cout << "<0> EXIT" << std::endl;
        // Keyboards still free: d, k, n
        std::cout<<std::endl;

        system("stty raw");
        option=getchar();
        system("stty cooked");
        std::cout << std::endl;
        // std::cout << "Option: " << option << std::endl;
        switch (option){
        case '1':
        {
            // Set controller
            std::vector<std::string> controllers_names;
            Allegro->getControllersNames(true, controllers_names);
            unsigned int index;

            do {
                std::cout <<"Enter the number of the desired controller: " << std::endl;
                std::cin >> index;
            }while ((index != 0) && (index > controllers_names.size()));

            Allegro->setController(controllers_names.at(index-1));
            option=getchar();
            break;
        }
        case '2':
        {
            std::cout << std::endl << "Computing the Forward Kinematics of the current configuration (pose of fingertip)" << std::endl;
            Eigen::Matrix<double,3,4> eul_angles_solved;
            Eigen::Matrix<double,3,4> translation_solved;
            bool success;
            // std::vector<double> q_values_hand = {0.250382, 0.491335, 0.847031, 7.00478, 0.249956, 0.495335, 0.82151, 7.02634, 0.250312, 0.49129, 0.847118, 7.00473, 0.953121, 0.307612, 0.677417, 0.791276};
            // Allegro->computeFK(true, q_values_hand, eul_angles_solved, translation_solved);
            success = Allegro->computeFK(true, eul_angles_solved, translation_solved);
            if (success){
                // Print eul_angles_solved
                std::cout << "** Result: ** " << std::endl;
                std::cout << "Euler Angles:" << std::endl;
                for (int i = 0; i < eul_angles_solved.cols(); ++i) {
                    // std::cout << "Finger " << i << ": ";
                    for (int j = 0; j < eul_angles_solved.rows(); ++j) {
                        std::cout << eul_angles_solved(j, i) << " ";
                    }
                    // std::cout << std::endl;
                }
                // Print translation_solved
                std::cout << std::endl << "Positions (x,y,z):" << std::endl;
                for (int i = 0; i < translation_solved.cols(); ++i) {
                    // std::cout << "Finger " << i << ": ";
                    for (int j = 0; j < translation_solved.rows(); ++j) {
                        std::cout << translation_solved(j, i) << " ";
                    }
                    // std::cout << std::endl;
                }
            }
            break;
        }
        case '3':
        {
            std::cout << std::endl << "Computing the Inverse Kinematics" << std::endl;
            std::vector<double> desired_position(12);
            std::vector<double> desired_orientation(12);

            std::cout << std::endl << "Enter the desired position of each finger. X, Y, Z of the Index, Middle, Anular and Thumb (12)" << std::endl;
            getline(std::cin,chain);
            std::istringstream iss(chain);
            std::string s;
            unsigned int i = 0;
            while (getline(iss, s,' ')) {
                if (i >= AllegroHandDriver::n_fingers*3) {
                    std::cout << "The number of positions can not be more than 12" << std::endl;
                    break;
                    break;
                }
                try {
                    std::cout << "Value (" << i  << ") = ";
                    desired_position[i] = std::stod(s);
                    std::cout << desired_position[i] << " ";
                    i++;
                } catch(...) {
                    std::cout << "Invalid value" << std::endl;
                    break;
                }
            }

            std::cout << std::endl << std::endl << "Enter the desired orientation of each finger in euler angles. Z, Y, Z of the Index, Middle, Anular and Thumb (12) in rad" << std::endl;
            getline(std::cin,chain);
            std::istringstream isss(chain);
            i = 0;
            while (getline(isss, s,' ')) {
                if (i >= AllegroHandDriver::n_fingers*3) {
                    std::cout << "The number of positions can not be more than 12" << std::endl;
                    break;
                    break;
                }
                try {
                    std::cout << "Value (" << i  << ") = ";
                    desired_orientation[i] = stod(s);
                    std::cout << desired_orientation[i] << " ";
                    i++;
                } catch(...) {
                    std::cout << "Invalid value" << std::endl;
                    break;
                }
            }
            std::cout << std::endl;
            // Executes the inverse kinematics. The fingers move to the desired position
            Allegro->executeIK(true, desired_position, desired_orientation);

            break;
        }
        case '4':
        {
            std::cout << std::endl << "Enter the configuration name" << std::endl;
            getline(std::cin,chain);
            Allegro->moveToConf(chain);
            break;
        }

        case '5':
        {
            Allegro->getCurrentStatePrintDeg(pos,vel,acc,torque);
            break;
        }
        case '6':
        {   
            Allegro -> setGravityController();
            std::cout << std::endl << "Setting Gravity compensator controller. \n Please move the hand to the desired configuration." << std::endl;
            std::cout << std::endl << "Enter the configuration name:" << std::endl;
            getline(std::cin,chain);
            Allegro -> storeCurrentConf(chain, false);
            break;
        }
        case '7':
        {   
            std::cout << std::endl << "Enter the configuration name of the current configuration:" << std::endl;
            getline(std::cin,chain);
            Allegro -> storeCurrentConf(chain, false);
            break;
        }
        case '8':
        {
            Allegro -> coupledClosing();
            break;
        }
        case '9':
        {
            std::cout << std::endl << "Enter the torque of the joints from index to thumb (16) in Nm:" << std::endl;
            getline(std::cin,chain);
            std::istringstream iss(chain);
            std::string s;
            unsigned int i = 0;
            while (getline(iss, s,' ')) {
                if (i >= AllegroHandDriver::n_fingers*AllegroHandDriver::finger_n_dofs) {
                    std::cout << "The number of torques can not be more than 16" << std::endl;
                    break;
                }
                try {
                    torque(i++) = stod(s);
                } catch(...) {
                    std::cout << "Invalid value" << std::endl;
                    break;
                }
            }
            while (i < AllegroHandDriver::n_fingers*AllegroHandDriver::finger_n_dofs){
                torque(i++) = 0.;
            }
            Allegro -> setTorque(torque);
            break;
        }
        // case '9':
        // {

        //     struct termios initial_settings, new_settings;

        //     tcgetattr(0,&initial_settings);

        //     new_settings = initial_settings;
        //     new_settings.c_lflag &= ~ICANON;
        //     new_settings.c_lflag &= ~ECHO;
        //     new_settings.c_lflag &= ~ISIG;
        //     new_settings.c_cc[VMIN] = 0;
        //     new_settings.c_cc[VTIME] = 0;

        //     tcsetattr(0, TCSANOW, &new_settings);
        //     int n = 0;
        //     std::string fingerNames[] = {"Index","Middle","Ring","Thumb"};
        //     while (n != 27) {
        //         n = getchar();

        //         Allegro->getCurrentState(pos,vel,acc,torque);
        //         system("clear");
        //         std::cout << "Joint values in degrees (press ESC to exit)" << std::endl;
        //         for (unsigned int f = 0; f < 4; ++f) {
        //             std::cout << fingerNames[f] << ":";
        //             for (unsigned int i = 0; i < 4; ++i) {
        //                 std::cout << "\t" << std::fixed << std::setprecision(2) << std::right << std::setw(8) << 180./M_PI*pos(i+4*f);
        //             }
        //             std::cout << std::endl;
        //         }
        //         usleep(0.5e6);
        //     }

        //     tcsetattr(0, TCSANOW, &initial_settings);

        //     break;
        // }
        
        
        // case 'Z':
        // case 'z':
        // {
        //     int finger_number; 
        //     std::vector<double> desired_position(3);
        //     std::vector<double> desired_normal(3);
        //     do {
        //         std::cout <<"Enter the desired finger: 1-Index, 2-Middle, 3-Annular, 4-Thumb" << std::endl;
        //         std::cin >> finger_number;
        //     }while (finger_number <= 0 && finger_number >5);
        //     getline(std::cin,chain);
        //     std::cout << std::endl << "Computing the Inverse Kinematics" << std::endl;
        //     std::cout << std::endl << "Enter the desired position of the finger. X, Y, Z." << std::endl;
        //     getline(std::cin,chain);
        //     std::istringstream iss(chain);
        //     std::string s;
        //     unsigned int i = 0;
        //     while (getline(iss, s,' ')) {
        //         if (i >= 3) {
        //             std::cout << "The number of positions can not be more than 3" << std::endl;
        //             break;
        //             break;
        //         }
        //         try {
        //             std::cout << "Value (" << i  << ") = ";
        //             desired_position[i] = std::stod(s);
        //             std::cout << desired_position[i] << " ";
        //             i++;
        //         } catch(...) {
        //             std::cout << "Invalid value" << std::endl;
        //             break;
        //         }
        //     }

        //     std::cout << std::endl << std::endl << "Enter the desired normal of the finger" << std::endl;
        //     getline(std::cin,chain);
        //     std::istringstream isss(chain);
        //     i = 0;
        //     while (getline(isss, s,' ')) {
        //         if (i >= 3) {
        //             std::cout << "The number of positions can not be more than 3" << std::endl;
        //             break;
        //             break;
        //         }
        //         try {
        //             std::cout << "Value (" << i  << ") = ";
        //             desired_normal[i] = stod(s);
        //             std::cout << desired_normal[i] << " ";
        //             i++;
        //         } catch(...) {
        //             std::cout << "Invalid value" << std::endl;
        //             break;
        //         }
        //     }
        //     std::cout << std::endl;
        //     Allegro->computeIKFinger(true, finger_number-1, desired_position, desired_normal);
        //     break;
        // }


        case 'G':
        case 'g':
        {
           Allegro->getConfList(true);
           break;
        }
        case 'c':
        {
            std::cout << std::endl << "Enter the hand configuration from index to thumb (16) in rad:" << std::endl;
            getline(std::cin,chain);
            std::istringstream iss(chain);
            std::string s;
            unsigned int i = 0;
            while (getline(iss, s,' ')) {
                if (i >= AllegroHandDriver::n_fingers*AllegroHandDriver::finger_n_dofs) {
                    std::cout << "The number of positions can not be more than 16" << std::endl;
                    break;
                    break;
                }
                try {
                    std::cout << "Value (" << i  << ") = " << stod(s) << "º = ";
                    pos_des(i) = stod(s);
                    std::cout << pos_des (i) << " rad " ;
                    i++;
                } catch(...) {
                    std::cout << "Invalid value" << std::endl;
                    break;
                }
            }
            std::cout << std::endl;
            while (i < AllegroHandDriver::n_fingers*AllegroHandDriver::finger_n_dofs){
                pos_des(i) = 0.;
                std::cout << "Value (" << i  << ") = "<< pos_des (i) << " " ;
                i ++;
            }
            Allegro->moveToConf(pos_des);


            break;
        }
        case 'C':
        {
            std::cout << std::endl << "Enter the hand configuration from index to thumb (16) in degrees:" << std::endl;
            getline(std::cin,chain);
            std::istringstream iss(chain);
            std::string s;
            unsigned int i = 0;
            while (getline(iss, s,' ')) {
                if (i >= AllegroHandDriver::n_fingers*AllegroHandDriver::finger_n_dofs) {
                    std::cout << "The number of positions can not be more than 16" << std::endl;
                    break;
                    break;
                }
                try {
                    std::cout << "Value (" << i  << ") = " << stod(s) << "º = ";
                    pos_des(i) = M_PI/180.*stod(s);
                    std::cout << pos_des (i) << " rad " ;
                    i++;
                } catch(...) {
                    std::cout << "Invalid value" << std::endl;
                    break;
                }
            }
            std::cout << std::endl;
            while (i < AllegroHandDriver::n_fingers*AllegroHandDriver::finger_n_dofs){
                pos_des(i) = 0.;
                std::cout << "Value (" << i  << ") = "<< pos_des (i) << " " ;
                i ++;
            }
            Allegro->moveToConf(pos_des);


            break;
        }
        case 'q':
        case 'Q':
        {
            std::vector<double> desired_pos = {-2.8480,   -2.0365,   -3.0469, -2.8916,  -2.0600,   -3.1416, -2.9380,   -2.0796,    3.0447};
            std::vector<double> desired_orntn = {0.0897, -0.0243, 0.0422, 0.0897, 0.0229, 0.0426, 0.0897, 0.07, 0.0382};
            Allegro->calculeMotionHand(1,desired_pos, desired_orntn);
            break;
        }
        case 'e':
        case 'E':
        {
            std::vector<double> desired_pos = {-2.8480,   -2.0365,   -3.0469, -2.8916,  -2.0600,   -3.1416, -2.9380,   -2.0796,    3.0447};
            std::vector<double> desired_orntn = {0.0897, -0.0243, 0.0422, 0.0897, 0.0229, 0.0426, 0.0897, 0.07, 0.0382};
            Allegro->calculeMotionHand(2,desired_pos, desired_orntn);
            break;
        }
        case 'x':
        case 'X':
        {
            unsigned int velocity;
            do {
                std::cout <<"Enter the desired velocity percentage [up to 100%] " << std::endl;
                std::cin >> velocity;
            }while (velocity == 0);
            getline(std::cin,chain);
            std::cout << std::endl << "Enter the configuration name" << std::endl;
            getline(std::cin,chain);
            AllegroHandDriver::Vector configuration_values;
            Allegro->getPredefinedConfigurationValues(chain, configuration_values);
            double vel = static_cast<double>(velocity)/100;
            Allegro->moveToConf(configuration_values, vel);
            break;
        }
        case 'y':
        case 'Y':
        {
            unsigned int velocity;
            do {
                std::cout <<"Enter the desired velocity percentage [up to 100%] " << std::endl;
                std::cin >> velocity;
            }while (velocity == 0);
            getline(std::cin,chain);
            std::cout << std::endl << "Enter the configuration name" << std::endl;
            getline(std::cin,chain);
            AllegroHandDriver::Vector configuration_values;
            Allegro->getPredefinedConfigurationValues(chain, configuration_values);
            double vel = static_cast<double>(velocity)/100;
            std::cout << std::endl << "Enter the name of the new trajectories file" << std::endl;
            getline(std::cin,chain);
            Allegro->moveToConfAndSaveTrajectory(configuration_values, vel, chain);
            break;
        }
        case 't':
        case 'T':
        {
            Allegro->getTrajectoriesListNames();
            std::cout << std::endl << "Enter the trajectory name" << std::endl;
            getline(std::cin,chain);
            Allegro->moveAlongTrajectory(chain);
            break;
        }
        case 'l':
        case 'L':
        {
            unsigned int listening_time;
            do {
                std::cout <<"Enter the recording time " << std::endl;
                std::cin >> listening_time;
            }while (listening_time == 0);
            getline(std::cin,chain);
            std::cout << std::endl << "Enter the name of the new trajectories file" << std::endl;
            getline(std::cin,chain);
            std::vector<AllegroHandDriver::Vector> trajectory;
            Allegro->storeCurrentTrajectory(chain, listening_time, trajectory);
            break;
        }
        // case 'k':
        // case 'K':
        // {
        //     unsigned int listening_time;
        //     do {
        //         std::cout <<"Enter the desired velocity percentage [up to 100%] " << std::endl;
        //         std::cin >> listening_time;
        //     }while (listening_time == 0);
        //     getline(std::cin,chain);
        //     std::cout << std::endl << "Enter the name of the new trajectories file" << std::endl;
        //     getline(std::cin,chain);
        //     std::vector<AllegroHandDriver::Vector> trajectory;
        //     Allegro->storeCurrentTrajectory2(chain, listening_time, trajectory);
        //     break;
        // }
        case 'v':
        case 'V':
        {
            double velocity;
            // do {
            //     std::cout <<"Enter the desired velocity in rad/s " << std::endl;
            //     std::cin >> velocity;
            // }while (velocity == 0);
            do {
                std::cout << "Enter the desired velocity in rad/s: ";
                std::cin >> velocity;
                if (std::cin.fail() || velocity <= 0) {
                    std::cin.clear(); // Clear the error flag
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Ignore the invalid input
                    std::cout << "Invalid input. Please enter a positive decimal number for the velocity." << std::endl;
                    velocity = 0; // Ensure the loop continues if invalid input was given
                }
            } while (velocity <= 0);
            getline(std::cin,chain);
            Allegro->getTrajectoriesListNames();
            std::cout << std::endl << "Enter the trajectory name" << std::endl;
            getline(std::cin,chain);
            Allegro->moveAlongTrajectory(chain, velocity);
            break;
        }
        case 'f':
        case 'F':
        {
            Allegro->getTrajectoriesPosesListNames();
            std::cout << std::endl << "Enter the trajectory name" << std::endl;
            getline(std::cin,chain);
            Allegro->moveAlongPosesTrajectory(chain);
            break;
        }
        case 'i':
        case 'I':
        {
            unsigned int velocity;
            do {
                std::cout <<"Enter the desired velocity percentage [up to 100%] " << std::endl;
                std::cin >> velocity;
            }while (velocity == 0);
            double vel = static_cast<double>(velocity)/100;

            unsigned int option;
            do {
                std::cout << "Enter the number of the desired grasp:" << std::endl;
                std::cout << "1 - Pinch grasp with Index and Thumb fingers." << std::endl;
                std::cout << "2 - Pinch grasp with Middle and Thumb fingers." << std::endl;
                std::cout << "3 - Prismatic grasp with 3 fingers." << std::endl;
                std::cout << "4 - Prismatic grasp with 4 fingers" << std::endl;
                std::cin >> option;
            }while ((option != 0) && (option > 4));
            switch(option){
                case 1:
                {
                    Allegro->moveGraspPinchIndexThumb(vel);
                    break;
                }
                case 2:
                {
                    Allegro->moveGraspPinchMiddleThumb(vel);
                    break;
                }
                case 3:
                {
                    Allegro->moveGraspPrismatic3(vel);
                    break;
                }
                case 4:
                {
                    Allegro->moveGraspPrismatic4(vel);
                    break;
                }
                default:{
                    std::cout << "Error selecting grasp: " << option << std::endl;
                    break;
                }

            }
            break;
        }
        case 'j':
        case 'J':
        {
            int index;
            do {
                std::cout << "Enter the index of the finger of the desired Jacobian (KDL)" << std::endl;
                std::cout << "1 - Index" << std::endl;
                std::cout << "2 - Middle" << std::endl;
                std::cout << "3 - Annular" << std::endl;
                std::cout << "4 - Thumb" << std::endl;
                std::cout << "5 - All" << std::endl;
                std::cin >> index;
            }while ((index != 0) && (index > 5));
            // do {
            //     std::cout << "Introduce cog_x" << std::endl;
            //     std::cin >> index;
            // }while ((index != 0) && (index > 4));
            std::vector<double> joints;
            Eigen::MatrixXd jacobian_matrix;
            joints.resize(4 * 4);
            std::vector<double> cog_x = std::vector<double>(4,0.);
            std::vector<double> cog_y = std::vector<double>(4,0.);
            Allegro->getCurrentState(pos,vel,acc,torque);
            for (int dof=0; dof < 16; dof++){
                joints[dof]=pos(dof);
            }
            Allegro->ComputeJacobian(joints, cog_x, cog_y);
            if(index == 5){
                for (unsigned int i = 0; i < 4; i++){
                    jacobian_matrix = Allegro->getRealJacobianKDLAtIndex(i);
                    std::cout << "- Jacobian Matrix Finger " << i << ": "<< std::endl;
                    std::cout << jacobian_matrix << std::endl<< std::endl;
                }
            }
            else{
                jacobian_matrix = Allegro->getRealJacobianKDLAtIndex(index-1);
                std::cout << "Jacobian Matrix:" << std::endl;
                std::cout << jacobian_matrix << std::endl;
            }
            
            option=getchar();
            break;
        }
        case 'z':
        case 'Z':
        {
            Allegro->getCurrentState(pos,vel,acc,torque);
            Allegro->getGravityComp(torques_gravity_compensator);
            
            std::cout << "Current hand torque [Nm]:" << std::endl << torque.transpose() << std::endl;
            std::cout << "Current torques related to compensate the gravity effects are: " << std::endl << torques_gravity_compensator.transpose() << std::endl;
            
            std::cout << "Difference between the torques applied and the gravity compensation torques is: " << std::endl << (torque - torques_gravity_compensator).transpose()  << std::endl;
            break;

        }
        case '0':
        {
            stop = true;
            delete Allegro;
            break;
        }
        case 'r':
        case 'R':
        {
            Allegro->moveToRockConf();
            break;
        }
        case 'm':
        case 'M':
        {
            Allegro->moveToPaperConf();
            break;
        }
        case 's':
        case 'S':
        {
            Allegro->moveToScissorsConf();
            break;
        }
        case 'a':
        case 'A':
        {
            Allegro->moveToAlohaConf();
            break;
        }
        case 'p':
        case 'P':
        {
            Allegro->moveToPunkConf();
            break;
        }
        case 'w':
        case 'W':
        {
            Allegro->moveToWinConf();
            break;
        }
        case 'b':
        case 'B':
        {
            Allegro->moveToBotifarraConf();
            break;
        }
        case 'u':
        case 'U':
        {
            Allegro->moveToThumbUpConf();
            break;
        }
        case 'o':
        case 'O':
        {
            Allegro->moveToOneConf();
            break;
        }
        case 'h':
        case 'H':
        {
            Allegro->moveToHomeConf();
            break;
        }
        default:
        std::cout << "Default option" << std::endl;
            break;
        }
    }

    std::cout << "PROGRAM FINISHED!!!!" << std::endl;
    // delete Allegro;

    return 0;
}
