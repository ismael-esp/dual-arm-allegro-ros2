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
    unsigned int Channel = 32;

    AllegroHandDriver::Vector pos_des, pos, vel, acc, torque;

    std::string chain;

    system("clear");

    char wpath[512];
    getcwd(wpath, 512);
    std::cout << "Current path: " << wpath << std::endl;
    std::string path = wpath;
    std::string pcan = "/dev/pcan";
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
        std::cout << "<2> Get configuration list" << std::endl;
        std::cout << "<3> Move to defined configuration" << std::endl;
        std::cout << "<4> Move to configuration [ª]" << std::endl;
        std::cout << "<5> Read State" << std::endl;
        std::cout << "<6> Store a configuration" << std::endl;
        std::cout << "<7> Store current configuration" << std::endl;
        std::cout << "<8> Coupled Closing" << std::endl;
        std::cout << "<9> Set direct torque" << std::endl;
        std::cout << "<L> Set direct torque to finger" << std::endl;
        std::cout << "<C> Move to configuration [rad]" << std::endl;
        std::cout << "<F> Move to finger configuration [rad]" << std::endl;
        std::cout << "<G> Move to finger configuration [ª]" << std::endl;
        std::cout << "Defined Configs: <a>, <b>, <m>, <o>, <p>, <r>, <s>, <t>, <w>" << std::endl;
        std::cout << "<0> EXIT" << std::endl;
        std::cout<<std::endl;

        system("stty raw");
        option=getchar();
        system("stty cooked");
        std::cout << std::endl;

        switch (option){
        case '1':
        {
            // Set controller
            std::vector<std::string> controllers_names;
            Allegro->getControllersNames(true, controllers_names);

            unsigned int option;

            do {
                std::cout <<"Enter the number of the desired controller: " << std::endl;
                std::cin >> option;
            }while ((option != 0) && (option > controllers_names.size()));

            Allegro->setController(controllers_names.at(option-1));
            break;
        }
        case '2':
        {
           Allegro->getConfList(true);
           break;
        }
        case '3':
        {
            std::cout << std::endl << "Enter the configuration name" << std::endl;
            getline(std::cin,chain);
            Allegro->moveToConf(chain);
            break;
        }
        case '4':
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
            Allegro -> setTorqueController();
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
            std::cout << "Torque sent: " << std::endl;
            for(int i = 0; i<16; i++){
                std::cout << torque(i) << ", ";
            }
            std::cout << std::endl << "Torque unsatturated: " << std::endl;
            Allegro -> setTorque(torque);
            for(int i = 0; i<16; i++){
                std::cout << torque(i) << ", ";
            }
            std::cout << std::endl;
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
         case 'L':
         case 'l':
            {
                Allegro -> setTorqueController();
                std::vector<double> torque_finger;

                std::cout << std::endl << "Enter the torque of the joints from the finger (4) in Nm:" << std::endl;
                getline(std::cin,chain);
                std::istringstream iss(chain);
                std::string s;
                unsigned int i = 0;
                while (getline(iss, s,' ')) {
                    if (i >= AllegroHandDriver::finger_n_dofs) {
                        std::cout << "The number of torques can not be more than 4" << std::endl;
                        break;
                    }
                    try {
                        torque_finger.push_back(stod(s));
                        i++;
                    } catch(...) {
                        std::cout << "Invalid value" << std::endl;
                        break;
                    }
                }
                while (i < AllegroHandDriver::finger_n_dofs){
                    torque_finger.push_back(0.);
                    i++;
                }
                
                do {
                    std::cout <<"Enter the number of the desired finger: " << std::endl;
                    std::cout << "1 - Index finger." << std::endl;
                    std::cout << "2 - Middle finger." << std::endl;
                    std::cout << "3 - Annular finger." << std::endl;
                    std::cout << "4 - Thumb finger." << std::endl;
                    std::cin >> option;
                }while ((option <= 0) && (option > AllegroHandDriver::n_fingers));

                if (!Allegro->setTorque(torque_finger, option-1))
                    std::cout << "Torque out of range, ignored" << std::endl;
                break;
            }
        case 'C':
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
        case 'F':
        case 'f':
        {
            std::vector<double> pos_finger;

            std::cout << std::endl << "Enter the position of the joints of the finger (4) in rad:" << std::endl;
            getline(std::cin,chain);
            std::istringstream iss(chain);
            std::string s;
            unsigned int i = 0;
            while (getline(iss, s,' ')) {
                if (i >= AllegroHandDriver::finger_n_dofs) {
                    std::cout << "The number of positions can not be more than 4" << std::endl;
                    break;
                }
                try {
                    std::cout << "Value (" << i  << ") = " << stod(s) << "º = ";
                    pos_finger.push_back(stod(s));
                    std::cout << pos_finger [i] << " rad " ;
                    i++;
                } catch(...) {
                    std::cout << "Invalid value" << std::endl;
                    break;
                }
            }
            std::cout << std::endl;
            while (i < AllegroHandDriver::finger_n_dofs){
                pos_finger.push_back(0.);
                std::cout << "Value (" << i  << ") = "<< pos_finger[i] << " " ;
                i ++;
            }
 
            do {
                std::cout <<"Enter the number of the desired finger: " << std::endl;
                std::cout << "1 - Index finger." << std::endl;
                std::cout << "2 - Middle finger." << std::endl;
                std::cout << "3 - Annular finger." << std::endl;
                std::cout << "4 - Thumb finger." << std::endl;
                std::cin >> option;
            }while ((option <= 0) && (option > AllegroHandDriver::n_fingers));
            // set desired configuration does not check the controller used
            Allegro->setDesiredConfiguration(pos_finger, option-1);
            break;
        }
        case 'G':
        case 'g':
        {
            std::vector<double> pos_finger;

            std::cout << std::endl << "Enter the position of the joints of the finger (4) in rad:" << std::endl;
            getline(std::cin,chain);
            std::istringstream iss(chain);
            std::string s;
            unsigned int i = 0;
            while (getline(iss, s,' ')) {
                if (i >= AllegroHandDriver::finger_n_dofs) {
                    std::cout << "The number of positions can not be more than 4" << std::endl;
                    break;
                }
                try {
                    std::cout << "Value (" << i  << ") = " << stod(s) << "º = ";
                    pos_finger.push_back(M_PI/180.*stod(s));
                    std::cout << pos_finger [i] << " rad " ;
                    i++;
                } catch(...) {
                    std::cout << "Invalid value" << std::endl;
                    break;
                }
            }
            std::cout << std::endl;
            while (i < AllegroHandDriver::finger_n_dofs){
                pos_finger.push_back(0.);
                std::cout << "Value (" << i  << ") = "<< pos_finger[i] << " " ;
                i ++;
            }

            
            do {
                std::cout <<"Enter the number of the desired finger: " << std::endl;
                std::cout << "1 - Index finger." << std::endl;
                std::cout << "2 - Middle finger." << std::endl;
                std::cout << "3 - Annular finger." << std::endl;
                std::cout << "4 - Thumb finger." << std::endl;
                std::cin >> option;
            }while ((option <= 0) && (option > AllegroHandDriver::n_fingers));
            // set desired configuration does not check the controller used
            Allegro->setDesiredConfiguration(pos_finger, option-1);
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
        case 't':
        case 'T':
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
            break;
        }
    }

    std::cout << "PROGRAM FINISHED!!!!" << std::endl;
    // delete Allegro;

    return 0;
}
