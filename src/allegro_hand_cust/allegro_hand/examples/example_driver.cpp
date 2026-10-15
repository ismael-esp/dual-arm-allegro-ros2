/*
This code is an example of the use of the AllegroHandDriver.
It allows to:
 - Comunicate with the hand.
 - Read the configurations of the hand.
 - Send the desired torque towards the hand
*/
#include <iostream>
// Include library to operate hand
#include <allegro_hand/AllegroHandDriver.h>

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

    AllegroHandDriver *Allegro;
    int option;
    std::string Hand;
    unsigned int Channel = 32;

    AllegroHandDriver::Vector q, torque;
    std::vector<double> torque_max;
    double torqueConv;
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
        Allegro = new AllegroHandDriver(true, argv[1],pcan, Channel);
    }
    else{
        Allegro = new AllegroHandDriver(argv[1]);
    }
    bool stop = false;

    std::cout << "Hand driver initialized" << std::endl;

    while(!stop){
        std::cout << std::endl;
        std::cout << "   UPC-ALLEGRO HAND APPLICATION" << std::endl;
        std::cout << std::endl;
        std::cout << "Please select one option to continue: " << std::endl;
        std::cout << std::endl;
        std::cout << "<1> Set hand torque" << std::endl;
        std::cout << "<2> Set finger torque" << std::endl;
        std::cout << "<3> Get current hand configuration" << std::endl;
        std::cout << "<4> Get Torque Conversion" << std::endl;
        std::cout << "<5> Get Torque Limits" << std::endl;
        std::cout << "<6> Get Torque" << std::endl;
        std::cout << "<0> EXIT" << std::endl;
        std::cout<<std::endl;

        system("stty raw");
        option=getchar();
        system("stty cooked");
        std::cout << std::endl;

        switch (option){
            case '1':
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

                if (!Allegro->setTorque(torque))
                    std::cout << "Torque out of range, ignored" << std::endl;

                break;
            }
            case '2':
            {
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
            case '3':
            {
                Allegro->getCurrentStateLoop(q);
                std::cout << "Current hand position in degrees:" << std::endl;
                for (int i = 0; i<=3; i++){
                    std::cout << "Finger " << i << ": ";
                    for (int ii = 0; ii<=3; ii++){
                        std::cout << "j" << i << ii << ": " << 180./M_PI*q(i*4+ii) << ", ";
                    }
                    std::cout << std::endl;
                }
                //std::cout << "Current hand torque in Nm:" << std::endl << torque.transpose() << std::endl;
                break;
            }
            case '4':
            {   
                torqueConv = Allegro->getTorqueConversion();
                std::cout << "The torque conversion is: " << torqueConv << std::endl;
                break;
            }
            case '5':
            {
                Allegro->getEffortLimits(torque_max);
                std::cout << "The torque max is: " << std::endl;
                for (int i = 0; i<=3; i++){
                    std::cout << "Finger " << i << ": ";
                    for (int ii = 0; ii<=3; ii++){
                        std::cout << "j" << i << ii << ": " << torque_max[i*4+ii] << ", ";
                    }
                    std::cout << std::endl;
                }
                break;
            }
            case '6':
            {
                AllegroHandDriver::Vector torque_;
                Allegro -> getTorque(torque_);
                std::cout << "The torque sent is: " << torque_.transpose() << std::endl;
                break;
            }
            case '0':
            {
                stop = true;
                Allegro->stopHand();
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
