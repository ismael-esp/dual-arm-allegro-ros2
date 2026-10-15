/*
This code is used for obtaining the limits of the joints
*/

#include <iostream>
// Include library to operate hand
#include <allegro_hand/AllegroHand.h>

#include <termios.h>
#include <cstdlib>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <thread>
#include <yaml-cpp/exceptions.h>
// // Function to read input without waiting for the enter key
// char getch() {
//     char buf = 0;
//     struct termios old;
//     struct termios newTermios;
//     fflush(stdout);
//     if(tcgetattr(0, &old) < 0)
//         perror("tcsetattr()");
//     newTermios = old;
//     newTermios.c_lflag &= ~ICANON;
//     newTermios.c_lflag &= ~ECHO;
//     newTermios.c_cc[VMIN] = 1;
//     newTermios.c_cc[VTIME] = 0;
//     if(tcsetattr(0, TCSANOW, &newTermios) < 0)
//         perror("tcsetattr ICANON");
//     if(read(0, &buf, 1) < 0)
//         perror ("read()");
//     if(tcsetattr(0, TCSADRAIN, &old) < 0)
//         perror ("tcsetattr ~ICANON");
//     return buf;
// }

AllegroHand *Allegro;


void continuousListening(AllegroHandDriver::Vector &current_configs, AllegroHandDriver::Vector &min_values, AllegroHandDriver::Vector &max_values, std::atomic<bool> &stopFlag, std::atomic<bool> &finishFlag) {
    while(!finishFlag){
        while (!stopFlag) {
            Allegro->getCurrentState(current_configs);
            // Update min and max values
            for (int i = 0; i < Allegro->n_fingers * Allegro->finger_n_dofs; ++i) {
                if (current_configs(i) < min_values(i)) {
                    min_values(i) = current_configs(i);
                }
                if (current_configs(i) > max_values(i)) {
                    max_values(i) = current_configs(i);
                }
            }
            // usleep(2000); // Sleep for 2 milliseconds
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

int main(int argc, char** argv) {
    // Ensure that the needed arguments are provided
    if (argc < 2){
        std::cout << "Syntax: example [yaml_file_configuration] [canport]" << std::endl;
        std::cout << "Please provide a YALM file as input" << std::endl;
        std::cout << "also, if the real hand is connected, the canport (ex. 32, 0, 1) [canport]" << std::endl;
        std::cout << "Ex: calc_min_max SAH030C035L" << std::endl;
        return 1;
    }

    
    int option;
    std::string Hand;
    unsigned int Channel = 32;

    AllegroHandDriver::Vector current_configs;
    std::atomic<bool> stopFlag(false);
    std::atomic<bool> finishFlag(false);
    std::string chain;

    system("clear");

    char wpath[512];
    getcwd(wpath, 512);
    std::cout << "Current path: " << wpath << std::endl;
    std::string path = wpath;
    std::string pcan = "/dev/pcan";

    std::cout << "Initializing hand object" << std::endl;

    if(argc == 3){
        Channel = atoi(argv[2]);
    }
    Allegro = new AllegroHand(argv[1],pcan, Channel);
    
    bool stop = false;

    std::cout << "Hand initialized" << std::endl;

    Allegro->getCurrentState(current_configs);
    // Initialize min and max values
    AllegroHandDriver::Vector min_values = current_configs;
    AllegroHandDriver::Vector max_values = current_configs;
    AllegroHandDriver::Vector ranges;
    std::thread listener(continuousListening, std::ref(current_configs), std::ref(min_values), std::ref(max_values), std::ref(stopFlag), std::ref(finishFlag));
    
    while(!stop){
        std::cout << std::endl;
        std::cout << "   UPC-ALLEGRO HAND APPLICATION" << std::endl;
        std::cout << std::endl;
        std::cout << "This program has been designed to store the minimum and the maximum of all the joints of the hands: " << std::endl;
        std::cout << "Once you are ready please press <1>, <2> and <3>. You should press them in order" << std::endl;
        std::cout << std::endl;
        std::cout << "<1> Start getting the minimum and maximum jointvalues (move all the joints to its maximum and minimum)" << std::endl;
        std::cout << "<2> Calculate the range, and print the values" << std::endl;
        std::cout << "<3> Store those values in a yaml file" << std::endl;
        std::cout << "<4> Print current configuration" << std::endl;
        std::cout << "<0> EXIT" << std::endl;
        std::cout<<std::endl;

        system("stty raw");
        option=getchar();
        system("stty cooked");
        std::cout << std::endl;

        switch (option){
        case '1':
        {
            
            std::cout << "*** listening ***" << std::endl;
            stopFlag = false;
            finishFlag = false;
            // char stop_key = 0;
            // while (stop_key != ' ') {
            //      std::cout << "New pose" << std::endl;
            //     Allegro->getCurrentState(current_configs);
            //     // Update min and max values
            //     for (int i = 0; i < Allegro->n_fingers * Allegro->finger_n_dofs; ++i) {
            //         if (current_configs(i) < min_values(i)) {
            //             min_values(i) = current_configs(i);
            //         }
            //         if (current_configs(i) > max_values(i)) {
            //             max_values(i) = current_configs(i);
            //         }
            //     }
            //     usleep(2000); // Sleep for 2 milliseconds

            //     if (std::cin.rdbuf()->in_avail() > 0) {
            //         std::cout << "ASDFADSFASDFADSFADSFASDFADSFASDFASDF";
            //         stop_key = getch();
            //         if (stop_key == ' ') {
            //             break;
            //         }
            //     }
            // }

            // char stop_key = '1';
            // do {
            //     std::cout << "New pose" << std::endl;
            //     Allegro->getCurrentState(current_configs);
            //     // Update min and max values
            //     for (int i = 0; i < Allegro->n_fingers * Allegro->finger_n_dofs; ++i) {
            //         if (current_configs(i) < min_values(i)) {
            //             min_values(i) = current_configs(i);
            //         }
            //         if (current_configs(i) > max_values(i)) {
            //             max_values(i) = current_configs(i);
            //         }
            //     }
            //     usleep(2000); // Sleep for 2 milliseconds
            //     stop_key = getchar();
            // } while (stop_key == '1');
            
            break;
        }
        case '2':
        {
            stopFlag = true;
            std::cout << "Listening has finished :)" << std::endl;
            std::cout << "Printing values" << std::endl;
            // Calculate the ranges:
            for (int i = 0; i < Allegro->n_fingers * Allegro->finger_n_dofs; ++i) {
                ranges(i) = abs(double(max_values(i)) - double(min_values(i)));
                // std::cout << "Joint " << i << ": " << min_values(i) << " to " << max_values(i) << std::endl;
            }
            //Print the values:
            for (int i = 0; i< Allegro->n_fingers; i++){
                std::cout << "Finger " << i << ": " << std::endl;
                for (int ii = 0; ii< Allegro->finger_n_dofs; ii++){
                    std::cout << "    " << "j" << i << ii << ": [ " << min_values(i*4+ii) << " , " << max_values(i*4+ii) << " ]. Range: " << ranges(i*4+ii)<< std::endl;
                }
                std::cout << std::endl;
            }
            break;
        }
        case '3':
        {
            std::cout << "Storing the values in a yaml file" << std::endl;
            stopFlag = true;
            try{
                YAML::Emitter out;
                out << YAML::BeginMap;

                // Iterate through the configurations
                for (int i = 0; i < 4; ++i) {
                    std::string finger_key = "Finger " + std::to_string(i);
                    out << YAML::Key << finger_key;
                    out << YAML::Value << YAML::BeginMap;
                    for (int j = 0; j < 4; ++j) {
                        std::string key = "j" + std::to_string(i) + std::to_string(j);

                        out << YAML::Key << key;
                        out << YAML::Value << YAML::BeginMap;

                        out << YAML::Key << "min_value";
                        out << YAML::Value << min_values(i * 4 + j);

                        out << YAML::Key << "max_value";
                        out << YAML::Value << max_values(i * 4 + j);

                        out << YAML::Key << "range";
                        out << YAML::Value << ranges(i * 4 + j);

                        out << YAML::EndMap;
                    }
                    out << YAML::EndMap;
                }

                out << YAML::EndMap;

                // // Create YAML document
                // YAML::Emitter out;
                // out << YAML::BeginMap;
                // out << YAML::Key << "min_values";
                // out << YAML::BeginSeq; // Start a sequence for each configuration. This is [
                // // Iterate through the values of the configuration.
                // for (const auto& min_val : min_values) {
                //     out << min_val; // Add each value to the sequence
                // }
                // out << YAML::EndSeq; // End the sequence for each configuration. This is ]
                // out << YAML::Key << "max_values";
                // out << YAML::BeginSeq; // Start a sequence for each configuration. This is [
                // // Iterate through the values of the configuration.
                // for (const auto& max_val : max_values) {
                //     out << max_val; // Add each value to the sequence
                // }
                // out << YAML::EndSeq; // End the sequence for each configuration. This is ]

                // out << YAML::Key << "ranges";
                // out << YAML::BeginSeq; // Start a sequence for each configuration. This is [
                // // Iterate through the values of the configuration.
                // for (const auto& rang_val : ranges) {
                //     out << rang_val; // Add each value to the sequence
                // }
                // out << YAML::EndSeq; // End the sequence for each configuration. This is ]
                // out << YAML::EndMap;

                // Write YAML to file
                std::ofstream fout(Allegro->directory_conf_folder + "joints_limits/" +Allegro->hand_name + ".yaml");
                fout << out.c_str();
                fout.close();

                std::cout << "Data saved to " << Allegro->directory_conf_folder << "joints_limits/" << Allegro->hand_name << ".yaml"<< std::endl;
            }catch(const YAML::ParserException& ex) {
                std::cout <<"File: ../conf/joints_limits/" << Allegro->hand_name << ".yaml" << std::endl;
                std::cout << ex.what() << std::endl;
                break;
            }
            break;
        }
        case '4':
        {
            Allegro->getCurrentState(current_configs);
            std::cout << "Storing the values in a yaml file" << std::endl;
            for (int i = 0; i< Allegro->n_fingers; i++){
                std::cout << "Finger " << i << ": " << std::endl;
                for (int ii = 0; ii< Allegro->finger_n_dofs; ii++){
                    std::cout << "    " << "j" << i << ii << ": " << current_configs(i*4+ii)<< std::endl;
                }
                std::cout << std::endl;
            }
            break;
        }
        
        case '0':
        {
            stop = true;
            stopFlag = true;
            finishFlag = true;
            listener.join();
            delete Allegro;
            std::cout << "*** Finished ***" << std::endl;
            break;
        }
        
        default:
            break;
        }

        
    }

    std::cout << "****** PROGRAM FINISHED!!! **********" << std::endl;
    // delete Allegro;
    listener.join();
    return 0;
}
