/***************************************************************************
 *   Copyright (C) 2021 by Andres Montano                                  *
 *   andres.felipe.montano@upc.edu                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 ***************************************************************************/

#include <allegro_hand/AllegroHandDriver.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
#include <filesystem>
#include <iterator>
#include <yaml-cpp/exceptions.h>
#include <Eigen/QR>
#include <ctime>
#include <iostream>   // std::cout, std::ios
#include <sstream>    // std::ostringstream
#include <fstream>    //std::ofstream

#include <boost/signals2/signal.hpp> // Signal library
#include <boost/asio.hpp>


//! \brief Sign template function
//! from https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
//! \param val int, float or double
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

//! \brief Constructor Default constructor. Reads configuration file, Opens CAN communication and if _driver_loop creates a loop that continuously reads the hand
//! \param _conf_file Hand configuration file. Can be the directory or the filename (with or without the .yaml extension)
//! \param _pcan_port Pcan port
//! \param _channel Communications channel
//! \param driver_loop boolean to decide wether to create a thread inside the driver to read the configurations constantly and write the torque
AllegroHandDriver::AllegroHandDriver(bool _driver_loop, std::string _conf_file, std::string _pcan_port, unsigned int _channel)
    : torque(Vector::Zero()),
      torque_desired(Vector::Zero()),
      device(nullptr),
      driver_loop(_driver_loop),
      status_driver_loop(true)
    {
    
    setUpAHD(_conf_file, _pcan_port, _channel);
}

 //! \brief Constructor Default constructor. Simulated hand.
 //! \param _conf_file Hand configuration file. Can be the directory or the filename (with or without the .yaml extension)
AllegroHandDriver::AllegroHandDriver(std::string _conf_file)
    : torque(Vector::Zero()),
    torque_desired(Vector::Zero()),
      device(nullptr),
       //pcan_port("sim"),
       //channel(0),
       driver_loop(false)
    {
    setUpAHD(_conf_file);
 }

//! \brief Empty constructor
AllegroHandDriver::AllegroHandDriver()
    : torque(Vector::Zero()),
      torque_desired(Vector::Zero()),
      device(nullptr),
      driver_loop(false)
    {}

//! \brief Set up function for the constructor of the real hand. Reads configuration file, Opens CAN communication and if _driver_loop creates a loop that continuously reads the hand
//! \param _conf_file Hand configuration file. Can be the directory or the filename (with or without the .yaml extension)
//! \param _pcan_port Pcan port
//! \param _channel Communications channel
//! \param driver_loop boolean to decide wether to create a thread inside the driver to read the configurations constantly and write the torque
void AllegroHandDriver::setUpAHD(std::string _conf_file, std::string _pcan_port, unsigned int _channel){
    simulated = false;
    // Initialize the torque_max
    for (unsigned int i = 0; i < n_fingers * finger_n_dofs; ++i) {
        torque_max.push_back(0.7);
    }
    // Obtain the path of the conf folder
    getConfDirectory(_conf_file);
    //  Read the configuration file of the hand. We need some parameters for the CAN.
    if(readConfigurationFile(directory_configuration_file)){
        std::cout << "AllegroHandDriver: Configuration file has been succesfully read" << std::endl;
    }
    // Initialize CAN device. Starts CAN communications
    device = new CAN_AH(_pcan_port + std::to_string(_channel), params);
    std::cout << "Starting init for Hand " << params.serial << " on the pcan port/channel" << _pcan_port << _channel << std::endl;

    std::cout << "Real Hand" << std::endl;
    std::cout << "AllegroHandDriver object initialization finished" << std::endl;
    
    // Start driver loop. 
    // The hand needs a loop to constantly read and write to the hand. Otherwise it stops refreshing and nor the read nor the write works properly.
    if (driver_loop){
        // Starting driver loop. 
        std::cout << "Starting driver thread" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        driverThread = std::thread(&AllegroHandDriver::driverLoop, this);
    }
}

//! \brief Set up function for the constructor of the simmulated hand. Reads configuration file, Opens CAN communication and if _driver_loop creates a loop that continuously reads the hand
//! \param _conf_file Hand configuration file. Can be the directory or the filename (with or without the .yaml extension)
void AllegroHandDriver::setUpAHD(std::string _conf_file){
    simulated = true;
    // Initialize the torque_max
    for (unsigned int i = 0; i < n_fingers * finger_n_dofs; ++i) {
        torque_max.push_back(0.7);
    }
    // Obtain the path of the conf folder
    getConfDirectory(_conf_file);
    //  Read the configuration file of the hand. We need some parameters for the CAN.
    if(readConfigurationFile(directory_configuration_file)){
        std::cout << "AllegroHandDriver: Configuration file has been succesfully read" << std::endl;
    }
    std::cout << "Simulated Hand" << std::endl;
    std::cout << "AllegroHandDriver object initialization finished" << std::endl;
}
//! \brief Default destructor
AllegroHandDriver::~AllegroHandDriver() {
    stopHand();
    // Free memory up
    delete device;
}

//! \brief StopHand Stops the communications and contorl threads
void AllegroHandDriver::stopHand() {
    //Send zero torque to the hand
    Vector tau_ = Vector::Zero();
    setTorque(tau_);
    // Stop the driverLoop
    if(driver_loop){
        mutex.lock();
            status_driver_loop = false;
        mutex.unlock();
        driverThread.join();
    }
}


//! \brief Loop to continuously read the current state and write the desired torque. Only used if the AllegroHand class is not runned
void AllegroHandDriver::driverLoop(){
    bool status_driver_loop_;
    Vector torque_;
    // Initialize status
    mutex.lock();
        status_driver_loop_ = status_driver_loop;
    mutex.unlock();
    while(status_driver_loop_){
        // Read the current configuration from the hand
        getCurrentStateComm(q);
        mutex.lock();
            status_driver_loop_ = status_driver_loop;
        mutex.unlock();

        mutex_torque.lock();
            torque = torque_desired;
            torque_ = torque_desired;
        mutex_torque.unlock();

        // Send the torque to the Communications class
        device->setTorque(torque_.data());
        // Send the message with the desired torque to the hand 
        device->writeDevices(); 
    }
}

//! \brief Gets the current configuration values when the driver loop is applied
//! \param q_ reference variable to store the configuration values
void AllegroHandDriver::getCurrentStateLoop(Vector &q_) {
    // if(driver_loop){
    mutex.lock();
        q_ = q;
    mutex.unlock();
    // }
    // else{
    //     std::cout << "Driver Loop not applied. Using getCurrentStateComm() instead"
    //     getCurrentStateComm(Vector &q_);
    // }
}

//! \brief Gets the current configuration values by reading the messages. Use when driver loop not used
//! WARNING: We need to define a loop on an upper level to read continuously for this function to work properly
//! \param q_ reference variable to store the configuration values
void AllegroHandDriver::getCurrentStateComm(Vector &q_) {
    Vector q_current;
    TPCANTimestamp timeStampMsg_;
    // TODO: Asses if it is better to do the readDataTimeStamp without returning the TimeStamp data without the timestamp. 
    // Information: In the previous version of the code (3/2024) for controlling the time here instead of setting the Allegro Hand to periodic is done and commented 
    // TODO: Change name to read devices so it is understood that readDataTimeStamp gets the message, readDevices parses the message
    device->readDataTimeStamp(timeStampMsg_); // Obtain the data and the timestamp of the data
    device->readDevices(); // Parse the data and store the position
    device->getPosition(q_current.data()); // Obtain the position
    mutex.lock();
        q_ = q_current;
    mutex.unlock();
}

//! \brief Ensures that the desired torque is within the limits. If nots it sets it accordingly.
//! \param torque_ Desired torque values
bool AllegroHandDriver::setDesiredTorque(Vector &torque_) {
    // Check if q_des_ is finite and inside the limits
    for (unsigned int i = 0; i < n_fingers * finger_n_dofs; ++i) {
        if (isfinite(torque_(i))){
            if(fabs(torque_(i))>=torque_max[i]){
                //need to check the sign
                // bound to max
                torque_(i) = sgn(torque_(i)) * torque_max[i];
            }
        }
        else{
            std::cerr << "Torque " << i << " is not finite" << std::endl;
            return false;
        }
    }
    return true;
}

//! \brief Set the desired torque of the hand. It checks that is within the limits. (The torque will be send in the control loop if it has one)
//! \param torque_ Desired torque values
bool AllegroHandDriver::setTorque(Vector &torque_){
    bool success;
    // Check/Adjust the torque wiwthin the actual limits
    success = setDesiredTorque(torque_);
    if (!success){
        std::cout << "Torque out of range, ignored" << std::endl;
    }
    // Store the torque sent. (If within limits is the same as the given)
    mutex_torque.lock();
        torque_desired = torque_;
    mutex_torque.unlock();
    return success;
}

//! \brief Ensures that the desired finger torque is within the limits. If nots it sets it accordingly.
//! \param torque_ Desired finger torque values
//! \param finger_index Finger index (0:Index 1:Middle 2:Annular 3:Thumb)
bool AllegroHandDriver::setDesiredTorque(std::vector<double> &torque_finger_desired, int finger_index) {
    if (finger_index < 0 || finger_index >= n_fingers) {
        std::cerr << "Invalid finger index: " << finger_index << std::endl;
        return false;
    }

    if (torque_finger_desired.size() != finger_n_dofs) {
        std::cerr << "Invalid size for torque_finger_desired: " << torque_finger_desired.size() << std::endl;
        return false;
    }

    bool success = true;
    int start_index = finger_index * finger_n_dofs;
    std::vector<double> torque_finger_corrected = torque_finger_desired;

    for (int i = 0; i < finger_n_dofs; ++i) {
        int index = start_index + i;
        if (std::isfinite(torque_finger_corrected[i])) {
            if (fabs(torque_finger_corrected[i]) > torque_max[index]) {
                torque_finger_corrected[i] = (torque_finger_corrected[i] > 0 ? 1 : -1) * torque_max[index];
            }
        } else {
            std::cerr << "Torque " << index << " is not finite" << std::endl;
            success = false;
        }
    }
    if(success){
        torque_finger_desired = torque_finger_corrected;    
    }
    return success;
}

//! \brief Set the desired torque of the finger. It checks that is within the limits. (The torque will be send in the control loop if it has one)
//! \param torque_ Desired torque values
//! \param finger_index Finger index (0:Index 1:Middle 2:Annular 3:Thumb)
bool AllegroHandDriver::setTorque(std::vector<double> &torque_finger_desired, int finger_index) {
    bool success = setDesiredTorque(torque_finger_desired, finger_index);
    int start_index = finger_index * finger_n_dofs;
    if (!success) {
        std::cout << "Torque out of range, ignored" << std::endl;
    }
    else{
        AllegroHandDriver::Vector torque_;
        mutex_torque.lock();
            torque_ = torque;
        mutex_torque.unlock();
        for (int i = 0; i < finger_n_dofs; ++i) {
            torque_(start_index + i) = torque_finger_desired[i];
        }
        mutex_torque.lock();
            torque_desired = torque_;
        mutex_torque.unlock();
    }
    return success;
}

//! \brief Set and send directly the desired torque of the hand. It checks that is within the limits.
//! \param torque_ Desired torque values
void AllegroHandDriver::sendTorque(Vector &torque_){
    setTorque(torque_);
    // Send the torque to the Communications class
    device->setTorque(torque_.data());
    // Send the message with the desired torque to the hand 
    device->writeDevices(); 
}

//! \brief Obtain the directory of the /conf folder and the hand name (obtained from the filename)
//! \param yaml_conf_file string that can either contain the hand name (with or without extension) or the absolute path of the conf file (located in the conf folder)
//! REMINDER: If yaml_conf_file is the path, then name of the folders DO NOT matter (conf and configuration_files)
//! WARNING: If yaml_conf_file is the hand version name, then name of the folder names DO matter (conf and configuration_files)
void AllegroHandDriver::getConfDirectory(std::string yaml_conf_file){
       
    // The param yaml_conf_file can either be the filename (hand name) or the path to the configuration file
    // Check if yaml_conf_file has a "/". 
    bool is_a_file_path = (yaml_conf_file.find('/') != std::string::npos);
    // If so the yaml_conf_file is a path.
    //! REMINDER: The name of the folders do not matter (conf and configuration_files folders)
    if(is_a_file_path){
        // Path of the configuration file. Example: ../conf/configuration_files/V4R284.yaml
        std::filesystem::path path_conf_file = yaml_conf_file;
        // Absolute path of the configuration file. Example: ~/..../conf/configuration_files/V4R284.yaml
        std::filesystem::path directory = std::filesystem::absolute(path_conf_file);
        // Save absolute path of the configuration file as a string into the class variable: directory_configuration_file
        directory_configuration_file = directory;
        // Remove the filename to the absolute path to obtain the directory of the conf folder (that contains the yaml_conf_file)
        directory.remove_filename(); //Example: ~/..../conf/configuration_files/
        // Remove the folder where the configuration file is stored. One parent_path removes last / and the other removes the /configuration_files
        directory = directory.parent_path().parent_path(); //Example: ~/..../conf;
        // Save the absolute path of te conf file as a string into the class variable: directory_conf_folder. 
        directory_conf_folder = directory;  //Example: ~/..../conf;
        directory_conf_folder += '/';       //Example: ~/..../conf/;
        std::cout << "directory: " << directory << std::endl;
        // Extract the filename (without the .yaml) to obtain the hand name and save it as a string into the class variable: hand_name
        hand_name = path_conf_file.stem(); //Example: V4R284
    }
    // Otherwise it is the filename with no extension = hand name.
    //! WARNING: The folder names DO matter (conf and configuration_files folders)
    else{
        
        // Obtain the path of the AllegroHandDriver.cpp file. Example: /home/users/marina.pujol/Documents/colcon_ws/a_ros2/src/allegro_hand/src/AllegroHandDriver.cpp
        std::filesystem::path currentPath(__FILE__);
        // Obtain the path of the src folder of the allegro_hand package. Example: Example: /home/users/marina.pujol/Documents/colcon_ws/a_ros2/src/allegro_hand/src
        currentPath = currentPath.parent_path();
        // Move towards the conf folder
        currentPath /= "../conf/";
        // Write the path to the conf and the conf/configuration_files path
        std::string conf_folder = currentPath;
        std::string configuration_files_folder = conf_folder + "configuration_files/";
        std::string configuration_file_path;
        // Absolute path of the configuration file. Example: ~/..../conf/configuration_files/
        std::filesystem::path directory = std::filesystem::absolute(conf_folder);
        // Save the absolute path as a string into the class variable: directory_conf_folder. 
        directory_conf_folder = directory;
        bool has_extension = (yaml_conf_file.find('.') != std::string::npos);
        // If the yaml_conf_file does not have an extension (.yaml) we need to add it
        if(has_extension){
            configuration_file_path = configuration_files_folder + yaml_conf_file;
            // Search for the dot
            size_t lastDotPos = yaml_conf_file.find_last_of('.');
            if (lastDotPos != std::string::npos) {
                // Remove the last part after the dot
                hand_name = yaml_conf_file.substr(0, lastDotPos);
            }
        }
        else{
            configuration_file_path = configuration_files_folder + yaml_conf_file + ".yaml";
            hand_name = yaml_conf_file;
        }
        // Absolute path of the configuration file. Example: ~/..../conf/configuration_files/V4R284.yaml
        directory = std::filesystem::absolute(configuration_file_path);
        // Save absolute path of the configuration file as a string into the class variable: directory_configuration_file
        directory_configuration_file = directory;
    }
    std::cout << "Hand name is: " << hand_name << '\n'
              << "The yaml files are located in the folder: " << directory_conf_folder << std::endl 
              << "The configuration folder is located at " << directory_configuration_file << std::endl;
}

//! \brief Read hand configuration from the YAML file
//! \param yaml_file Absolute directory of the hand configuration file
bool AllegroHandDriver::readConfigurationFile(std::string& yaml_file)
{
    YAML::Node config;
    // File name of the urdf. (Read from the conf hand file)
    std::string urdf_file;
    try {
        config = YAML::LoadFile(yaml_file);
    }catch(const YAML::ParserException& ex) {
        std::cout << ex.what() << std::endl;
        return  false;
    }

    try {
        std::cout << "Hand Info" << std::endl;
        std::cout << "DOF: " << config["hand_info"]["DOF"].as<int>() << std::endl;
        control_period = config["hand_info"]["control_period_s"].as<double>();
        std::cout << "Condrol period: " << control_period << std::endl;
        //std::cout << "Which hand: " << config["hand_info"]["which_hand"].as<std::string>() << std::endl;
        std::cout << "Serial: " << config["hand_info"]["serial"].as<std::string>() << std::endl;
        params.serial = config["hand_info"]["serial"].as<std::string>() ;
        params.handVersion = config["hand_info"]["version"].as<double>() ;
        std::cout << "Version: " << params.handVersion << std::endl;

        urdf_file = config["hand_info"]["urdf_file"].as<std::string>();
        std::cout << "Urdf_file: " << urdf_file << std::endl;

        is_left = false;
        if (urdf_file.find("right") != std::string::npos) {
            std::cout << "Right hand" << std::endl;
        } else if (urdf_file.find("left") != std::string::npos) {
            std::cout << "Left hand" << std::endl;
            is_left = true;
        } else {
            std::cout << "WARNING: Unknown left or right. Please check the urdf name in conf/urdf" << std::endl;
        }

        //Save URDF as string
        // Parse urdf model without virtual links
        std::cout << "Parsing urdf: " << directory_conf_folder << "urdf/" << urdf_file << std::endl;
        urdf_string = urdfToString(directory_conf_folder + "urdf/" + urdf_file);

        params.encoderOffset.j00 = config["zero"]["encoder_offset"]["j00"].as<int>();
        params.encoderOffset.j01 = config["zero"]["encoder_offset"]["j01"].as<int>();
        params.encoderOffset.j02 = config["zero"]["encoder_offset"]["j02"].as<int>();
        params.encoderOffset.j03 = config["zero"]["encoder_offset"]["j03"].as<int>();

        params.encoderOffset.j10 = config["zero"]["encoder_offset"]["j10"].as<int>();
        params.encoderOffset.j11 = config["zero"]["encoder_offset"]["j11"].as<int>();
        params.encoderOffset.j12 = config["zero"]["encoder_offset"]["j12"].as<int>();
        params.encoderOffset.j13 = config["zero"]["encoder_offset"]["j13"].as<int>();

        params.encoderOffset.j20 = config["zero"]["encoder_offset"]["j20"].as<int>();
        params.encoderOffset.j21 = config["zero"]["encoder_offset"]["j21"].as<int>();
        params.encoderOffset.j22 = config["zero"]["encoder_offset"]["j22"].as<int>();
        params.encoderOffset.j23 = config["zero"]["encoder_offset"]["j23"].as<int>();

        params.encoderOffset.j30 = config["zero"]["encoder_offset"]["j30"].as<int>();
        params.encoderOffset.j31 = config["zero"]["encoder_offset"]["j31"].as<int>();
        params.encoderOffset.j32 = config["zero"]["encoder_offset"]["j32"].as<int>();
        params.encoderOffset.j33 = config["zero"]["encoder_offset"]["j33"].as<int>();

        params.encoderDirection.j00 = config["zero"]["encoder_direction"]["j00"].as<int>();
        params.encoderDirection.j01 = config["zero"]["encoder_direction"]["j01"].as<int>();
        params.encoderDirection.j02 = config["zero"]["encoder_direction"]["j02"].as<int>();
        params.encoderDirection.j03 = config["zero"]["encoder_direction"]["j03"].as<int>();

        params.encoderDirection.j10 = config["zero"]["encoder_direction"]["j10"].as<int>();
        params.encoderDirection.j11 = config["zero"]["encoder_direction"]["j11"].as<int>();
        params.encoderDirection.j12 = config["zero"]["encoder_direction"]["j12"].as<int>();
        params.encoderDirection.j13 = config["zero"]["encoder_direction"]["j13"].as<int>();

        params.encoderDirection.j20 = config["zero"]["encoder_direction"]["j20"].as<int>();
        params.encoderDirection.j21 = config["zero"]["encoder_direction"]["j21"].as<int>();
        params.encoderDirection.j22 = config["zero"]["encoder_direction"]["j22"].as<int>();
        params.encoderDirection.j23 = config["zero"]["encoder_direction"]["j23"].as<int>();

        params.encoderDirection.j30 = config["zero"]["encoder_direction"]["j30"].as<int>();
        params.encoderDirection.j31 = config["zero"]["encoder_direction"]["j31"].as<int>();
        params.encoderDirection.j32 = config["zero"]["encoder_direction"]["j32"].as<int>();
        params.encoderDirection.j33 = config["zero"]["encoder_direction"]["j33"].as<int>();

        params.motorDirection.j00 = config["zero"]["motor_direction"]["j00"].as<int>();
        params.motorDirection.j01 = config["zero"]["motor_direction"]["j01"].as<int>();
        params.motorDirection.j02 = config["zero"]["motor_direction"]["j02"].as<int>();
        params.motorDirection.j03 = config["zero"]["motor_direction"]["j03"].as<int>();

        params.motorDirection.j10 = config["zero"]["motor_direction"]["j10"].as<int>();
        params.motorDirection.j11 = config["zero"]["motor_direction"]["j11"].as<int>();
        params.motorDirection.j12 = config["zero"]["motor_direction"]["j12"].as<int>();
        params.motorDirection.j13 = config["zero"]["motor_direction"]["j13"].as<int>();

        params.motorDirection.j20 = config["zero"]["motor_direction"]["j20"].as<int>();
        params.motorDirection.j21 = config["zero"]["motor_direction"]["j21"].as<int>();
        params.motorDirection.j22 = config["zero"]["motor_direction"]["j22"].as<int>();
        params.motorDirection.j23 = config["zero"]["motor_direction"]["j23"].as<int>();

        params.motorDirection.j30 = config["zero"]["motor_direction"]["j30"].as<int>();
        params.motorDirection.j31 = config["zero"]["motor_direction"]["j31"].as<int>();
        params.motorDirection.j32 = config["zero"]["motor_direction"]["j32"].as<int>();
        params.motorDirection.j33 = config["zero"]["motor_direction"]["j33"].as<int>();

        std::vector<double> position = config["filter_coefs"]["position"].as<std::vector<double>>();
        std::vector<double> velocity = config["filter_coefs"]["velocity"].as<std::vector<double>>();

        filtCoeffsPos = Eigen::Vector3d(position.data());
        filtCoeffsVel = Eigen::Vector3d(velocity.data());

    }catch(const YAML::ParserException& ex) {
        std::cout << ex.what() << std::endl;
        return  false;
    }

    return true;
}

//! \brief Read URDF and store it as a string
//! \param yaml_file Absolute directory of the hand configuration file
std::string AllegroHandDriver::urdfToString(const std::string& urdf_file_path) {
    std::ifstream urdf_file(urdf_file_path);
    if (!urdf_file) {
        throw std::runtime_error("Failed to open URDF file: " + urdf_file_path);
    }

    std::stringstream buffer;
    buffer << urdf_file.rdbuf();
    return buffer.str();
}