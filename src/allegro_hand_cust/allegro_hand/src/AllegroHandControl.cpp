
#include <allegro_hand/AllegroHand.h>

/**
Functions of the AllegroHand class related to the controller
**/

//! \brief Create controller instance based on the controller name
//! \param controller_name. Controller name
//! \return AllegroHandController* Pointer to the new instance of a derived class of AllegroHandController
AllegroHandController* AllegroHand::createController(const std::string& controller_name){
        std::string controller_path = directory_conf_folder + "controllers/" + hand_name + "_" + controller_name + ".yaml";

        if (controller_name == "GravityCompensator") {
            return new AllegroHandControllerGravityCompensator(finger_chain, chain, torqueConv, torque_max, filtCoeffsPos, filtCoeffsVel);
        } else if (controller_name == "Torque") {
            mutex_torque.lock();
                torque = Vector::Zero();
                torque_desired = Vector::Zero();
            mutex_torque.unlock();
            return new AllegroHandControllerTorque(finger_chain, chain, torqueConv, torque_max, filtCoeffsPos, filtCoeffsVel);
        }else if (controller_name == "JointPositionPID") {
            return new AllegroHandControllerJointPositionPID(controller_path, finger_chain, chain, torqueConv, torque_max, filtCoeffsPos, filtCoeffsVel);
        }else if (controller_name == "JointPositionPD") {
            return new AllegroHandControllerJointPositionPD(controller_path, finger_chain, chain, torqueConv, torque_max, filtCoeffsPos, filtCoeffsVel);
        }else if (controller_name == "Fake") {
            return new AllegroHandControllerFake(finger_chain, chain, torqueConv, torque_max, filtCoeffsPos, filtCoeffsVel);
        }
        std::cout << "Invalid controller name" << std::endl;
        return nullptr; // Return nullptr if the controller name is invalid
}

 //! \brief Print a list of the available controllers and store them in class variable controllers_names.
 //! \param print True print on the console.
 //! \param controllers_names_ Controller names
void AllegroHand::getControllersNames(bool print, std::vector<std::string> &controllers_names_){
    controllers_names_ = controllers_names;
    // Print on the console the available controllers
    if (print){
        std::cout << "Available controllers: " << std::endl;
        auto i = 1;
        for (auto controller: controllers_names ){
            std::cout << i << ". " << controller << std::endl;
            ++i;
        }
    }
}

 //! \brief Print a list of the available controllers and store them in class variable controllers_names.
void AllegroHand::getControllersNames(){
    std::cout << "Available controllers: " << std::endl;
    auto i = 1;
    for (auto controller: controllers_names ){
        std::cout << i << ". " << controller << std::endl;
        ++i;
    }
}

 //! \brief Set controller
 //! \param controller Controller name to set
bool AllegroHand::setController(std::string controller){
    std::cout << "Setting controller: " << controller << std::endl;

    // Check that the controller that we want to set up is not the same as the one that we are currently using
    if(!(controller == current_controller)){
        // Check that the desired controller is available
        auto controller_check = std::find(controllers_names.begin(), controllers_names.end(), controller);
        if(controller_check != controllers_names.end()){
            // Delete AllegroController instance and create a new one
            mutex_controller.lock();
                controller_available = false;
            mutex_controller.unlock();
            if (AllegroController != nullptr) {
                delete AllegroController; // Delete the current controller instance
            }
            AllegroController = createController(controller); // Create a new controller instance
            // Change the current controller name
            current_controller = controller;
            mutex_controller.lock();
                controller_available = true;
            mutex_controller.unlock();
            // Define the desired configuration as the current configuration.
            // We do not need to check that is within joint limits because it is the current configuration
            mutex.lock();
                q_desired = q_current;
            mutex.unlock();
            // if (!setDesiredConfiguration(q_current)) {
            //     std::cout << "Invalid configuration, ignored" << std::endl;
            // }
            return true;
        }
        else{
            std::cout << "Error setting up the controller: " << controller << std::endl;
            return false;
        }
    }
    return true;
}

 //! \brief Set Torque controller
bool AllegroHand::setTorqueController(){
    return setController("Torque");
}
 //! \brief  Set controller Gravity Compensator
bool AllegroHand::setGravityController(){
    return setController("GravityCompensator");
}
 //! \brief  Set controller PD
bool AllegroHand::setPDController(){
    return setController("JointPositionPD");
}
 //! \brief Set controller PID
bool AllegroHand::setPIDController(){
    return setController("JointPositionPID");
}

//! \brief Obtain the available controllers. We read the /conf/controllers folder and search for the ones that contain the hand_name in the filename
//! WARNING: The controller filename is: hand_name + '_' + controller_name + .yaml
bool AllegroHand::searchControllers(){
    // Obtain path to the conf/controllers directory
    std::string controller_folder_path = directory_conf_folder + "controllers/";
    std::cout << "Searching for controllers available of " << hand_name << '\n'
            << "located at " << controller_folder_path << std::endl;
    try {
        // Go to the conf/controllers directory and iterate through all the files
        for (const auto & controller_file : std::filesystem::directory_iterator(controller_folder_path)){
            // Get the filename
            // std::string controller_file_name = controller_file.path().filename().string();
            // Get the filename without extension .yaml
            std::string controller_file_name_no_ext = controller_file.path().stem().string();
            // Get the filepath
            std::string controller_file_path = controller_file.path().string();
            // If the hand_name is in the filename of the controller: extract the controller name and store it
            if (controller_file_name_no_ext.find(hand_name) != std::string::npos){
                // Find the position of the '_' character in the filename
                //! REMINDER: The controller filename is: hand_name + '_' + controller_name + .yaml
                size_t underscorePos = controller_file_name_no_ext.find('_');
                // Check if '_' is found
                if (underscorePos != std::string::npos) {
                    // Extract the controllername (substring after the '_')
                    controllers_names.push_back(controller_file_name_no_ext.substr(underscorePos + 1));
                } else {
                    std::cout << "Error reading the controller file: " << controller_file_name_no_ext << ". The structure of the filename should be: hand_name + '_' + controller_name";
                }
            }
        }
    }catch(const YAML::ParserException& ex) {
        std::cout << ex.what() << std::endl;
        return  false;
    }
    return true;
}


void AllegroHand::getGravityVector(KDL::Vector &gravity_){
    AllegroController->getGravityVector(gravity_);
}

void AllegroHand::setGravityVector(KDL::Vector &gravity_){
    AllegroController->setGravityVector(gravity_);
}

//! \brief Set gravity vector for gravity compensation purposes.
void AllegroHand::setGravityVector(std::array<double, 3> &gravity_vect){
  KDL::Vector gravity_vector;
  for (size_t i=0; i<gravity_vect.size(); ++i){
    gravity_vector[i] = gravity_vect[i];
  }
  AllegroHand::setGravityVector(gravity_vector);
}

//! \brief Get gravity vector for gravity compensation purposes.
void AllegroHand::getGravityVector(std::array<double, 3> &gravity_vect){
  KDL::Vector gravity_vector;
  AllegroHand::getGravityVector(gravity_vector);
  for (size_t i=0; i<gravity_vect.size(); ++i){
    gravity_vect[i] = gravity_vector[i];
  }
}

void AllegroHand::getGravityComp(AllegroHandDriver::Vector &gc_) {
    AllegroController->getGravityComp(gc_);
}

//! \brief get finger frames with the virtual link
void AllegroHand::getFingerFrames(std::array<std::vector<KDL::Frame>, n_fingers> &finger_frames_) {
    AllegroController->getFingerFrames(finger_frames_);
}