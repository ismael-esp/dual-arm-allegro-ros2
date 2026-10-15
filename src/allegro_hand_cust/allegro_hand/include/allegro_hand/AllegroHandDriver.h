#ifndef ALLEGROHANDDRIVER_H_
#define ALLEGROHANDDRIVER_H_

#include "dynamics_AH.h"
#include "CAN_AH.h"

#include <thread>
#include <mutex>

#include <list>
#include <yaml-cpp/yaml.h>


class AllegroHandDriver {
public:
    // Number of fingers and DOF of each finger
    static constexpr int n_fingers = 4;
    static constexpr int finger_n_dofs = 4;

    // Vector used for the q, vel, acc, torque
    typedef Eigen::Matrix<double, n_fingers*finger_n_dofs,1> Vector;

    //! Constructor. Reads configuration file, Opens CAN communication and if _driver_loop creates a loop that continuously reads the hand
    AllegroHandDriver(bool _driver_loop, std::string _conf_file, std::string _pcan_port, unsigned int channel);

    //! Constructor. Create a simulated hand to operate
    AllegroHandDriver(std::string _conf_file);

    //! Empty Constructor
    AllegroHandDriver();

    //! Stops control loop and closes CAN communication
    ~AllegroHandDriver();
    // Sends torque set to zero to the hand
    void stopHand();

    // Sets torque of the hand. It checks that the torque is within the hand torque limits (< torque_max)
    bool setTorque(Vector &torque_);
    bool setTorque(std::vector<double> &torque_finger_desired, int finger_index);
    // Sets and sends the hand torque. Checks that is within the limits. (Only if there is no control loop that already sends it)
    void sendTorque(Vector &torque_);
    
    //! Reads joint values from real hand. We read the message, we use the CAN_AH functions
    void getCurrentStateComm(Vector &q_);
    //! Reads joint values from real hand when the driver_loop is applied
    void getCurrentStateLoop(Vector &q_);

    // Get the torque sent to the hand
    inline void getTorque(Vector &torque_){mutex_torque.lock(); torque_ = torque; mutex_torque.unlock();}

    //Get limits
    inline void getLimitsMin(std::vector<double> &limits){
        std::cout <<"Deprecated function. Use getJointLimitsMin" << std::endl;
        limits = q_min;}
    inline void getLimitsMax(std::vector<double> &limits){
        std::cout <<"Deprecated function. Use getJointLimitsMax" << std::endl;
        limits = q_max;}
    inline void getJointLimitsMin(std::vector<double> &limits){limits = q_min;}
    inline void getJointLimitsMax(std::vector<double> &limits){limits = q_max;}
    inline void getVelocityLimits(std::vector<double> &limits){limits = vel_max;}
    inline void getEffortLimits(std::vector<double> &limits){limits = torque_max;}

   //! Reads torque conversion constant from real Allegro Hand
    inline double getTorqueConversion() { return device->getTorqueConversion();}
    
    // String containing the absolute path to the conf folder.
    std::string directory_conf_folder;
    // String containing the hand name. WARNING: It is the name that we put into the conf file (located in the conf folder). It may differ from the official one depending on the name of the file
    std::string hand_name;
protected:
    // Set up function to initialize the real hand
    void setUpAHD(std::string _conf_file, std::string _pcan_port, unsigned int _channel);
    // Set up function to initialize the simulated hand
    void setUpAHD(std::string _conf_file);

    // Joint limits. Configuration, velocity and torque
    std::vector<double> q_min, q_max, vel_max, torque_max;
    // Boolean to know if it is a simulated hand or a real hand
    bool simulated;

    /* Related to the torque we send to the hand*/
    // Ensures that the desired torque is within the limits. If nots it sets it accordingly.
    bool setDesiredTorque(Vector &torque_);
    bool setDesiredTorque(std::vector<double> &torque_finger_desired, int finger_index);
    // Torque sent to the hand
    Vector torque, torque_desired;
    // Communications period
    double control_period;
    
    /*Related to reading YAML files*/
    // URDF as string
    std::string urdf_string;
    // Coefficients of the filters (used for the controllers, read from the conf hand file)
    Eigen::Vector3d filtCoeffsPos, filtCoeffsVel;
    // Mutual exclusive to ensure variables are not written/read at the same time
    std::mutex mutex;
    std::mutex mutex_torque;
    bool is_left; // True: left; False: right.

private:
    /* Related to the CAN communication*/
    // Instance of the CAN communications service
    CAN_AH *device;
    // Parameters obtained from the conf file (defined by the manufacturer). Useful for versions < 4.
    CAN_AH::Parameters params;
    
    /*Related to reading YAML files*/
    // Obtain the path to the /conf folder
    void getConfDirectory(std::string yaml_conf_file);
    // Read hand configuration from the YAML file
    bool readConfigurationFile(std::string& yaml_file);
    // String containing the absolute path to the configuration file.
    std::string directory_configuration_file;

    /* Below only useful when the driverLoop exists*/
    // Current configurations. Useful only when we have a driver_loop 
    Vector q; 
    // If true we define a driver loop to constantly read the hand configuration within the driver 
    bool driver_loop;
    // Status for running or not the driver_loop
    bool status_driver_loop;
    // Driver Thread that will call the driverLoop
    std::thread driverThread;
    // Main loop. Reads the actual configuration, and sends a message with the desired torque towards the hand
    void driverLoop();

    // Sends torque (private var torque) to the hand
    void sendTorque();
    // bool setCommandedTorque(Vector &torque_);
    // void saveRetValuesToExcel(const std::string &filename);

    // Read URDF and return a string containing the hole URDF
    std::string urdfToString(const std::string& urdf_file_path);
};

#endif // AHAND_H_
