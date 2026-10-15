# 7 - The Basic Driver Class: AllegroHandDriver

---
> - 🔙 Go back to the main README file: [0 - Main](../README.md#the-basic-driver-class-allegrohanddriver)
> - ⬅️ Previous section: [6 - The Kinematics Class: AllegroFinger](6_KinematicsClass.md)
> - ➡️ Next section: [8 - The Primary Class: AllegroHand](8_AllegroHandClass.md)
---

The *AllegroHandDriver* class is a basic driver to communicate and
control the Allegro robotic hand, without implementing any advanced
control algorithms. It can only run real Allegro Hands, although a
simulated constructor is also included. It reads the hand configuration
file, handles communication with the hand since it allows reading the
current joint positions and setting specific torques. It contains an
instance of the *CAN_AH* class.

## Constructors

There are three constructors available, one for the real hand, one for
the simulated hand and an empty constructor. 

-   **Constructor for a Real Hand:** The input parameters include:

    -   *\_conf_file:* Hand configuration filename or file path. An
        string variable that can be the path of the configuration file
        or the filename, with or without the .yaml extension.

    -   *\_pcan_port:* Communications port.

    -   *\_channel:* CAN communications channel.

    -   *\_driver_loop:* A boolean to decide wether to create a control
        thread inside the driver to continuously read and write hand
        states.

    Starts by initializing four member variables: *torque* is set to a
    zero vector, *device* is initialized to nullptr, the *driver_loop*
    is set to the input variable *\_driver_loop*, and the additional
    flag variable of the driver loop is set to `true`
    *status_driver_loop*. The *device* variable is the one that will
    contain the *CAN_AH* object. The constructor first obtains the
    *conf* folder directory using the private method
    *getConfDirectory()*. This method stores in the class variables
    *hand_name* and *directory_conf_folder*, the filename of the hand
    configuration file and the absolute path of the *conf* folder. It
    has as an input parameter *\_conf_file*, that can be a filename or a
    relative or absolute file path. The *conf/configuration_files*
    folder contains the configuration files of several hands. Each
    manufactured hand has its own file, due to the varying parameters
    between manufactured hands. The configuration files are named using
    the serial number of the hand, in case the user wants to add a new
    configuration file it is a good practice to use the serial number of
    the hand.

    The constructor proceeds by reading the YAML hand configuration file
    through the method *readConfigurationFile()*. Which stores the hand
    serial number and version, the URDF filename and the communications
    parameters, such as the encoder offset, the encoder direction and
    the motor direction and the filter coefficients for the controllers.
    This method searches in the URDF filename 'left' and 'right' to store
    the hand type. Moreover, the URDF is stored as a string.

    The constructor instantiates a *CAN_AH* class to initialize
    communications with the hand. In case *\_driver_loop* input is set
    to `true`, it starts a thread to continuously read and write hand
    states, through the *driverLoop()* method. The constructor
    initializes the maximum torque values variable, *torque_max*.

-   **Constructor for a Simulated Hand:** Similar to the real hand
    constructor but tailored for a simulated environment. It only
    contains one input parameter, the hand configuration filename or
    file path, *\_conf_file*.

    Starts by initializing three member variables: *torque* is set to a
    zero vector, *device* is initialized to nullptr, the *driver_loop*
    is set to `false`. As before, the constructor first obtains the
    *conf* folder directory using the private method
    *getConfDirectory()*. This method stores in the class variables
    *hand_name* and *directory_conf_folder*, the filename of the hand
    configuration file and the absolute path of the *conf* folder. The
    constructor proceeds by reading the YAML hand configuration file
    through the method *readConfigurationFile()*, that has been
    described in the previous constructor. It also initializes the
    maximum torque values variable, *torque_max*.

-   **Empty Constructor:** The empty constructor has no input
    parameters. It initializes three member variables: *torque* is set
    to a zero vector, *device* is initialized to nullptr, and
    *driver_loop* is set to `false`. The constructor body is empty,
    meaning no additional operations are performed during
    initialization.

The body parts of the constructor for the real hand and for the
simulated hand are grouped in the *setUpAHD()* method. When the method
has one input parameter, the configuration hand file *conf_file*, it
corresponds to the body of the constructor for the simulated hand.
Otherwise, if the method has three parameters, *conf_file*, *pcan_port*
and *channel* it corresponds to the body of the constructor for the real
hand.

## Destructor

Ensures that resources are released properly, such as stopping the hand
and freeing up memory by deleting the communication device

## Key Methods

The key methods of the *AllegroHandDriver* are listed below. These
methods can be grouped in related to reading hand's joint positions are
*getCurrenStateComm()* and *getCurrenStateLoop()*, related to sending
torques towards the hand are *setTorque()* and *sendTorque()*, safety
methods (*stopHand()*) and the driver loop method, *driverLoop()*.

1.  **getCurrenStateComm():** This method can only be used when working
    with a real hand. Retrieves the current joint values in radians from
    the real hand. The *CAN_AH* instance is being used in this method.
    For obtaining the current hand configuration, the method
    *readDataTimeStamp()* of the *CAN_AH* object is called first, in
    order to read the messages containing the joint position values of
    the hand. Then, the method *readDevices()* of the *CAN_AH* object is
    called to parse those messages. Finally, the *getPosition()* method
    of the *CAN_AH* object returns the current hand joint position
    values.

2.  **getCurrentStateLoop():** This method can only be used when working
    with a real hand, and the control loop is in the *AllegroHandDriver*
    class.

3.  **setTorque():** Stores the desired torque in the class variable
    *torque*, ensuring the torque values remain within predefined
    limits. Can set the torques of the entire hand or only the torques
    of one finger, in that case, the other fingers torque will remain
    constant.

4.  **sendTorque():** Directly sends the desired torque to the hand
    after checking that the values are within acceptable limits. The
    *CAN_AH* instance is being used in this method. It has as an input
    the desired torques. First, the *setTorque()* method is called and
    then the methods *setTorque()* and *writeDevices()* of the *CAN_AH*
    object are called to send the torques to the hand.

5.  **stopHand():** Stops all communication and control threads,
    ensuring the hand is safely powered down by sending zero torque
    values.

6.  **driverLoop():** The developed software is designed to be
    controlled by only one loop. This method contains a basic control
    loop of the Allegro Hand. It runs in a separate thread if the
    *driver_loop* option is enabled. The driver loop continuously reads
    the current state of the hand, by calling the method
    *getCurrentStateLoop()* and sends the desired torque towards the
    real Allegro hand.

7.  **getTorqueConversion():** Retrieves the tau conversion constant of
    the Allegro Hand using the method *getTorqueConversion()* of the
    *CAN_AH* object.

---
> - 🔙 Go back to the main README file: [0 - Main](../README.md#the-basic-driver-class-allegrohanddriver)
> - ⬅️ Previous section: [6 - The Kinematics Class: AllegroFinger](6_KinematicsClass.md)
> - ➡️ Next section: [8 - The Primary Class: AllegroHand](8_AllegroHandClass.md)
---