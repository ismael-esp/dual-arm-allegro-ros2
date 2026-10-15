# Robotic Antropomorphic Hand Driver - Allegro Hand
Repository of a software to control a robotic antropomorphic hand, specifically the Allegro Hand.
This repository contains the Allegro Hand Driver and a demo application. 

## Introduction

The Allegro Hand Library, implemented in C++, has been designed to simplify the usage of the hand while enhancing its functionality. 
This software handles **communications**, complete **Forward Kinematics** and **Inverse Kinematics** (FK and IK), and supports multiple control modes, such as **Gravity Compensator, PD, PID** controller. 
Additionally, it offers both **predefined and customizable hand configurations and grasp modes**. 

The architecture decouples the drivers (communications and control) from the framework, thereby facilitating integration with other frameworks such as ROS 1, ROS 2, Orocos, and Matlab. 
This library has been used for the Allegro Hand integration in ROS 1 and ROS 2, available in [gitioc.upc.edu/hands/allegro_hand_ros](https://gitioc.upc.edu/hands/allegro_hand_ros).
It supports operating a real Allegro Hand and also can perform a simulation environment.
Works with all Allegro Hand versions and it has been tested with several Allegro Hands including different versions such as version 2,
version 3 and version 4.
Moreover, this software can be easily adapted for other robotic anthropomorphic hands.

This file contains a brief introduction to this software, its structure, key functions, requirements and installation instructions. 
Although, more extensive documentation is available. 
This extended documentation is structured in different files, for readability purposes:

---
> Extended documentation:
> - [1 - Key Functionalities](/README/1_KeyFunctionalities.md). List of key functionalities of this library.
> - [2 - Code Structure](/README/2_CodeStructure.md). Basic structure of the classes and brief description of each one of them.
> - [3 - Directory Organization](/README/3_DirectoryOrganization.md). Organization of the file and directory files.
> - [4 - The CAN Communications Class: CAN_AH](/README/4_CommunicationsClass.md). CAN Communications Class.
> - [5 - The Controller Class: AllegroHandController](/README/5_ControllerClass.md). Controller Class.
> - [6 - The Kinematics Class: AllegroFinger](/README/6_KinematicsClass.md). Kinematics Class.
> - [7 - The Basic Driver Class: AllegroHandDriver](/README/7_DriverClass.md). Basic Driver Class.
> - [8 - The Primary Class: AllegroHand](/README/8_AllegroHandClass.md). AllegroHand Class.
> - [9 - Installation and SetUp](/README/9_InstallationAndSetup.md). Requirements, how to build and how to integrate the library in a Project
> - [10 - Testing the Allegro Hand Library](/README/10_Testing.md). Examples and how to run them
> - [11 - The Official Allegro Hand Software](/README/11_TheOfficialSoftware.md) Presentation and comparison with the official Allegro Hand Software provided by the manufacturer
> - [12 - Miscellaneous](/README/12_Miscellaneous.md) 
> - [13 - Additional Information](/README/13_AdditionalInformation.md) 
---

## Step by step initial instructions

Congratulations on acquiring an Allegro Hand!
This section will provide a step by step guide to use this library.

- **Step 0:** The initial step consists on setting up the power supply and communication hardware according to the [official wiring information.](https://www.allegrohand.com/wiring-and-connector-info) This step is only necessary once when you have bought the hand.

### Step 1: Create the Hand Configuration File
 Create the configuration file for your hand. 
Go to the directory: conf/configuration_files and use either the right hand or left hand template ([Right_template](/conf/configuration_files/Right_template.yaml) and [Left_template](/conf/configuration_files/Left_template.yaml), respectively).
This file contains basic information such as communication parameters necessary to operate the hand.
This step is simple and straightforward, nevertheless the template files contain a step by step instruction.

Optional: [Adding controllers (PD, PID or others)](/README/12_Miscellaneous#-1-adding-new-controllers)

### Step 2: Install the necesary requirements
To build this library, the following dependencies are required:

- [**CMake**](https://www.cmake.org)
- [**Eigen**](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [**yaml-cpp**](https://github.com/jbeder/yaml-cpp)
- [**pcanbasic**](https://www.peak-system.com/PCAN-Basic-API.199.0.html)
- [**Boost**](http://www.boost.org)
- [**URDF**](http://wiki.ros.org/urdf)
- [**Orocos KDL**](https://github.com/orocos/orocos_kinematics_dynamics)

On a Debian/Ubuntu system, these dependencies can be installed using:
```
    $ sudo apt install cmake libeigen3-dev libyaml-cpp-dev libboost-filesystem-dev 
    libboost-system-dev liburdf-dev liborocos-kdl-dev
```
For libpcan, use the packages available for Debian or Ubuntu.

-   For Debian, add the Debian Robotic Project (DRP) repository by
    replacing `XXXX` with `bookworm` or `bullseye`:
    ```
            $ sudo sh -c 'echo "deb [arch=amd64] http://debrob.upc.edu/debian-robotics 
            XXXX-robotics main" > /etc/apt/sources.list.d/debian-robotics.list'
            $ sudo apt update
            $ sudo apt install libpcan-dev
    ```

-   For Ubuntu Focal (20.04), add the relevant PPA:
    ```
            $ sudo add-apt-repository ppa:deb-rob/focal-robotics
            $ sudo apt update
            $ sudo apt install libpcan-dev
    ```

The driver uses a modified version of `kdl_parser` by Wim Meeussen,
adapted by Pouya Mohammadi, which is not integrated with ROS.

### Step 3: Build the driver

To compile the project: first, create a build directory and execute the command cmake:

```bash
$ mkdir build
$ cd build
$ cmake ../
```

Then execute the command:

```bash
$ make
```

Upon successful compilation, the library output includes:

-   The shared library *liballegro_hand.so*.
-   Several example programs to test the library.

### Step 4: Testing the Allegro Hand Library

There are several examples to test the Allegro Hand Library. 
In this section the most complete example will be briefly presented.
This example allows the user to control through keyboard commands the Allegro Hand.
The available commands are:
<p align="center">
    <img src="README/images/results/ahlib/main.png" alt="Main Menu Allegro Hand Example" width="400"/>
</p>

- **Select a Controller:** The user can select from available controllers such as Gravity Compensator, PD, PID, or Torque controllers by entering the corresponding number (option `<1>`).
- **Forward Kinematics (FK):** The program computes and prints the Forward Kinematics of the fingertips (option `<2>`). This calculation provides the positions (Cartesian coordinates: X, Y, Z) and orientations (ZYZ Euler angles) of each fingertip.
- **Inverse Kinematics (IK):** The user can move the fingertips to a desired position and orientation using Inverse Kinematics (option `<3>`). This requires specifying 12 position values and 12 orientation values of the fingertips.
- **Move to a Defined Configuration:** The user can move the hand to a predefined configuration by specifying its name (option `<4>`).
- **Read Hand State:** The program reads and displays the current state of the hand, including joint positions in degrees and radians, velocities in degrees per second, accelerations in degrees per second squared, and torques in Newton-meters (option `<5>`).
- **Store a Configuration:** The program allows the user to manually move the hand (with the Gravity Compensator automatically activated) and save the current configuration to a YAML file under a specified name (option `<6>`).
- **Perform a Coupled Closing:** The program enables a coupled closing of the hand (option `<8>`).
- **Retrieve Configuration List:** The program lists all available configurations stored in the YAML files (option `<G> or <g>`).
- **Move to a Predefined Configuration at a Desired Velocity:** The user can specify a configuration name and a velocity percentage (up to 100%) to move the hand at the desired speed (option `<X> or <x>`).
- **Store and Execute a Trajectory:** The user can define a velocity and a final configuration name (option `<Y> or <y>`). The hand moves from the current configuration to the user-selected final configuration.
- **Move Along a Predefined Trajectory:** The user can move the hand along a trajectory defined as a list of configurations (`<T> or <t>`).
- **Record and Store a Trajectory:** The program can listen to the manually manipulated hand and store the movement trajectory.
- **Execute grasps:** The program can execute some predefined grasps (option `<i> or <I>`).
    -   **Prismatic grasp using 3 fingers:** Executes a prismatic grasp
        using 3 fingers, the Index, the Middle and the Thumb (option
        `<1>`).
    -   **Prismatic grasp using 4 fingers:** Executes a prismatic grasp
        using all 4 fingers (option `<2>`).
    -   **Pinch grasp using 3 fingers:** Executes a pinch grasp using 2
        fingers, the Index and the Thumb (option `<3>`).
    -   **Pinch grasp using 3 fingers:** Executes a pinch grasp using 2
        fingers, the Middle and the Thumb (option `<3>`).
-   **Predefined Hand Configurations:** The program includes several
    predefined hand configurations that can be selected directly:
    -   **Home Configuration:** Option `<h> or <H>`
    -   **Rock:** Option `<r> or <R>`
    -   **Paper:** Option `<m> or <M>`
    -   **Scissors:** Option `<s> or <S>`
    -   **Aloha:** Option `<a> or <A>`
    -   **Punk:** Option `<p> or <P>`
    -   **Win:** Option `<w> or <W>`
    -   **Botifarra:** Option `<b> or <B>`
    -   **Thumb up:** Option `<u> or <U>`
    -   **One finger:** Option `<o> or <O>`
-   **Exit the Program:** To safely exit the program the user selects
    option `<0>`. The hand is properly stopped before exiting.


#### Running the Main Example 

To run this example, the user must specify the hand configuration file,
which can be provided either by the file name (with or without the
`.yaml` extension) or by the full path to the YAML configuration file.

-   **Real Allegro Hand**
    
    Ensure that the hand power is activated first (the hand should be plugged in with the switch positioned upward, and the device will display a steady red light). 
    To run the example with the real Allegro Hand, connect the hand to
    the computer through the PCAN device and identify the CAN port number. 

    Then, run the following command, where hand_configuration_file refers to the name of the file created in [Step 1](#step-1-create-the-hand-configuration-file).

            $ examples/test_allegro_hand <hand_configuration_file> real

    For example, with the Allegro Hand model R283:

            $ examples/test_allegro_hand V4R283 real
            $ examples/test_allegro_hand V4R283.yaml real
            $ examples/test_allegro_hand ../conf/configuration_files/V4R284.yaml real

-   **Simulation Mode**

    To run the example in simulation mode, use the following command in
    the terminal, where hand_configuration_file refers to the name of the file created in [Step 1](#step-1-create-the-hand-configuration-file).

            $ examples/test_allegro_hand <hand_configuration_file> sim

    For example, with the Allegro Hand model R283 the program can be
    executed using:

            $ examples/test_allegro_hand V4R283 sim
            $ examples/test_allegro_hand V4R283.yaml sim
            $ examples/test_allegro_hand ../conf/configuration_files/V4R284.yaml sim


In both cases, the hand used is version 4, model right 283, with a
serial number of R283. The `pcan` driver generates the device
`/dev/pcan32`. Replace this with the appropriate number for your setup.

> - More information available in: [10 - Testing the Allegro Hand Library](/README/10_Testing.md). Contains the examples available.


### Integrating the Driver into a Project

The driver is designed to be built within a catkin workspace. To
incorporate it into your own project, include the following lines in
your `CMakeLists.txt` file:
```
    $ find_package(allegro_hand_driver REQUIRED)
    $ get_target_property(allegro_hand_INCLUDE_DIRS ${allegro_hand_LIBRARIES} 
    INTERFACE_INCLUDE_DIRECTORIES)
    $ include_directories(${allegro_hand_INCLUDE_DIRS})
    # Ensure to link ${allegro_hand_LIBRARIES} appropriately in your target.
```


<!-- The library has a main class, AllegroHand, that encapsulates the functionality of the hand. 
It includes the communication, the movements, the forward kinematics and the inverse kinematics, predefined configurations and grasps. -->


## Code Structure

The developed software package is structured into five interconnected
classes, each playing a specific role in the functionality of the
Allegro Hand. 

The primary class is the *AllegroHand* class, which orchestrates the overall operations by
embedding instances of the other classes. 
This class derives from the *AllegroHandDriver*, responsible for communications and basic control of
the hand, it integrates the *CAN_AH* class for managing CAN bus communication with the physical hardware. 
The control algorithms for the hand are encapsulated in the *AllegroHandController* class, which offers
various control modes, including torque control and gravity
compensation. 
The kinematic modeling of the fingers is handled by the
*AllegroFinger* class, which provides both Forward and Inverse
Kinematics functionalities. Together, these classes form a cohesive
system that supports both real and simulated operation of the Allegro
Hand, ensuring efficient control, communication, and kinematic
computations.

<p align="center">
    <img src="README/images/EsquemaGeneralClasses.png" alt="Modules and clases" width="800"/>
</p>

The *AllegroHand* class contains the main -and only- loop, *controlLoop* or
*controlLoopFake*, for real and simulation hands, respectively. 
The *controlLoop* reads current joint configurations,
executes the current controller and sends the desired torques to the
real Allegro Hand. The *controlLoopFake* executes the fake controller,
ensuring the desired torques are within the limits of the simulated hand.

<p align="center">
    <img src="README/images/EsquemaRealHandOperation.png" alt="Real Hand Ooperation" width="800"/>
     <img src="README/images/EsquemaSimulation.png" alt="Simulation Operation" width="400"/>
</p>

> - More information available in: [2 - Code Structure](/README/2_CodeStructure.md). Contains a brief introductory explanation of the classes.

## Software Walkthrough

<!-- This section provides a walkthrough of this software. -->
<!-- The goal of this section is to provide a comprehensive understanding of
all the classes in the software and how they interact with one another. -->
<!-- First, the organization of the code's files and directories will be
presented. Then, the functionality of each class will be thoroughly
explained, covering the purpose of the class, its constructors, and key
methods. -->
The software files and directories are organized as follows:
- The *include* and *src* folders contains the classes library and definition, respectively. 
- The *examples* directory, contains a set of files that allow to test the allegro hand driver and the allegro hand library. 
- The *conf* directory contains a set of YAML files with the hand configuration information, the controller parameters, the URDFs, a set of predefined joint positions and predefined grasps and the actual joint limits of each tested hand. The naming convention for this files is crucial.

> - More information available in: [3 - Directory Organization](/README/3_DirectoryOrganization.md). Contains the organization of the code's files and directories and the naming convention.

The functionality of each class will be presented as long as the main functions. 
A thoroughly explanation is also available in the extended documentation of each class, covering the purpose of the class, its constructors, and key
methods.


### The CAN Communications Class: CAN_AH

The communication class works with all the current AllegroHand versions and it uses the [PCANBasic library](https://www.peak-system.com/PCAN-Basic.239.0.html?&L=1&gclid=Cj0KCQiAwvKtBhDrARIsAJj-kTgDIe9BhlrAOthiNdiSRtiQbaFc9zrcyc5dCyMXV0G-_g-duScbNXsaAr-EEALw_wcB). Due to differences in the AllegroHand communication protocol there exists two analogous subclasses, one for versions 1, 2 and 3 and another subclass for the latest version, 4.

The constructor of the communication class initializes the *Controller Area Network* (CAN) communications with the hand and it sets a 3ms periodic communication. The AllegroHand sends one message per finger, and it is always sent in the same order. Thus, for ensuring that the finger messages belong to the same time, the constructor also performs an initial read in order to identify the ID of the finger of the first message.

The most important functions being:

||       **Function**     |                              **Description**                              |
|-|------------------------|---------------------------------------------------------------------------|
|1|    *readDataTimeStamp* | Reads a set of 4 messages (1 msg/finger) and returns the timeStamp     |
|2|     *readDevices*      | Parses the read messages to extract and store the position (q) of all the fingers.  |
|3|     *getPosition*      | Returns the position of the hand.
|4|      *setTorque*       | Sets the torque variable to the desired values.
|5|    *writeDevices*      | Creates and sends 4 messages (1/finger) to the AllegroHand with the desired torque values.|


> - More information available in: [4 - The CAN Communications Class: CAN_AH](/README/4_CommunicationsClass.md).


### The Controller Class: AllegroHandController
The *AllegroHandController* class is in charge of the control.

As previously mentioned, this software has implemented 5 different controllers, each controller is a class that extends the *AllegroHandController* class.

-   **AllegroHandControllerTorque**. Implements a torque pass-through open-loop controller, where the desired torque is applied directly to the joint servos.

-   **AllegroHandControllerGravityCompensator**. Compensates the
gravity action on the real hand. The required torque ($\tau$) is computed using the *Kinematics and Dynamics Library* (*KDL*), which constructs the kinematic and dynamic models of the AH hand from the URDF configuration file. 
The information needed to compute the compensation torque are the actual joint configuration low-pass filtered, it is necessary to filter the *CAN* noisy readings from the servos, the coefficients used are given by the manufacturer; and the gravity vector ($\{ g_{x}, g_{y}, g_{z} \}$). It can be set dynamically.
-   **AllegroHandControllerJointPositionPD**. 
Implements the PD controller, which takes the gain parameters for each servo from the YAML configuration file, set in the initialization of the *AllegroHand* class. This values are given by the manufacturer.
Furthermore, it is added to the PD torque signal the gravity
compensation one, explained above. It is the manufacturer original one.
-   **AllegroHandControllerJointPositionPID**. This control-loop implements the PID controller, which takes the gain parameters for each servo from the YAML configuration file, set in the initialization of the *AllegroHand* class. 
Here, it is also added to the PID torque signal the gravity compensation one, explained above.
-   **AllegroHandControllerFake**. As mentioned before, it is used when it is working with the simulated hand. In this case, it is not necessary to compensated the gravity action, as it is treated like an ideal hand. The torque command obtained is directly the desired torque to reach the target joint configuration, although later a saturation check is performed in the *AllegroHand* class.

The most important functions being:

||       **Function**     |                              **Description**                              |
|-|------------------------|---------------------------------------------------------------------------|
|1|    *executeController* | Returns the torque computed by the controller     |
|2|    *setGravityVector* | Sets the Gravity Vector (useful for compensating the gravity effects)     |
|3|     *getGravityVector*      | Gets the Gravity Vector that is being used to compensate the gravity effects.  |
|4|     *getGravityComp*      | Returns the torque needed to to compensate the gravity effects.
|5|      *getCurrentVel*       | Gets the calculated current velocity of the joints.
|6|    *getCurrentAcc*      | Gets the calculated current acceleration of the joints.|
|7|      *ComputeJacobian*       | Calculates the Jacobian.
|8|    *getRealJacobianKDL*      | Gets the Jacobian computed with the kinematic chain obtained with the KDL library.|
|9|    *ComputeFingerFrames*      | Calculates the Finger frames.|
|10|      *getFingerFrames*       | Gets the Finger frames.|
|11|    *getKDLChain*      | Get the kinematic chain obtained using the KDL library.|

> - More information available in: [5 - The Controller Class: AllegroHandController](/README/5_ControllerClass.md). 

### The Kinematics Class: AllegroFinger
As we previously mentioned it has two child classes, one for the Index, Middle and Annular (IMA) fingers, *IMAFingers*, and another for the Thumb, *ThumbFinger*. 

The most important functions being:

||       **Function**     |                              **Description**                              |
|-|------------------------|---------------------------------------------------------------------------|
|1|    *Fkine* | Returns the position and orientation of a fingertip. Forward kinematics of a fingertip     |
|2|    *Ikine* | Returns the configuration to set a fingertip to the desired position and orientation    |
|3|     *eul2Tmat*      | Convert from eul angles to Tranformation matrix.
|4|      *getJointAngles*       | Returns current joint angles.
|5|    *getTransformationMatrix*      | Returns the desired transformation frame.|
|6|      *getTotalTransformationMatrix*       | Returns the total transformation frame.
|7|    *getTF_w2b*      | Returns the transformation from world to the base of the finger.|

> - More information available in: [6 - The Kinematics Class: AllegroFinger](/README/6_KinematicsClass.md).

### The Basic Driver Class: AllegroHandDriver
The *AllegroHandDriver* class is a basic driver to communicate with the hand without performing advanced control actions. 
It contains an instance of the *CAN\_AH* class. 

The most important functions being:

||       **Function**     |                              **Description**                              |
|-|------------------------|---------------------------------------------------------------------------|
|1|     *getCurrentStateComm*      | Reads joint values from real hand.
|2|    *setTorque* | Sends torque to the hand.     |
|3|    *getTorque* | Gets the torque sent to the hand    |
|4|     *getTorqueConversion*      | Returns torque conversion constant from real Allegro Hand.|
|5|      *getJointLimitsMin*       | Returns minimum value in rad of all hand joints.|
|6|    *getJointLimitsMax*      | Returns maximum value in rad of all hand joints.|
|7|      *getVelocityLimits*       | Returns maximum velocity value in rad of all hand joints.|
|8|      *getEffortLimits*       | Returns maximum torque allowed of all hand joints.|
|9|    *stopHand*      | Stops communication with the hand.|

> - More information available in: [7 - The Basic Driver Class: AllegroHandDriver](/README/7_DriverClass.md).

### The Primary Class: AllegroHand 
The *AllegroHand* class is the most crucial, 
since it is a derived class from the *AllegroHandDriver* and embeds instances of the other classes.      
When initialized it reads a configuration file, the URDF and predefined joint configurations and grasp modes.
It can run both, a real Allegro Hand or a simulated one. 
It instantiates a *AllegroController* class (Torque or Fake for real or simulated, respectively) and four instances of the kinematic classes (three IMA finger and one Thumb finger).
In both cases it contains a thread executing the main -and only- loop, *controlLoop* or *controlLoopFake*.
When working with a physical hand, it initializes an instance of the *CAN_AH* class for communication and finds the available controllers.
The *controlLoop* reads current joint configurations, executes the current controller and sends the desired torques to the Allegro Hand. The *controlLoopFake* executes the fake controller, ensuring the desired torques are within the limits of the simulated hand.

<p align="center">
    <img src="README/images/InicialitzacioAH.png" alt="AllegroHand Class Initialization" width="600"/>
</p>

<!-- Previously, we have stated the features of this class, we will now show the most important functions: -->
The most important functions being:

||       **Function**     |                              **Description**                              |
|-|------------------------|---------------------------------------------------------------------------|
|1|    *getCurrentState*   |                       Get the current states configuration [rad], velocity [rad/s], acceleration [rad/s²], torque [Nm].  
|2|    *getStatus*   |                       Get status.  
|3|    *getCurrentController*   |       Gets the name of the current controller being used.      |
|4|     *getControllersNames*   |                  Gets the list of available controllers.                  |
|5|     *setController*   |            Sets the desired controller.            |
|6|    *setGravityVector*  |            Set the gravity vector. $\{ g_{x}, g_{y}, g_{z} \}$            |
|7|    *setGravityVector* | Sets the Gravity Vector (useful for compensating the gravity effects)     |
|8|     *getGravityComp*   |  Gets the gravity compensation torque for the current hand configuration. |
|10|   *getError* |                          Get joint position errors.                          |
|11|   *readPredefinedConfigurations* |                     Read yaml with the predefined configurations.                      |
|12|        *getConfList*      |     Gets list of configurations.                         |
|13|    *addConf* | Add a new configuration to the list     |
|14|     *delConf*      | Erase a configuration from the list.
|15|      *storeCurrentConf*       |Store current configuration temporarily or in the YAML file.
|16|    *setDesiredConfiguration*      | Set desired hand configuration.|
|17|      *moveToConf*       | Move hand to a configuration (name or joint values) with or without a defined velocity.
|18|    *moveToConfAndSaveTrajectory*      | Move hand to a configuration with a defined velocity and store the trajectory in a yaml file.|
|19|    *getTrajectoriesListNames*      | Gets list with the names of the predefined hand trajectories defined by a list of configurations.|
|20|    *getTrajectoriesPosesListNames*      | Gets list with the names of the predefined hand trajectories defined by fingertip poses.|
|21|      *moveAlongTrajectory*       | Move the hand along a defined trajectory defined by a list of configuration.|
|22|    *moveAlongPosesTrajectory*      |Move the hand along a defined trajectory defined by fingertip poses.|
|23|      *storeCurrentTrajectory*       | Store current hand movement as a trajectory.|
|24|    *stopListeningCurrentTrajectory*      | Stops storing current trajectory.|
|25|    *stopMovement*      | Cancels current trajectory movement.|
|26|    *isMoving*      | Get if the hand is moving to a desired configuration|
|27|      *computeFKFinger*       | Forward kinematics of one fingertip.
|28|    *computeFK*      |  Forward kinematics of the whole hand (4 fingertips).|
|29|    *computeIKFinger*      | Computes the inverse kinematics of one fingertip.|
|30|    *computeIKHand*      | Computes the inverse kinematics of the whole hand.|
|31|      *executeIK*       | Executes the inverse kinematics. The fingers move to the desired pose.|
|32|    *ComputeJacobian*      |Computes the Jacobian.|
|33|      *getRealJacobianKDL*       | Computes Jacobian with the KDL.|
|34|    *getJointLimits*      | Get minimum and maximum joint position values.|
|35|    *getJointLimitsSafetyMargin*      | Gets the safety margin of the joint positions.|
|36|    *setJointLimitsSafetyMargin*      | Set a safety margin for the joint positions.|
|37|    *useSafetyMargin*      | Chooses to use or not the safety margin|


> - More information available in: [8 - The Primary Class: AllegroHand](/README/8_AllegroHandClass.md).


## Conclusion

The Allegro Hand Library is a fully developed open-source C++ software
package that allows improving and facilitating the use of robotics
hands, and in particular, the Allegro Hand. It provides crucial
functionalities, such as FK and IK, and simplifies integration with other
frameworks, laying a solid foundation for future work in grasping
research.

The comparison between the official software and the this library, Allegro
Hand Library, is shown below: 

<p align="center">
    <img src="README/images/SoftwareComparison.png" alt="Comparison between the official software and the this library" width="60%"/>
</p>

> - More information available in: [11 - The Official Software](/README/11_TheOfficialSoftware.md). Contains an explanation of the official Allegro Hand Software as well as a comparison

The modular design allows to easily add new functionalities within the
software, such as a specialized controller to enhance grasping
capabilities. Moreover, the implementation of several control
algorithms, such as the PID, allows performing precise grasps.
Furthermore, the software is designed to be easily integrated with
packages containing customized controllers, thus eliminating the need to
allocate the controller within the package.

The software facilitates advanced research and development in grasp
theory since it allows to run both a real and a simulated Allegro Hand.

