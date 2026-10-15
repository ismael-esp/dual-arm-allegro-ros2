# 8 - The Primary Class: AllegroHand

---
> - 🔙 Go back to the main README file: [0 - Main](../README.md#the-primary-class-allegrohand)
> - ⬅️ Previous section: [7 - The Basic Driver Class: AllegroHandDriver](7_DriverClass.md)
> - ➡️ Next section: [9 - Installation And Setup](9_InstallationAndSetup.md)
---

The *AllegroHand* class is the most crucial, since it is a derived class
from the *AllegroHandDriver* and embeds instances of the other classes.
When initialized it reads a configuration file, the
URDF and
predefined joint configurations and grasp modes. It can run both a real
Allegro Hand or a simulated one. It instantiates a *AllegroController*
class (Torque or Fake for real or simulated, respectively) and four
instances of the kinematic classes (one per finger, thus three
*IMAFinger* and one *ThumbFinger*). In both cases it contains a thread
executing the main -and only- loop, *controlLoop* or *controlLoopFake*.

<p align="center">
    <img src="images/EsquemaRealHandOperation.png" alt="Real Hand Ooperation" width="800"/>
     <img src="images/EsquemaSimulation.png" alt="Simulation Operation" width="400"/>
</p>


When working with a physical hand, it initializes an instance of the
*CAN_AH* class for communication and finds the available controllers.
The *controlLoop* reads current joint configurations, executes the
current controller and sends the desired torques to the Allegro Hand.
The *controlLoopFake* executes the fake controller, ensuring the desired
torques are within the limits of the simulated hand.

The *AllegroHand* key features of the *AllegroHand* class are listed
below.

-   **Control algorithms:** It allows users to obtain a list of the
    available controllers, set the desired controller, get the current
    controller and obtain joint position errors. It can also get and set
    the hand gravity vector, used to calculate the necessary torque to
    compensate the gravity effect, considering the hand orientation.
    This feature is important when the hand is installed on a robot arm
    whose movement affects the hand gravity vector. When switching the
    controller, the previous controller instance is deleted, ensuring
    efficient memory management.

-   **Hand configuration:** It allows to obtain the current joint
    positions and set the desired ones. It can read YAML files with
    predefined configurations, list all saved configurations,
    temporarily add and delete configurations from the list, permanently
    store a configuration into the desired YAML file and move the hand
    to a new or a predefined configuration, among other functionalities.

-   **Grasps modes:** It allows to move the hand to a predefined grasp
    configuration along a given trajectory. Additionally, it can read
    and store trajectories to perform different grasp types from YAML
    files.

-   **Forward and Inverse Kinematics:** These functions compute the
    FK and the
    IK of the hand.
    Moreover, there is a function to obtain the Jacobian, which uses the
    kinematic chain built from the
    URDF model
    using the KDL library.

**Constructor**

The AllegroHand class provides an interface for controlling and
simulating the Allegro robotic hand. The class extends from the
*AllegroHandDriver* class, which encapsulates the basic driver functions
necessary for communicating with the robotic hand.

There are three constructors available, one for the real hand, one for
the simulated hand and an empty constructor. The differences between
initializing a real and a simulated hand are noticeable in the
constructor method. For this reason each constructor is explained
separately, first stating the input parameters and then the constructor
method will be explained. The procedure has been grouped into five
different sections in order to highlight the differences between the
constructor for the real and the simulated hand. 

<p align="center">
    <img src="images/InicialitzacioAH.png" alt="AllegroHand Class Initialization" width="600"/>
</p>

-   **Constructor for a Real Hand:** Initializes a real Allegro Hand. To
    do so, stores the hand information and sets up the
    CAN
    communications, instantiating a *CAN_AH* class. Additionally, it
    computes the kinematic chain of the hand and instantiates four
    *AllegroFinger* classes to handle the kinematics. Moreover, it
    stores all the available controllers names and sets the torque
    control as the initial controller with commanded torque initialized
    to zero. The constructor starts the control loop thread to control
    the real Allegro Hand.

    The input parameters are:

    -   *\_conf_file:* Hand configuration filename or file path. An
        string variable that can be the path of the hand configuration
        file or the filename, with or without the .yaml extension.

    -   *\_pcan_port:* Communications port.

    -   *\_channel:* CAN communications channel.

    Now, the constructor method for a real hand will be explained in
    detail, the process is enumerated since the constructor of the
    simulated hand share many steps.

    1.  Start hand communications and store basic information.

        The constructor initializes several member variables to default
        parameters:

        -   *moving* is set to `false`.

        -   *link_prefix* and *joint_prefix* are initialized as empty
            strings.

        -   *q_desired* is initialized to a zero vector.

        -   *status* is set to `true`.

        -   *listening_to_trajectory_to_store* is initialized to
            `false`.

        -   *safety_margin_joint_limits_values* is initialized to a
            vector of 16 elements, all set to `0.0`.

        The constructor initializes the *AllegroHandDriver* constructor
        for the real hand with the *\_driver_loop* parameter set to
        `false`, and passing the input parameters *\_conf_file*,
        *\_pcan_port* and *\_channel*. Once this is done, the
        information extracted from the hand configuration file and the
        corresponding URDF will be stored. This information
        involves the hand name, serial number, type (right or left) and
        version, the communications parameters (encoder offset, encoder
        direction and motor direction), the filter coefficients for the
        controllers, the maximum torque values and the
        URDF
        information as an string. Also, the *conf* folder directory will
        be stored in the class variable *directory_conf_folder*.
        Additionally, an instance of the *CAN_AH* class is added to
        initialize communications with the real hand. The torque
        conversion constant is stored using the *getTorqueConversion()*
        method.

    2.  Read the URDF and instantiate the kinematics
        classes.

        Concurrently, the constructor extracts the information of the
        URDF
        string, through the method *readConfURDF()*. This method reads
        the URDF
        extracts important information about the hand's joints and
        kinematics. This includes joint limits (position, velocity, and
        torque), the coordinate position of joints (origin), and the
        lengths of the finger links.

        Additionally, it constructs the Kinematic Description Language
        (KDL) tree and the finger
        kinematic chains, which are crucial for performing
        kinematic calculations on the hand. For each two
        kinematic chains have been stored, *finger_chain*
        and *chain*, the arrays contain the
        kinematic chain of all the fingers of the hand.
        The former, *finger_chain*, represents the
        kinematic chain of the finger without considering
        the very last link, the tip link, this one will be used when
        virtual segments are added to the
        kinematic chain. While the latter, *chain*, is
        the actual physical chain, including all joints and the final
        tip link.

        The constructor also initializes four *AllegroFinger* classes
        corresponding to each Allegro finger; three *IMAFinger* classes
        and one *ThumbFinger* class. To do so, the values extracted from
        the URDF
        are passed as input values of these classes.

    3.  Store the available controllers and set the initial Torque
        controller.

        The constructor adds to the list of available controllers
        *controller_names* the torque and gravity compensator
        controllers, \"Torque\" and \"GravityCompensator\",
        respectively. Some controllers, such as PD and PID, have parameters that can be
        tunned and that change depending on the manufactured hand and
        application. For this reason, a YAML file containing these
        parameters is stored in the *conf/controllers* folder. The
        filename contains the hand name and the controller name. The
        list of the available controllers gets completed using the
        *searchControllers()* method. To do so, this method iterates
        through all the files of the *conf/controllers* folder and
        stores the controllers whose filenames contain the current hand
        name. Then it initializes the torque controller through the
        method *setController()*, that creates an instance of the
        corresponding *AllegroHandController* class.

    4.  Initialize the control thread.

        Up next, the control thread, responsible for running the control
        loop, *controlThread*, is initialized. The *controlLoop()*
        function is started in the control thread. By launching the
        control loop in a separate thread, the loop continues running
        without being blocked by the main program operations.

        The *controlLoop()* method implements the main and only control
        loop of the *AllegroHand*. It continuously reads the current
        state of the hand, *q_current*, by calling the method
        *getCurrentStateLoop()*. Then, if the controller is available,
        the *executeController()* method of the corresponding
        *AllegroController* class is called to compute the desired
        torques. Finally, the calculated torques are sent towards the
        real Allegro hand using the method *sendTorque()*. This loop
        uses mutexes to update the previous hand configuration and the
        status flag, *q_prev* and *status*. In case the *status* flag is
        `false`, then the control loop stops.

    5.  Read predefined configurations and trajectories.

        Finally, the *AllegroHand* constructor reads the predefined
        configurations and trajectories. First, it stores the filepath
        of the files that will contain the hand configurations
        permanently stored by the *AllegroHand* class. There exist one
        file for storing the left hand configurations and one file for
        the right hand configurations. Additionally, all the files of
        the *conf/configurations/left* or *conf/configurations/right*
        folder are read and all the configurations within each file are
        stored in the class variable *configurations_list*. This
        variable maps the names and the corresponding joint position
        values.

        Then, the method *readTrajectories()* is called. This method
        stores the available trajectory names, by storing the names of
        all the files within the *conf/trajectories/left* or
        *conf/trajectories/right* folder. Each trajectory file contains
        only one trajectory, the name of the file corresponds to the
        trajectory name. Moreover, the list of filenames of the
        *conf/trajectories/finger_poses* folder is also stored. This
        list contains the list of trajectory names corresponding to the
        trajectories defined using the fingertips positions and not the
        joint position values. The constructor prints a list of the
        available configurations and trajectory names.

-   **Constructor for a Simulated Hand.**

    Similar to the real hand constructor but tailored for a simulated
    environment. It only contains one input parameter, the hand
    configuration filename or file path, *\_conf_file*.

    As before, the constructor method for a simulated hand will be
    explained in detail, the steps that are shared with the real hand
    constructor will not be explained in detail.

    1.  Store basic hand information.

        The constructor initializes several member variables to default
        parameters:

        -   *moving* is set to `false`.

        -   *link_prefix* and *joint_prefix* are initialized as empty
            strings.

        -   *q_desired* is initialized to a zero vector.

        -   *status* is set to `true`.

        -   *listening_to_trajectory_to_store* is initialized to
            `false`.

        -   *safety_margin_joint_limits_values* is initialized to a
            vector of 16 elements, all set to `0.0`.

        The constructor initializes the *AllegroHandDriver* constructor
        for the simulated hand by passing the input parameters
        *\_conf_file*. Once this is done, the information extracted from
        the hand configuration file and the corresponding
        URDF will
        be stored. This information involves the hand name, serial
        number, type (right or left) and version, the communications
        parameters (encoder offset, encoder direction and motor
        direction), the filter coefficients for the controllers, the
        maximum torque values and the URDF information as an string. Also,
        the *conf* folder directory will be stored in the class variable
        *directory_conf_folder*.

        This part is very similar to the real constructor, although a
        very important difference occurs. Since the aim is to run a
        simulation, the communications will not be initialized because a
        real hand is not being used. For this reason there is no
        instance of the *CAN_AH* class.

    2.  Read the URDF and instantiate the kinematics
        classes.

        This part is identical to the previous constructor part. The
        URDF file
        is read to extract the information related to joint limits, the
        coordinate position of the joints and the lengths of the finger
        links. Also, the
        kinematic chains of the hand are stored. Moreover,
        four *AllegroFinger* instances are initialized, three
        *IMAFinger* classes and one *ThumbFinger* class.

    3.  Set the initial Fake controller.

        The simulation constructor differs from the real one in the
        controller part since it can only have one available controller.
        The only available controller when running in simulation is the
        Fake controller. The *AllegroHandControllerFake* class is
        instantiated through the method *setController()*.

    4.  Initialize the control thread.

        Since there is no real hand connected to the computer system,
        this constructor part is also different.

        The control thread, responsible for running the control loop,
        *controlThread*, is initialized. Although, instead of the
        *controlLoop()* function, the *controlLoopFake()* function is
        started in the control thread. By launching the control loop in
        a separate thread, the loop continues running without being
        blocked by the main program operations.

        The *controlLoopFake()* method implements the main and only
        control loop of the *AllegroHand* when running in simulation. It
        continuously executes the *executeController()* method, to
        compute the desired torques. Finally, the calculated torques are
        stored in the class *torque* variable using the
        *setDesiredTorque()* method, that ensures that the torques are
        within the joint limits. This loop uses mutexes to update the
        previous hand configuration and the status flag, *q_prev* and
        *status*. In case the *status* flag is `false`, then the control
        loop stops.

    5.  Read predefined configurations and trajectories.

        Finally, this part is identical to the *AllegroHand* constructor
        for the real hand. Where the constructor reads and stores the
        predefined configurations and trajectories names from the YAML
        files.

-   **Empty Constructor.**

    The constructor has no inputs and initializes several member
    variables as follows:

    -   Inherits from the *AllegroHandDriver* empty constructor.

    -   *moving* is set to `false`.

    -   *link_prefix* and *joint_prefix* are initialized as empty
        strings.

    -   *q_desired* is initialized to a zero vector.

    -   *status* is set to `true`.

    -   *listening_to_trajectory_to_store* is initialized to `false`.

    -   *safety_margin_joint_limits_values* is initialized to a vector
        of 16 elements, all set to `0.0`.

    The constructor body is empty, meaning no additional operations are
    performed during initialization. The empty constructor only provides
    default values.

The body parts of the constructor for the real hand and for the
simulated hand are grouped in the *setUpAH()* method. This method has
one input parameter, a boolean that indicates whether the hand is
simulated, `true`, or not, `false`.

**Key Methods**

Previously, the key methods of the *AllegroHand* Class have been stated.
Now, these key methods will be explained in detail. The *AllegroHand*
methods can be grouped into five different groups.

-   **Control algorithms:**

    There are five controller algorithms available, each one defined in
    a derived class of the *AllegroController* class. In order to avoid
    having five instances of each controller, each time a new controller
    is selected, first the previous instance is deleted and then the new
    instance is created. This ensures efficiency memory management. To
    ensure that the controller methods are not called while there is no
    instance of any controller, variables such as *controller_available*
    are always accessed using mutex. The class variable
    *controller_available* is a flag that regulates the use of the
    controller features.

    The *AllegroHand* class can search for all the hand available
    controllers in the *conf/controllers* folder, set the desired
    controller and get the current controller being used. Additionally,
    there are functions related to the gravity vector of the hand with
    respect to the base of the hand. The key methods related to the
    controllers being used are listed below.

-   **Hand configuration:**

    These methods are related to the reading and setting the current and
    desired joint positions, respectively. Additionally, the methods
    related to moving the hand to the desired joint positions are listed
    below, along the functions that allow to read, store and delete
    predefined hand configurations.

    1.  **getCurrentState():** Retrieves the current states, this is the
        current positions \[rad\], velocities \[rad/s\], accelerations
        \[rad/s²\] and torques \[Nm\] of each one of the 16 joints of
        the Allegro Hand.

    2.  **getStatus():** Retrieves the current hand status.

    3.  **getError():** Returns the joint position errors, the error is
        computed as the difference between the desired and the current
        joint positions.

    4.  **readPredefinedConfigurations():** Stores the hand
        configurations defined in the desired YAML file. This file can
        contain several hand configurations, each one defined with a
        name and 16 joint values. The YAML file path is provided, and
        the configurations are stored in a variable that maps the
        configuration name with the corresponding joint position values.

    5.  **getConfList():** Retrieves the list of hand configurations
        stored in the class variable *configurations_list*.

    6.  **addConf():** Stores a new hand configuration. The name of the
        configuration as well as the corresponding joint position values
        need to be provided. The addition can be temporarily, by adding
        the hand configuration into the class variable
        *configurations_list*, or permanent by writing the configuration
        into the YAML files defined in the constructor part, using the
        *writeConfiguration()* method. In the permanent case the desired
        configuration will be added, and the corresponding hand
        configuration of the opposite hand will be computed using the
        *getConfigsForTheOppositeHand()* function and then stored.

    7.  **delConf():** Deletes a hand configuration from the class
        variable *configurations_list*.

    8.  **storeCurrentConf():** Stores temporarily or permanently the
        current hand configuration. The current configuration is read
        and then it is passed to the *addConf()* method along with the
        configuration name.

    9.  **setDesiredConfiguration():** The joint position values
        provided are stored in the class variable *q_desired*. It
        ensures that the values are within the joint limits. The control
        loop will move the hand towards the desired configuration, when
        the current controller being used is either the
        PD or the
        PID
        controller. There are two possible options, the whole hand
        configuration can be provided or just one finger joint position
        values and the corresponding finger index. In the second case,
        and since the Allegro Hand needs to continuously receive all the
        hand joint position values, the rest of the finger joint
        position values will be identical as the current values.

    10. **moveToConf():** Moves the hand to the desired joint
        configuration. The hand configuration can be provided by using
        the configuration name or the joint position values. In case
        that the current controller is either the Torque or the Gravity
        Compensation controller, the method sets the
        PID as the
        current controller, through the *setPIDController* function.
        Then, it provides the joint position values to the
        *setDesiredConfiguration()* method. Additionally, this method
        can have another input variable defining the velocity percentage
        at which we desire to move the hand. In that case, it computes
        the hand trajectory, using the *calculateTrajectory()* method
        which will be explained later. Until the final configuration is
        achieved, each hand configuration of the computed trajectory is
        sent at every time interval.

-   **Grasps modes:**

    The *AllegroHand* has some predefined hand trajectories in the
    *conf/trajectories* folder. Grasps are defined as trajectories. Each
    trajectory YAML file contains an orderly fashioned list of the hand
    configurations. The filename corresponds to the trajectory name.
    Since the AllegroHand periodically receives messages, the
    trajectories are computed considering the communications time
    period. Moreover, trajectories can also be defined through a list of
    the fingertip positions and orientations.

    The list below shows the key methods related to reading and storing
    trajectories, as well as moving along trajectories.

    1.  **readTrajectories():** Retrieves the list of the names of the
        available trajectories (defined by a list of hand
        configurations). This method reads the filenames of all the
        files located in the *conf/trajectories/left* or
        *conf/trajectories/right* folder. The list is stored in the
        class variable *trajectories_list*.

    2.  **readTrajectory():** Retrieves the trajectory corresponding to
        the given trajectory name. To do so, it reads the YAML
        trajectory file. A list with the hand configuration values is
        obtained.

    3.  **readPosesTrajectory():** Retrieves the trajectory
        corresponding to the given trajectory name. To do so, it reads
        the YAML trajectory file. A list with the fingertip
        poses is
        obtained.

    4.  **writeTrajectories():** Creates a new YAML file containing the
        provided trajectory (list of hand configurations). The analogous
        trajectory for the opposite hand is also computed and stored in
        a YAML file.

    5.  **writePosesTrajectory():** Creates a new YAML file containing
        the provided trajectory (list of fingertip
        poses).

    6.  **getTrajectoriesListNames():** Retrieves a list with the names
        of the predefined hand trajectories (defined by a list of
        configurations). The class variable *trajectories_list* is used.

    7.  **getTrajectoriesPosesListNames():** Retrieves a list with the
        names of the predefined hand trajectories (defined by a list of
        fingertip poses).

    8.  **storeCurrentTrajectory():** Creates a new YAML file containing
        the new trajectory (list of hand configurations).

        The input parameters include the desired name of the new
        trajectory and the maximum listening time. The method sets the
        Gravity Compensator controller to allow free movement of the
        hand. Then it starts a clock, and the flag variables *moving*
        and *listening_to_trajectory_to_store* are set to `true`. While
        the *listening_to_trajectory_to_store* flag is `true`, the
        method continuously stores the current hand configuration,
        *q_current*, computes the current time to calculate the elapsed
        time since the method started and checks whether the elapsed
        time exceeds the maximum allowed listening time. The previous
        loop waits 3/4 of the communications time period between
        iterations. Also, before storing a new hand configuration,
        checks that is not identical to the previous hand configuration
        stored. Finally, the corresponding trajectory for the opposite
        hand is computed and both trajectories are stored into a new
        YAML file, using the *writeTrajectories()* method.

        Listens and reads the current hand positions and store a
        trajectory. Reads for a defined time, but it can also stop
        listening with the stopListeningCurrentTrajectory() function.

    9.  **calculateTrajectory():** Calculates a trajectory defined by a
        list of hand configurations. There exist several ways of
        calculating the trajectory depending on the information
        provided:

        -   Given the initial hand configuration, the final hand
            configuration and a desired velocity percentage. In this
            case, the method calculates the difference between initial
            and final desired joint values, using the largest difference
            to determine the number of intervals needed to reach the
            final hand configuration at the desired speed, considering
            the sample period. Then it computes the differential values
            for each joint, creating a list of interpolated
            configurations between the initial and final configurations.
            All fingers reach the final position simultaneously.

        -   Given the final hand configuration and a desired velocity
            percentage. The procedure is identical to the previous one,
            although the initial hand configuration value is defined
            with the current joint position values.

        -   Given a trajectory and a maximum velocity value in rad/s.
            Slows down the trajectory. It considers the sample time and
            the maximum velocity to calculate the maximum allowed
            difference between two consecutive joint position values of
            the trajectory, this will be the threshold. Then for each
            hand configuration of the trajectory, it checks that the
            maximum joint difference between the current and the
            previous hand configuration of the trajectory is smaller
            than the threshold. If not it computes and adds to the
            trajectory the necessary hand configurations to adapt the
            velocity.

    10. **ConvertTrajectoryFromConfigurationToPoses():** Converts a
        trajectory defined by a list of hand configurations into a
        trajectory defined by a list of fingertip
        poses,
        through the use of the *computeFK()* method.

    11. **moveAlongTrajectory():** Moves the hand along a trajectory
        defined by a list of hand configurations. The method first
        checks the current controller being used. In case that the
        current controller is either the Torque or the Gravity
        Compensation controller, the method sets the
        PID as the
        current controller, through the *setPIDController* function.
        This method is only executed in case the class flag *moving* is
        set to `false`. If so, the method sets *moving* to `true` and
        starts a loop, that is going to be executed as long as *moving*
        is `true`. Once the loop finishes, *moving* is set to `false`.

        The loop updates the hand desired joint position values,
        *q_desired* using the *setDesiredConfiguration()* method. At
        each iteration, the joint positions of a hand configuration of
        the trajectory is sent, until the final hand configuration is
        achieved. Then, the loop is paused, the time that is paused is
        equal to the control time period.

        There exist several ways of executing this method depending on
        the information provided:

        -   Given the trajectory.

        -   Given the trajectory name. The trajectory is obtained
            through the *readTrajectory()* method.

        -   Given the trajectory and a maximum velocity value in rad/s.
            The new slowed down trajectory is computed using the
            *computeTrajectory()* method.

        -   Given the trajectory name and a maximum velocity value in
            rad/s. The trajectory is obtained through the
            *readTrajectory()* method and then slowed down using the
            *computeTrajectory()* method.

        -   Given the initial and final joint positions and a velocity
            percentage value. It uses the *moveToConf()* method.

    12. **moveAlongPosesTrajectory():** Moves the hand along a
        trajectory defined by a list of fingertip
        poses.
        Similar to the *moveAlongTrajectory()* method, although it uses
        the *executeIK()* method instead of the
        *setDesiredConfiguration()* method.

    13. **moveGraspPrismatic3():** Executes a grasp using 3 fingers, the
        Index, the Middle and the Thumb. The initial and final values of
        the grasp are hardcoded in the method. The
        *moveAlongTrajectory()* method is executed.

    14. **moveGraspPrismatic4():** Executes a grasp using all fingers.
        The initial and final values of the grasp are hardcoded in the
        method. The *moveAlongTrajectory()* method is executed.

    15. **moveGraspPinchIndexThumb():** Executes a grasp using 2
        fingers, the Index and the Thumb. The initial and final values
        of the grasp are hardcoded in the method. The
        *moveAlongTrajectory()* method is executed.

    16. **moveGraspPinchMiddleThumb():** Executes a grasp using 2
        fingers, the Middle and the Thumb. The initial and final values
        of the grasp are hardcoded in the method. The
        *moveAlongTrajectory()* method is executed.

    17. **coupledClosing():** Performs a copuled closing, the joint
        position values are incremented so the hand closes. There are
        several ways of executing this method depending on the
        information provided:

        -   Given 16 coupling values \[º\], one per each joint. Performs
            a coupled closing of all the joints at the desired couple
            values. The couple value is converted first into radians.

        -   Given a coupling value \[º\]. Couples joints 1, 2 and 3 of
            the IMA
            fingers and joints 2 and 3 of the Thumb finger. These joints
            are coupled by adding the same couple value which is first
            converted into radians.

        -   Given 4 coupling values \[º\], one per each finger. Couples
            joints 1, 2 and 3 of the IMA fingers and joints 2 and 3 of
            the Thumb finger. The couple values of each finger is
            different. The couple value is converted first into radians.

        -   Default. When no input parameters are provided, the method
            couples 5º joints 1, 2 and 3 of the
            IMA
            fingers and joints 2 and 3 of the Thumb finger.

    18. **isMoving():** Retrieves the value of the class variable flag
        *moving*. It contains the information of whether the hand is
        currently moving along a trajectory.

-   **Kinematics:**

    The key functions related to kinematics involve computing the
    FK, which return
    the current position and orientation of the fingertips and executing
    the IK in order
    to move the fingertips to the desired poses or to the desired fingertip position
    with the desired fingertip normal vector. Moreover, there are
    functions related to obtaining the Jacobian matrix.

    1.  **computeFKFinger():** Retrieves the fingertip position and
        orientation of the desired finger. To do so, it computes the
        Forward kinematics of the current joint position values of the
        finger. To do so, it uses the *AllegroFinger* *Fkine()*
        function. The resulting fingertip position and orientation is defined in the finger base
        frame. Then it is defined with respect to the base of the hand
        frame.

    2.  **computeFK():** Retrieves all the fingertips positions and
        orientations. To do so, the *computeFKFinger()* method has been
        used.

    3.  **computeIKFinger():** Retrieves the joint values of the desired
        finger to allocate the fingertip at the desired position and
        orientation. To do so, it computes the Inverse kinematics using
        the *Ikine()* function.

    4.  **computeIKHand():** Retrieves the joint values of the hand to
        allocate the fingertips at the desired positions and
        orientations. To do so, the *computeIKFinger()* method has been
        used.

    5.  **executeIK():** Moves the hand in order to allocate the
        fingertips at the desired positions and orientations. To compute
        corresponding joint values, the *computeIKHand()* method has
        been used. Then, the hand is moved using the *moveToConf()*
        method.

    6.  **ComputeJacobian():** Computes the Jacobian matrix using the
        *AllegroHandController* *ComputeJacobian()* method.

    7.  **getRealJacobianKDL():** Retrieves the computed Jacobian
        matrices either for all fingers or a specific finger, it uses
        the *AllegroHandController* *getRealJacobianKDL()* method.

    8.  **getFingerFrames():**Retrieves the positions and orientations
        of all the frames of the
        kinematic chain obtained using the
        *AllegroHandController* *getFingerFrames()* method.

-   **Joint limits:**

    1.  **updateJointLimitsURDF():** Computes the joint limits of the
        URDF
        considering the real joint limit values of the manufactured hand
        stored in a YAML file. These are called the unsaturated
        URDF joint
        limits, and are stored in the *q_min_urdf_unsaturated* and
        *q_max_urdf_unsaturated* class variables. The method starts by
        considering the URDF joint limit values. In case there
        exists a file containing the real joint limits of the
        manufactured hand in the */conf/joint_limits* folder, these
        values are compared. For each joint, in case the
        URDF joint
        limit values are outside the range of the real joint limit
        values of the hand, then the corresponding values are changed to
        match the real joint limit values.

    2.  **updateJointLimits():** Updates the joint limit values, stored
        in the *q_min* and *q_max* variables. It considers the safety
        margin and the unsaturated URDF joint limit values.

    3.  **getJointLimits():** Retrieves the minimum and maximum allowed
        joint position values stored in the class variable *q_min* and
        *q_max*, respectively.

    4.  **getJointLimitsSafetyMargin():** Retrieves the safety margin of
        the joint limit values.

    5.  **setJointLimitsSafetyMargin():** Sets the margin of the joint
        limit values.

    6.  **useSafetyMargin():** Chooses to use or not the safety margin
        for the joint limit values. The method then uses the
        *updateJointLimits()* function to update the joint limits
        accordingly.

---
> - 🔙 Go back to the main README file: [0 - Main](../README.md#the-primary-class-allegrohand)
> - ⬅️ Previous section: [7 - The Basic Driver Class: AllegroHandDriver](7_DriverClass.md)
> - ➡️ Next section: [9 - Installation And Setup](9_InstallationAndSetup.md)
---
