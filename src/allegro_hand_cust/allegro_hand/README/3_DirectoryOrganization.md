# 3 - File and Directory Organization

---
> - 🔙 Go back to the main README file: [0 - Main](../README.md#software-walkthrough)
> - ⬅️ Previous section: [2 - Code Structure](2_CodeStructure.md)
> - ➡️ Next section: [4 - The CAN Communications Class: CAN_AH](4_CommunicationsClass.md)
---


The developed software for controlling the Allegro robotic hand is
organized into a comprehensive set of directories and files. Each
directory serves a specific purpose within the overall framework of the
software, ensuring modularity and ease of maintenance.

-   **cmake-modules** directory: This directory contains essential CMake
    files, such as *FindFilesystem*, *FindPCAN*, and *FindPCANBasic*.
    These files are necessary for configuring the build environment and
    locating necessary dependencies.

-   **conf** directory: This directory holds configuration files related
    to the Allegro Hand and is subdivided into several sub directories:

    -   **configuration_files** directory: Contains YAML files detailing
        the initialization parameters required for the Allegro Hand.
        Each manufactured hand has unique parameters. The hand
        configuration file name includes the hand's serial number. The
        YAML files are organized into three primary parameter groups:

        -   *Basic Information*: Includes the serial number, version,
            and DOF
            of the hand, as well as the control period and the
            URDF
            file name.

        -   *Communication Parameters*: Specifies the encoder offset and
            direction and the motor direction of each joint.

        -   *Filter Coefficients*: Contains coefficients for filtering
            joint position and velocity data.

        Additional metadata, such as the manufacturer or origin, can be
        included but is not required for software operation. The naming
        convention for the hand configuration files follows the format:
        `hand_name.yaml`. Currently, the *hand_name* convention used is
        the serial number of the hand, but as long as this structure is
        fulfilled, other developers can choose any desired *hand_name*.

    -   **controllers** directory: This sub directory contains YAML
        files defining the controller parameters for specific hands.
        Parameters for the JointPositionPD and JointPositionPID
        controllers are specified here, including proportional (*k_p*),
        derivative (*k_d*), and integral (*k_i*) gains for each joint.
        The files are named using the format:
        `hand_name + ’_’ + controller_name.yaml`.

    -   **configurations** directory: Stores predefined hand
        configurations within YAML files. The directory is further
        divided into sub directories for right and left hands. Each file
        in these sub directories includes multiple hand configurations,
        each defined by a unique name and 16 joint position values in
        radians. The file names are arbitrary.

    -   **trajectories** directory: This directory is partitioned into
        sub directories for left and right hands, each containing YAML
        files that define trajectories through sequences of hand
        configurations. These configurations consist of 16 joint
        position values in radians. Additionally, the **finger_poses**
        sub directory stores trajectories specified by fingertip
        positions and orientations, represented by 4 sets of XYZ
        coordinates and 4 sets of ZYZ Euler angles, respectively. The
        naming convention for these files is: `trajectory_name.yaml`.

    -   **urdf** directory: Contains URDF files necessary for the hand's
        simulation and operation. The **meshes** sub directory includes
        STL files required for 3D modeling. Different
        URDF files
        exist for right and left hands and for different fingertips. The
        Allegro Hand allows to switch the default tip to add tactile
        sensors (e.g., Weiss, Optoforce). The naming convention for
        these files is:
        `’allegroHand_’ + hand_type + ’_’ + tip_name.urdf`.

    -   **joint_limits** directory: Holds YAML files with the real joint
        limits for a specific hand. These files are automatically
        generated using the *calculate_joint_limits* program, which
        records the minimum and maximum joint values. The naming
        convention for these files is: `hand_name.yaml`.

-   **examples** directory: This directory provides example programs for
    controlling the Allegro Hand. These programs enable users to
    interact with the hand via keyboard input, with available functions
    and corresponding key mappings displayed in the console. Later on,
    these programs will be presented.

-   **include** directory: Contains header files (`.h`) that define the
    classes and methods used in the software.

-   **src** directory: Contains the source code (`.cpp` files)
    implementing the functionality of the classes. The *AllegroHand*
    class alone spans nearly 3000 lines of code, with methods
    distributed across multiple files to enhance readability and
    maintainability. These files include:

    -   *AllegroHand.cpp*: Core implementation of the AllegroHand class.

    -   *AllegroHandConfigurations.cpp*: Manages hand configurations and
        trajectories.

    -   *AllegroHandControl.cpp*: Implements control algorithms.

    -   *AllegroHandJointLimits.cpp*: Related with hand joint limits.

    -   *AllegroHandKinematics.cpp*: Manages kinematic calculations and
        properties.

Additionally, the software package includes:

-   **README** directory: Provides a general overview of the software,
    including system requirements, necessary packages, and libraries. It
    also offers a brief description of the software's structure, key
    classes, and functions, along with instructions for running the
    provided programs. This documentation is crucial for users to
    successfully deploy the software to control the Allegro Hand.

-   **package.xml** file: Specifies the package information, including
    dependencies, build type, and versioning, to facilitate integration
    with ROS.

-   **CMakeLists.txt** file: Defines the build process for the software,
    specifying the source files, include directories, and any necessary
    compilation flags. This file ensures that the software is correctly
    compiled and linked on the target system.

---
> - 🔙 Go back to the main README file: [0 - Main](../README.md#software-walkthrough)
> - ⬅️ Previous section: [2 - Code Structure](2_CodeStructure.md)
> - ➡️ Next section: [4 - The CAN Communications Class: CAN_AH](4_CommunicationsClass.md)
---