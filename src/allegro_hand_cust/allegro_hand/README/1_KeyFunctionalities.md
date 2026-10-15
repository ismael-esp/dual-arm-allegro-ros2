# 1 - Key Functionalities
---
> - ⬅️ Previous section: [0 - Main](../README.md)
> - ➡️ Next section:     [2 - Code Structure](2_CodeStructure.md)
---

The Allegro Hand Library is a powerful and versatile software package
that provides a wide range of features and functionalities to facilitate
the control and operation of the Allegro robotic hand.

-   **Suitable for Real-Time Control:** The software is developed to be
    able to work on real-time. Although currently it does not work on
    real-time yet because the operating system used is not real-time,
    thus due to the kernel it can not guarantee the sample time in
    between operations. Nevertheless, it allows users to obtain the
    current joint positions, as well as moving the hand to a desired
    configuration or along a trajectory. Before initializing the hand
    movement, it checks whether the Gravity Compensator or the Torque
    controller are being used, if so, it switches to a
    PID controller.

-   **Simulation and Real-World Operation:** The library supports
    operating a real Allegro Hand and also can perform a simulation
    environment. Users can test and validate in simulation before using
    the physical Allegro Hand, this reduces development time and ensures
    safety operations.

-   **Control Algorithms:** Users can select the controller algorithm,
    depending on the task being performed. The available controllers
    are:

    -   **Torque Controller:** The Torque controller directly controls
        the torque applied to each joint. It is a very simple controller
        that maintains the torques.

    -   **Gravity Compensator Controller:** The Gravity Compensator is a
        specialized controller that counteracts the effects of gravity
        on the hand's joints. By applying the necessary torques to
        neglect gravitational forces, enables the hand to maintain its
        position, making it easier to manually manipulate the hand or
        hold a pose (position + orientation).

    -   **Joint Position PD Controller:** Operates by
        calculating the error between the desired and current joint
        positions and applying a corrective torque based on the
        proportional (P) and derivative (D) terms. The proportional term
        addresses the magnitude of the error, while the derivative term
        counteracts the rate of change of the error, thus improving
        system stability and reducing overshoot. Moreover, it also
        considers the gravitty effects.

    -   **Joint Position PID Controller:** The PID
        controller extends the PD controller by adding an
        integral (I) term, which accounts for the cumulative error over
        time, thus it handles the steady-state error. Experimentally,
        the PID controller is the most precise one, as it can compensate for
        both immediate and long-term discrepancies between the desired
        and current joint positions. Moreover, it also considers the
        gravitty effects.

    -   **Fake Controller:** The fake controller is a controller for the
        simulation case.

-   **Forward and Inverse Kinematics:** Contains functions for computing
    the full Forward Kinematics and Inverse Kinematics. Enables users to
    calculate the current positions and orientations of the fingertips
    (FK) and to move the fingertips to the desired positions and orientations
    (IK). An essential feature for grasp and manipulation applications.

-   **Predefined and Customizable Hand Configurations:** Users can
    define and store, temporarily or permanently, custom hand
    configurations. The analogous physical configuration between the
    right and the left hand, do not correspond to the same joint values
    due to differences in joints 0 of the Index, Middle and Annular
    (IMA) fingers.
    These discrepancies are accounted for, and, upon saving a hand
    configuration, the corresponding appropriated hand configuration for
    the opposite hand is calculated and stored. This feature is
    particularly valuable for applications that require repetitive tasks
    or specific hand postures. Additionally, a set of predefined hand
    configurations is provided.

-   **Predefined and Customizable Grasping Modes:** A selection of
    grasps that allow in-hand manipulation, specifically all types of
    prismatic precision grasp, have been already
    implemented. Additionally, users can store grasps and hand
    trajectories. By either providing the final configuration and the
    desired speed or by recording the current hand state during a period
    of time while the hand is being manually manipulated.

-   **Open Source and Modular Design:** It is an open source library
    designed to be compatible with various operating systems, including
    Linux-based platforms. The library is built with a modular
    architecture, it decouples the drivers (communications and control)
    from the framework, thereby facilitating integration with other
    frameworks such as ROS 1, ROS 2, Orocos, and Matlab. The design
    eases software customization, enabling the development of
    specialized applications, such as adding new controllers.

-   **Version Agnostic:** All the current Allegro Hand versions can be
    controlled with this software. Additionally, it is designed to
    facilitate its use for other robotic hands with similar structure,
    such as the Leaphand, requiring only minimal
    modifications in the communications part, dependent on the
    communications protocol.

-   **Detailed Logging and Diagnostics:** The software provides
    extensive logging and diagnostic tools, allowing users to monitor
    the state of the hand, including joint positions, velocities,
    accelerations, and torques. This feature is essential for debugging
    and performance analysis.

-   **Extensive Documentation and Examples:** Comprehensive
    documentation and a variety of example programs are provided, making
    it easier for users and developers to get started. The examples
    demonstrate all the functionalities of the software.

The library's design and capabilities are tailored to support advanced
robotic grasps and manipulation tasks, enabling users to achieve precise
and reliable control of the Allegro Hand. These features make the
Allegro Hand Library a robust and adaptable tool for researchers,
developers, and engineers working with robotic hands.


---
> - ⬅️ Previous section: [0 - Main](../README.md)
> - ➡️ Next section: [2 - Code Structure](2_CodeStructure.md)
---

