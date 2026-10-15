# 6 - The Kinematics Class: AllegroFinger

---
> - 🔙 Go back to the main README file: [0 - Main](../README.md#the-kinematics-class-allegrofinger)
> - ⬅️ Previous section: [5 - The Controller Class: AllegroHandController](5_ControllerClass.md)
> - ➡️ Next section: [7 - The Basic Driver Class: AllegroHandDriver](7_DriverClass.md)
---

The *AllegroFinger* class is designed to handle the kinematics of a
single finger of the Allegro robotic hand, including both Forward
Kinematics and Inverse Kinematics.

Two classes inherit from the *AllegroFinger* class, one for the
IMA fingers,
*IMAFinger*, and another for the Thumb, *ThumbFinger*. As it has been
previously stated, the kinematic chain of the IMA fingers is identical, since the physical
structure of these fingers is the same, whereas the Thumb
kinematic chain is different. The implementation allows
to compute the fingertip position (X, Y, Z coordinates) and orientation
(Euler angles) given the joint position values of the finger. It also
allows to compute the joint position values necessary to move the
fingertip to the desired frame, defined by the desired fingertip
position (X, Y, Z coordinates) and orientation (Euler angles), through
the use of the IK.
Additionally, it can also compute the joint position values that achieve
the desired fingertip position (X, Y, Z coordinates) and the desired
fingertip normal vector, through the use of the IK. This class was originally developed by a
student at IOC,
Alex Mauricio Calle Clavel. In this project, some improvements have been
developed, such as new constructors, updating the
kinematic chain values instead of hard coding them and
adding a method to compute the IK of the IMA fingers using the fingertip position and
normal. Regardless, this section will dive deeply into the class for
fully understanding its purpose and functionalities and how it embeds
with the rest of the classes.

## Constructors

There are two constructors available, depending on the information
provided. There is a constructor whose only parameter is the specific
finger selection, that indicates which finger is the class referring to.
The most complete constructor has additional input parameters, such as
the link lengths of the
kinematic chain and the joint limits values.

In case these values are not provided, the class has some default values
to be used. Besides the differences regarding the input parameters, the
constructor structure is identical. The *AllegroFinger* constructor
initializes the DH
parameters specific to the selected finger, which are essential for
kinematic calculations.

Finally, the transformation matrices *T_world_base* and
*T_tcp_fingerTip* are initialized, representing the transformation from
the world frame (the base of the hand) to the base of the finger and
from the Tool Center Point (TCP) frame to the fingertip frame,
respectively.

## Key Methods

The main key methods of the *AllegroFinger* class are listed below.

1.  **Fkine():** Computes the Forward Kinematics of the Allegro finger,
    which involves calculating the fingertip position and orientation
    based on a given set of joint angles. Initially, it checks if the
    provided joint angles are within the feasible physical limits of the
    finger, using the *checkAnglesFeasibility()* method. If the angles
    are invalid, it outputs a warning and returns a zero matrix.

    Otherwise, all the HT matrices of each one of the joints is computed
    using the corresponding Denavit-Hartenberg
    parameters. These matrices describe the transformation from one
    joint to the next in terms of rotation and translation.
    Concurrently, the method multiplies these HT matrices in sequence to
    compute the cumulative transformation from the base of the finger to
    the fingertip. This final matrix represents the position and
    orientation of the fingertip relative to the base of the finger.
    Finally, the matrix is returned as the result of the forward
    kinematics computation of the fingertip position of the finger.

2.  **eul2Tmat():** Converts a given a position (X, Y, Z coordinates)
    and orientation, specified by the ZYZ Euler angles configuration,
    into a homogeneous transformation (HT) matrix, a 4x4 matrix
    combining both rotation and translation. This method is crucial, for
    instance to obtain the transformation matrix from the base of the
    hand to the base of the finger, *T_world_base*.

3.  **getJointAngles():** Retrieves the last computed joint angles of
    the finger. Returns the value of the class variable *joint_angles*,
    that represent the angular positions in radians of the finger's
    joints.

4.  **getTransformationMatrix():** By passing an index, it retrieves the
    corresponding HT matrix, which represents the transformation between
    consecutive joints.

5.  **getTotalTransformationMatrix():** Returns the complete HT matrix
    that represents the transformation from the base of the finger to
    its fingertip. This matrix encapsulates the cumulative
    transformations through all joints and links, providing the
    position and orientation of the
    fingertip with respect to the base of the finger.

6.  **getTF_w2b():** Returns the static HT matrix that relates the world
    coordinate frame, this is the base of the hand, to the base of the
    finger.

7.  **getTF_tcp2tip():** Retrieves the static homogeneous transformation
    matrix that represents the transformation from the
    TCP to its
    fingertip.

**Related classes**

As it has been previously stated, the IMA fingers
kinematic chain is shared among these fingers, whereas
the Thumb kinematic chain is different. This results in having two
child classes that inherit from the *AllegroFinger* class, one for the
IMA fingers,
*IMAFinger*, and another for the Thumb, *ThumbFinger*. Both classes use
the AllegroFinger constructor by passing the class input parameters.

The Inverse Kinematics procedure is different for each class, thus the
specific methods of the class are related to computing the
IK. Depending on the
input provided there are two types of IK procedures to compute the joint position
values:

-   Providing the fingertip desired frame, defined by the desired
    fingertip position (X, Y, Z coordinates) and orientation (Euler
    angles), defined with respect to the base hand frame.

-   Providing the fingertip desired fingertip position (X, Y, Z
    coordinates) and the desired fingertip normal vector, defined with
    respect to the base hand frame.

The reason for the existence of this two IK procedures lies on the fact that the
former one, considering a desired frame, was previously developed by
another student, Alex Mauricio Calle Clavel. Then while working on
applying grasping theoretical methods, it was required to move the
fingertips into a desired position and normal. Which lead to
reconsidering the IK procedure, since due to the physical structure of the hand only one
finger configuration could achieve the desired fingertip position and
normal. Moreover, the two methods are different since the first one
focuses on optimization procedures, computing the
IK in different
stages, while the new IK method uses geometrical procedures. To
differentiate both methods the former one is called *Ikine()*, while the
new method is called *IkineNormal()*.

-   **IMAFinger**

    1.  **Ikine():** The method starts by converting the desired
        fingertip pose, into a HT matrix, using the
        *eul2Tmat()* function. The obtained matrix is defined with
        respect to the base hand frame. This matrix is converted into
        the finger's base frame. Then, the method extracts the
        translation part and the rotation part of the transformation
        matrix, which is used to obtain the Euler angles. These angles
        represent the desired orientation of the fingertip in the
        finger's base frame.

        Before proceeding with IK computation, the method checks if
        the desired fingertip position is within the finger's reachable
        workspace, using the *checkPointInsideWs()* function. The
        reachable workspace was computed in Matlab and hardcoded into
        the class. If the position is outside the workspace, the method
        terminates early displaying the error into the console.

        The IK
        calculation process is divided in 3 stages.

        -   Stage 1. First it attempts to find a direct
            IK
            solution with the initial desired orientation with respect
            to the finger base frame. To do so, first calculates the
            possible solutions for the first joint, theta_0, using
            trigonometry, ensuring that the solution is within the
            feasible range. Next, the method computes the translation
            vector from the first joint to the goal frame. Which is used
            to compute the remaining joint angles, through the Siciliano
            method for solving the 3 link planar arm
            IK
            problem. As a result, two potential solutions for each joint
            angle are obtained due to the nature of trigonometric
            functions used. The computed joint angles are checked for
            feasibility, to ensure that the values are within the
            mechanical limits of the joints.

            The Siciliano method stated that in case the *subtraction*
            variable is negative there does not exists a feasible
            solution. This method iteratively changes the desired
            fingertip orientation, using a small reorientation increment
            and recomputes the joint angles (theta_1, theta_2 and
            theta_3) until a feasible solution is found or all
            possibilities are exhausted.

            If the solutions are not feasible, the method proceeds with
            Stage 2.

        -   Stage 2. In case no feasible solution has been found, the
            desired frame orientation is switched. This stage computes
            the IK
            so that the Z axis of the given desired fingertip frame
            matches the Z axis of the new desired fingertip frame. It
            proceeds similarly to stage 1, it introduces small
            orientation adjustments and recomputes
            IK to
            obtain the remaining joint angles (theta_1, theta_2 and
            theta_3), until a feasible solution is found or all
            possibilities are exhausted. The computed joint angles are
            checked for feasibility, to ensure that the values are
            within the mechanical limits of the joints.

            If the solutions are not feasible, the method proceeds with
            Stage 3.

        -   Stage 3.

            Analogously to stage 2, the desired orientation frame is
            rotated. The joint angles are found by continuously
            adjusting the orientation and recomputing the
            IK when
            computing theta_1, theta_2 and theta_3 until a feasible
            solution is found or all possibilities are exhausted. The
            computed joint angles are checked for feasibility, to ensure
            that the values are within the mechanical limits of the
            joints.

        The method finally returns the computed joint angles in case a
        feasible solution has been found, otherwise an error is
        displayed into the console. The final orientation of the
        fingertip frame may not be the same as the desired orientation
        proposed.

    2.  **IkineNormal():** The function is responsible for computing the
        joint angles required to achieve a specified fingertip position
        and normal vector, relative to the base of the hand. First, sets
        the input with respect to the base of the finger, then
        determines the plane that includes the $Z$-axis of the finger's
        base, the desired contact point, and the fingertip's normal
        vector. The angle between this plane and the $XZ$ plane provides
        the value for joint 0, the only joint that rotates along the
        $Z$-axis. If this angle falls within the joint's physical
        constraints, the remaining joints are treated as a planar 3-link
        arm, allowing the function to compute the necessary joint angles
        to achieve the desired fingertip position and normal.

    
        The IK
        problem for the IMA fingers given the fingertip
        position and normal was thoroughly detailed previously, where
        the mathematical foundation and implementation approach were
        described. The function ensures that the computed joint angles
        respect the physical limits of the Allegro Hand, returning a
        boolean value indicating the success or failure of the
        IK  computation.

-   **ThumbFinger**

    1.  **Ikine():** The method starts by converting the desired
        fingertip pose, into a HT matrix, using the
        *eul2Tmat()* function. The obtained matrix is defined with
        respect to the base hand frame. This matrix is converted into
        the finger's base frame. Then, the method extracts the
        translation part and the rotation part of the transformation
        matrix, which is used to obtain the Euler angles. These angles
        represent the desired orientation of the fingertip in the
        finger's base frame.

        Before proceeding with IK computation, the method checks if
        the desired fingertip position is within the finger's reachable
        workspace, using the *checkPointInsideWs()* function. The
        reachable workspace was computed in Matlab and hardcoded into
        the class. If the position is outside the workspace, the method
        terminates early displaying the error into the console.

        The position components are extracted and used to find the
        IK solution
        of the thumb, through the method *systemEquationsSolver()*. This
        solution was implemented by Alex Mauricio Calle Clavel.

---
> - 🔙 Go back to the main README file: [0 - Main](../README.md#the-kinematics-class-allegrofinger)
> - ⬅️ Previous section: [5 - The Controller Class: AllegroHandController](5_ControllerClass.md)
> - ➡️ Next section: [7 - The Basic Driver Class: AllegroHandDriver](7_DriverClass.md)
---
