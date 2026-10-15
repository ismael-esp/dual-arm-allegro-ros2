# 5 - The Controller Class: AllegroHandController

---
> - 🔙 Go back to the main README file: [0 - Main](../README.md#the-controller-class-allegrohandcontroller)
> - ⬅️ Previous section: [4 - The CAN Communications Class: CAN_AH](4_CommunicationsClass.md)
> - ➡️ Next section: [6 - The Kinematics Class: AllegroFinger](6_KinematicsClass.md)
---

The *AllegroHandController* class contains algorithms to control the
Allegro Hand. The class leverages the Kinematic and Dynamics Library
(KDL), the kinematic_chain of the hand is used when computing the
necessary torque to compensate the gravity effects. Moreover, this class
also computes the Jacobian using the KDL. The software has five
different control modes: Torque, Gravity Compensator,
PD,
PID and Fake
controller. Each controller being a derived class of the
*AllegroHandController* class.

-   **Torque**. Sets and maintains the desired torques.

-   **Gravity Compensator:** Calculates the needed torques to counteract
    the effects of gravity using KDL. It considers the whole hand
    orientation.

-   **Joint Position PD:** Uses a PD control to achieve desired joint
    positions, considering gravity effects.

-   **Joint Position PID:** Uses a PID control to achieve desired joint
    positions, considering gravity effects.

-   **Fake:** Used only for simulation, computes the gravity
    compensation torques and updates the virtual hand state.

First the *AllegroHandController* class will be presented, starting by
the constructor and following up with the key methods. Later on, each
specific controller class will be discussed.

## Constructor

The *AllegroHandController* constructor initializes several key
attributes necessary for the control of the robotic hand.

-   **Kinematic Chains:** The class contains two arrays of KDL chains
    (*finger_chain* and *chain*), the arrays contain the
    kinematic chain of all the fingers, representing the
    kinematic chain of the robotic hand. The former,
    *finger_chain*, can include virtual links, when initialized it
    represents the kinematic chain of the finger without considering the
    very last link, the tip link. While the latter, *chain*, is the
    actual physical chain without these additional segments, including
    all joints and the final tip link. The
    kinematic chains are initialized with the values
    provided from input parameters.

-   **Gravity Vector:** The *gravity_vector* variable represents the
    gravity vector with respect to the hand base frame. It is
    initialized with the Earth's gravity value, ($(0, 0, -9.80665)$) ms.
    It is crucial for calculating gravitational compensation forces
    considering the full hand orientation. There are methods for
    updating this variable.

-   **Torque Parameters:** The input torque parameters initialize the
    variables related to the torque conversion constant, *torqueConv*,
    ($$I = motorDirection \cdot k \cdot \tau$$) and
    a vector representing the maximum allowable torque for each joint,
    *torque_max*.

-   **Filtering Coefficients:** The *filtCoeffsPos* and *filtCoeffsVel*
    vectors store coefficients for filtering the position and velocity
    of the joints, aiding in smoother control. These variables are
    initialized with the input constructor parameters.


## Key Methods

The *AllegroHandController* class key methods are presented below. These
are all the public methods and the protected method
*computeGravityCompensationTorque()*. 

1.  **executeController():** Executes the control algorithm and returns
    the torque computed by the controller. Starts by calling the method
    *setValues()* to update the controller's internal state. The
    *setValues()* function updates the desired torque values,
    *torque_desired*, and the values for joint positions regarding the
    previous, the current and the desired joint values in radians,
    *q_prev*, *q_current* and *q_desired*, respectively. It uses a mutex
    to ensure thread safety when updating these shared variables. Then
    it invokes the *control()* method, this method applies the control
    algorithm to compute the torque of the robotic hand. The control()
    method is overridden by all the controllers derived classes to
    include their own control algorithm. Finally, after control
    execution, the method returns the calculated torque.

2.  **getGravityVector():** Retrieves the gravity vector
    (${ g_{x}, g_{y}, g_{z} }$) of the hand, currently stored in the
    private variable *gravity_vector*.

3.  **setGravityVector():** Sets the gravity vector value with respect
    to the hand base frame (${ g_{x}, g_{y}, g_{z} }$),
    *gravity_vector*.

4.  **computeGravityCompensationTorque():** This protected method,
    computes the torque required to compensate for gravitational forces
    acting on the robotic hand's joints. The method iterates through
    each finger of the hand. The required torque ($\tau$) is computed
    using the Kinematics and Dynamics Library (KDL), which constructs
    the kinematic and dynamic models of the Allegro Hand from the
    URDF
    configuration file, *chain*. For each finger, it uses the KDL
    Recursive Newton-Euler solver to compute the gravity compensation
    torque, providing it with the gravity vector, *gravity_vector* and
    the physical finger
    kinematic chain, *chain*. The computed torques are
    then stored in the private vector *gc*.

5.  **getGravityComp():** Retrieves the torque needed to compensate the
    gravity effects, *gc*. For optimal results, the private
    *computeGravityCompensationTorque()* method should have been
    executed first.

6.  **getCurrentVel():** Returns the calculated velocity vector of the
    hand.

7.  **getCurrentAcc():** Returns the calculated acceleration vector of
    the hand.

8.  **ComputeJacobian():** Computes the Jacobian matrix for the fingers
    of the hand. The kinematic chain of the fingers without considering
    the tip link, *finger_chain* is used. For each finger, a virtual
    segment, between joint 2 and the fingertip contact point, is added
    to the finger kinematic chain. Then, the KDL Jacobian and Forward
    Kinematics solvers are used to compute the Jacobian and the finger
    frames, which are then stored in private class variables,
    *RealJacobianKDL* and *FingersFrames*,respectively.

9.  **ComputeFingerFrames():** Computes the Forward Kinematics for the
    fingers of the Allegro hand and returns the frames (positions and
    orientations) of each segment in the
    kinematic chain. Similarly to the *ComputeJacobian()*
    method, a virtual segment, that depends on the location of the
    contact point, is added to the finger
    kinematic chain that does not consider the fixed tip
    link, *finger_chain*. Then, the KDL Forward Kinematics solvers is
    used to compute the finger frames, which are then stored in the
    private class variable *FingersFrames*.

10. **getRealJacobianKDL():** Retrieves the computed Jacobian matrices,
    *RealJacobianKDL*, either for all fingers or a specific finger. For
    optimal results, the *RealJacobianKDL()* method should have been
    executed first.

11. **getFingerFrames():** Retrieves the positions and orientations of
    all the frames of the
    kinematic chain, *FingersFrames*, considering the
    contact point as the final link. For optimal results, either the
    *RealJacobianKDL()* method or the *ComputeFingerFrames()* should
    have been executed first.

12. **getKDLChain():** Retrieves the KDL chain for a specified finger.
    It can return the physical kinematic finger chain, including the
    fixed final tip link, *chain*, or the
    kinematic chain that has a virtual segment
    considering the current contact point as the last link,
    *finger_chain*. For the last option optimal results, either the
    *RealJacobianKDL()* method or the *ComputeFingerFrames()* should
    have been executed first.


### Related classes

The *AllegroHandController* class integrates kinematic and dynamic
modeling and the necessary methods to neglect the gravity effects. It
has been designed to simplify the process of adding new controllers,
since it has a controller structure as well as many interesting features
for control algorithms. Each controller algorithm is a derived class of
the *AllegroHandController*. Within each specific controller class, the
previous methods can be used. There are two constraints for the new
controller to work; the constructor needs to be calling the base class
constructor *AllegroHandController* and the *control()* method must be
overridden. This method will be called each time the controller is being
used. It has no inputs, nor any outputs and it is mandatory that the
*torque* class variable is updated within the *control()* method.
Moreover, it is advisable to update it using mutexes, *mutex*. Following
up, a description of each one of the specific controller classes
corresponding to the available controllers will be shown. 



- **AllegroHandControllerTorque Class:** Implements a torque pass-through
open-loop controller, where the desired torque is applied directly to
the joint servos. To do so, the *control()* method updates the *torque*
variable with the desired torques.

- **AllegroHandControllerGravityCompensator Class:** Compensates the
gravity action on the real hand. The *control()* method starts by using
the filtering coefficients, *filtCoeffPos*, to low-pass filter the joint
position values. As a result the CAN noisy readings from the servos have been
filtered. Then the joint position filtered values are introduced in the
*computeGravityCompensationTorque()* function. Finally, the computed
torques necessary torque to neglect the gravity effects depending on the
hand orientation are stored.

- **AllegroHandControllerJointPositionPD Class:** The class employs a
Proportional-Derivative control strategy to manage the joint positions
of a robotic hand. It has an additional input parameter containing the
path to the YAML configuration file that contains the desired
PD gains,
*controller_file_path*. The constructor calls the
*readControllerConfigurationFile()* method, which reads and stores the
PD controller gains
(kp and kd) from the corresponding YAML file.

    In this case, the *control()* method computes the desired torques to
    apply to each joint based on a PD control law, including compensations for
    gravity effects. To do so, it begins by using the filtering
    coefficients, *filtCoeffPos* and *filtCoeffVel*, to filter the current
    joint positions and velocities using a low-pass filter to reduce noise
    and enhance signal stability. The controller then calculates the
    position error, defined as the difference between the desired and actual
    joint positions. This error is used to define the proportional term, kp,
    which directly scales with the magnitude of the error to correct
    deviations. Concurrently, the derivative term, kd, corresponds to the
    rate of change of the error, predicting and mitigating potential
    overshoot by damping the system's response. The controller also adds the
    gravity compensation torques that counteract the gravitational forces.
    Then the method checks that the computed torques are within the
    saturation limits.

- **AllegroHandConrollerJointPositionPID Class:** The class employs a
Proportional-Integral-Derivative control strategy to manage the joint
positions of a robotic hand. Similarly, to the
*AllegroHandControllerJointPositionPD* class, it also has an additional
input parameter containing the path to the YAML file that contains the
desired PID gains,
*controller_file_path*. The constructor calls the
*readControllerConfigurationFile()* method, which reads and stores the
PID controller
gains (kp, ki and kd) from the corresponding YAML file.

    In this case, the *control()* method computes the desired torques to
    apply to each joint based on a PID control law, including compensations for
    gravity effects. To do so, it begins by using the filtering
    coefficients, *filtCoeffPos* and *filtCoeffVel*, to filter the current
    joint positions and velocities using a low-pass filter to reduce noise
    and enhance signal stability. The controller then calculates the
    position error, defined as the difference between the desired and actual
    joint positions. This error is used to define the proportional term, kp,
    which directly scales with the magnitude of the error to correct
    deviations. Concurrently, the derivative term, kd, corresponds to the
    rate of change of the error, predicting and mitigating potential
    overshoot by damping the system's response. The integral term, ki,
    accumulates past errors over time. This integral action is crucial for
    eliminating steady-state errors, ensuring that even small, persistent
    discrepancies are corrected over time. The controller also adds the
    gravity compensation torques that counteract the gravitational forces.
    Then the method checks that the computed torques are within the
    saturation limits.

- **AllegroHandControllerFake Class:** As mentioned before, it is used
when it is working with the simulated hand. The *control()* method
starts by saving the previous iteration's filtered position and velocity
values. Then, the filtering coefficients, *filtCoeffPos* and
*filtCoeffVel*, are used to filter the current joint positions and
velocities using a low-pass filter. Then the joint position filtered
values are introduced in the *computeGravityCompensationTorque()*
function. The computed torques necessary torque to neglect the gravity
effects depending on the hand orientation are stored. Finally, the state
is updated.


The choice of control algorithm significantly impacts the performance
and precision of robotic systems like the Allegro Hand. While the
official BHand library utilizes a PD controller, the new software introduces a
PID controller,
which offers several advantages.

A PD controller
considers the current error (the difference between the desired and
actual joint positions) and the rate of change of this error, providing
proportional and derivative actions. However, PD controllers can struggle with
steady-state errors, where the system does not reach the exact desired
position, particularly in the presence of persistent disturbances. In
contrast, a PID
controller adds an integral component that accumulates past errors,
enabling it to correct steady-state errors more effectively. By
integrating the past error over time, the PID controller enhances the control system
and achieves higher precision and accuracy.

Overall, the PID
controller's approach combines the immediate response of the
proportional term, the predictive capability of the derivative term, and
the cumulative correction of the integral term. This is the reason why a
PID controller is a
better choice for controlling the Allegro robotic hand. Since it
enhances the hand's performance, making it more it a more accurate and
precise.

The user will be able to select which is the control mechanism that best
fits their application. The Gravity Compensator is a very useful
controller when storing configurations or types of grasps, since it
allows to move the hand manually to the desired position. Both the
PD and the
PID are the most
interesting ones, although the use of the PID is more encouraged. These controllers
provide precise, adaptive, and stable control. Thus, are well-suited for
grasping objects and holding a desired position despite external forces.

---
> - 🔙 Go back to the main README file: [0 - Main](../README.md#the-controller-class-allegrohandcontroller)
> - ⬅️ Previous section: [4 - The CAN Communications Class: CAN_AH](4_CommunicationsClass.md)
> - ➡️ Next section: [6 - The Kinematics Class: AllegroFinger](6_KinematicsClass.md)
---
