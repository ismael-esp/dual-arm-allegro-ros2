# Allegro hand ROS

Repository for the Allegro hand library ROS layer.
The Allegro Hand motion is mainly managed by the *allegro\_hand* class, which is wrapped
by the *allegro\_hand\_ros* that incorporates it to the ROS framework.

## Allegro Hand ROS Basic Features

It is a ROS communication layer of the *allegro\_hand* class, that provides *joint\_states* topic and the service functionality. 
The constructor of the *allegro\_hand\_ros* class needs the following
arguments to be initialized:

-   **ROS Node Handle**. It manages the execution of one node inside a *roscpp* program.
-   **Handed prefix**. It is necessary when two *allegro\_hand\_ros*
    classes are initialized. In a project, where the dual-arm
    manipulation structure is formed with two Allegro Hand, it is necessary
    two differentiate with a handed prefix both *allegro\_hand\_ros*
    instances, to avoid collision problems.
-   **Slave Flag**. It modifies the performance of the
    *allegro\_hand\_ros* class. If the flag is set to *True*, the class
    will behave as a slave of a master node that instance it. This
    means, that the class can not modify the configuration or the motion
    of itself, unless the parent node does not ask for it, the master
    manages the control loop. Despite that, if the flag is set to
    *False*, the class will behave as a master, enabling the
    modification of its configurations and allowing the user to directly set new joint configurations.

Furthermore, the constructor of the class take parameters from the
*parameter* server, which are set on the launch files that executes the *allegro\_hand\_ros* class. These are required for the initialization of the *allegro\_hand* class. They are the following ones:

-   **Yaml File**. It sets the *yaml* configuration file that the
    *allegro\_hand* class will use to construct the KDL model, from the
    URDF indicated, and to configure the servos. 
-   **Path Can**. It sets the path to which it is the AH device
    connected. As it was mentioned in the previous subsection, the
    communication channel of AH is the *CAN* one. Two alternatives can
    be used, the *Universal Serial Bus (USB)* and the *Peripheral
    Component Interconnect (PCI)* communication interfaces. 
-   **Channel**. It sets the *CAN*-based *PCI* channel used for each one
    of the devices. For the right hand it is used the channel 1 and for
    the left one, the 0. In the USB port it could be 32.
-   **Fake Hand**. It is a Boolean parameter that sets whether, it is
    simulated the AH device. In the case that the flag is true, it is
    not necessary to pass to the *allegro\_hand* class constructor,
    neither the path can and the channel arguments.

## Communication Loop 

The *loop\_function* of the *allegro\_hand\_ros* class is in charge of
both updating and publishing the current state of the AH device on the
ROS environment through the *joint\_states* topic. It obtains the
current from the *allegro\_hand* class, and publishes it to the
*joint\_state* topic. This is necessary for both simulation and real
hardware execution, since there is not any *joint\_state* controller
that actualizes the current joint state of the Allegro Hand, which it is
necessary for the *joint\_state\_publisher* and for the
*roboto\_state\_publisher*.

The *allegro\_hand\_ros/joint\_states* message contains all the joint
names of the AH device, which can be seen on next table, and the current joint state and
torque, alphanumerical sorted. 

## ROS services

The *allegro\_hand\_ros* class extends the *allegro\_hand* class. When initialized, the parameters are read and the Allegro Hand communication is initialized accordingly, then the services are set up and the initial controller .
If the **slave flag** is set to True, the *allegro\_hand\_ros* will
expose the following ROS service-based functions, which wraps the
respective class functionalities:

-   **GetController**. It is an empty-requesting ROS service where the
    server prints on the terminal the current active controller of the
    *allegro\_hand\_ros* class, and returns a Boolean status variable.
-   **SetController**. It is a ROS service where the client requests the
    desired *allegro\_hand* controller. Once
    received the request, the ROS service server calls the
    *SetController* *allegro\_hand* class function. That one, stops the
    active control thread, and starts the requested one. The ROS service
    server returns a Boolean success variable.
-   **SetDesiredPos**. It is a ROS service where the client requests the
    desired positions of the $16$ AH servos. Once received the request,
    the ROS service server calls the *SetDesiredPosition* function of
    the *allegro\_hand* instance, which updates its *pos\_des*membership
    variable, that is used on the PD and PID control loop to compute the
    torque signal. The ROS service server returns a Boolean success
    variable.
-   **SetDesiredTorque**. It is a ROS service where the client requests
    the desired toque of the $16$ AH servos. Once received the request,
    the ROS service server, using the before mentioned *SetController*
    function, sets the torque controller and then, calls the
    *SetCommandedTorque* function of the *allegro\_hand* instance, which
    updates its *torque\_des* membership variable, that is used on the
    torque-based control loop to directly apply the torque signal. The
    ROS service server returns a Boolean success variable.
-   **StopHand**. It is an empty-requesting ROS service where the
    service server calls the *StopHand* function of the *allegro\_hand*
    instance, which stops the communication with the hardware and the
    active control thread, and also, turns the *stop* Boolean flag, that
    ceases the *allegro\_hand\_ros* *loop\_function* execution. The ROS
    service server returns a Boolean success variable.

Furthermore, there is another main *allegro\_hand\_ros* functionality,
not wrapped by a ROS service, named *SetGravityVector* that calls the
same named function of the *allegro\_hand* class, which updates the
*gravity* membership variable, which is used to compute the gravity
compensation torque, mentioned before.

## Description

The robot description of the AH robot was originally defined as an URDF
file, where it is rawly described the kinematic chain of the robot with
the first fingertip models, which basically is a rubber sphere, without
any kind of tactile sensor, like this:

![Left Allegro Hand Basic Fingertip Model
rviz.](images/old_model_allegro_hand.png)

For this reason, it is opted to implement a Xacro configuration file,
which offer more versatility and modularity than the URDF one, enabling
the possibility to define arguments and properties tags, as well as
include other Xacro files. Following this line, three different Xacro
files are implemented, which are described below.

-   **rightAllegroHand.urdf.xacro**. It defines the right hand kinematic
    chain with some Xacro tags to specify the link prefix, the joint
    prefix and the fingertip type. The first two arguments are useful
    when two different AH are considered. Under this situation, it is
    necessary to add prefixes on the link and joint names, to avoid
    appointment conflicts. The last one is used to set the fingertip
    type. 
-   **leftAllegroHand.urdf.xacro**. It defines the left hand kinematic
    chain with some Xacro tags to specify the link prefix, the joint
    prefix and the fingertip type, as the right one. The only difference
    with the right hand is the finger position and rotation axis.
-   **allegroHandRobot.urdf.xacro**. It is a Xacro macro that
    encapsulates both right and left AH Xacro files, as well as the
    arguments and properties before mentioned.

![Right Allegro Hand Weiss Fingertip Model
rviz.](images/right_allegro_hand_weiss_tip_rviz.png)

The kinematic chain created by the *allegroHandRobot.urdf.xacro* can be
shown in the Figure [5.12](#figure:allegro_hand_rqt_tf_tree). The Xacro
file allows to joint different kinematic chains, setting a virtual joint
between the final and the initial link of each robot, respectively.
Therefore, it is possible to fix the AH kinematic chain to the MADAR
one, as it is explained in subsection
[4.1](#section:solution_madar_description).

![Allegro Hand rqt tf
Tree.](images/allegro_hand_rqt_tree.png){#figure:allegro_hand_rqt_tf_tree
width="160mm"}

## Example of Use

Once detailed the *allegro\_hand* and the *allegro\_hand\_ros*
structures and all their functionalities, it is needed to specify the
packages on which they depend on for their implementations, as the
*launch* files developed for both the simulation and the real hardware
bring up.

The main differences between the simulation and the real hardware bring
up *launch* files, are that the simulation does not need the *CAN* path
neither the channel specification, whereas the real hardware yes.

```
$ roslaunch allegro_hand_ros hand.launch yaml_file:=SAH020BR014.yaml channel:=1 fake_hand:=true hand:=right link_prefix:=right_hand_
$ roslaunch allegro_hand_ros hand.launch yaml_file:=SAH030C035L.yaml channel:=0 fake_hand:=false hand:=left link_prefix:=left_hand_
```

In another terminal:

```
$ rosservice call /right_hand/setController "tipo: 'joint_position_PID'"
```

For example, send a desired configuration

```
$ rosservice call /right_hand/setDesiredPosition "pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```
If you want to see two hands:
```
$ roslaunch allegro_hand_ros two_hands.launch
```
![Rviz with two hands.](images/two_hands_rviz.png){#figure:two_hands_rviz width="160mm"}


## hand.launch and others

```hand.launch ``` can be used as example to launch the allegro_hand_ros node. It has these arguments to take care:

* yaml_file :  it has a default empty value. It must contain the yaml file passed to the allegro_hand driver to initialize the hand. For example 'SAH020BR014.yaml'. It will search in the 'allegro_hand/conf' directory.

* path_pcan : it has a default="/dev/pcan". Allegro Hand driver uses libpcan as interfacte to communicate with the hand.

* channel : it has a default empty value. With the path_pcan defines pcan0, pcan1 or pcan32 port to communicate with the hand.

* fake_hand: it has a default empty value. It could be "true" or "false". It defines in the driver if it has to connect to a "real" hand or not. It is designed to simulation purposes.

* hand: it has a default empty value. It could be "left" or "right".

* link_prefix : it has default empty value. It is used a prefix used when you have two hands working together.
  
<!-- * joint_prefix: it has default empty value. It is used as prefix for the joints in case you need it when you have two hands working together. -->

* tip: it has a default value of "default" that refers to the default tip provided by the manufacturer. It can have as values "weiss" that is the tip with the weiss sensor and "optoforce" that is the tip with the Optoforce sensor mounted at 45 degrees.