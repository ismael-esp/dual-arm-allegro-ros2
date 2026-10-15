# 4 - The CAN Communications Class: CAN_AH

---
> - 🔙 Go back to the main README file: [0 - Main](../README.md#the-can-communications-class-can_ah)
> - ⬅️ Previous section: [3 - Directory Organization](3_DirectoryOrganization.md)
> - ➡️ Next section: [5 - The Controller Class: AllegroHandController](5_ControllerClass.md)
---


The *CAN_AH* class is a software module designed to manage communication
between the computer system and the robotic hand through a
CAN interface. It
enables communication with the Allegro Hand, since it allows to start
and stop communications, generates messages to send torques and parses
the received messages to retrieve the current hand configuration. This
class supports all Allegro Hand versions. Due to differences between
versions in the Allegro Hand communication protocol, there exist two
analogous sub classes, one for versions 1, 2 and 3 and another subclass
for the latest version, version 4. There are two libraries available for
using with CAN Bus
communications, the PCAN and the CAN library. The CAN library can be used with the Linux kernel
as a network driver, although the implementation is more complex. This
implementation has used the PCAN library, specifically the PCANBasic
library, because this powerful library eases the process of handling
CAN Bus
communications.

## Constructor

The constructor of the communication class initializes the
CAN communications
with the hand and it sets a 3 ms periodic communication. The AllegroHand
sends one message per finger, and it is always sent in the same order.
Thus, for ensuring that the finger messages belong to the same time, the
constructor also performs an initial read in order to identify the ID of
the finger of the first message.

The constructor of the *CAN_AH* class initializes the
CAN communication
channel and configures it based on the version of the robotic hand
specified. It has two input parameters:

-   *channel*: A string containing the appropriate
    CAN channel for
    communication. The channel depends on the port of the computer
    system where the Allegro Hand is connected to.

-   *params*: A Parameters struct that contains information of the
    Allegro Hand. The Parameters struct can contain the serial number of
    the hand, the hand version and the motor direction, the encoder
    offset and the encoder direction.

The initialization process starts by handling the version, depending on
the hand version, a specific handler (*CAN_AH_V3* or *CAN_AH_V4*) is
created. These handlers encapsulate version-specific operations. Up
next, the class retrieves and sets the appropriate
CAN channel for
communication. To do so, it uses the *channelToBus()* private method to
map a logical channel number to a physical CAN bus handle. The next step consists on
initializing the CAN channel with the *initCAN()* private
method, by setting the appropriate baud rate (1Mbps) and resetting the
CAN controller.
This is essential for establishing a reliable communication link.

The constructor then, starts the CAN communication with the Allegro Hand using
the specific handler for the hand's version. The
CAN communication
is started in the *startCAN()* private method. This function starts
requesting the hand information and displaying it on to the console. Up
to version 3, the Allegro Hand information provided by the hand through
the CAN bus
communications is the hardware and firmware version. Whereas, version 4
does not only provide the hardware and firmware version, it also
specifies the hand serial number, the hardware type, which is the type
of hand (0 for right and 1 for left), the temperature of the hand, the
hand and servo status and whether the hand has a fault due to high
temperature or internal communications. It proceeds by setting the
communication period to 3 ms, this is executed through the private
function *command_set_period()*. This is the minimum time period allowed
by the Allegro Hand according to the communications protocol provided by
the manufacturer.

As it has been previously stated, the Allegro Hand sends one message per
finger. Each message has an identifier that indicates which finger the
message belongs to. The Allegro Hand messages are orderly fashioned, the
order is Index, Middle, Annular and Thumb. Since the communications are
set to periodic, the Allegro Hand continuously sends messages containing
the current joint position values of the fingers. The whole hand
information is sent using 4 messages, each message batch is sent every 3
ms. To guarantee the correct functionality of the hand, and that each
time that a batch of messages is read, these messages correspond to the
same timestamp, a batch of messages is read and the ID of the Index
finger is retrieved and stored using *getIDFirstFinger()*. To ensure
that no old or irrelevant messages interfere with the hand operation,
the CAN buffer is
cleared at startup.


## Destructor

The Destructor of the *CAN_AH* class safely shuts down the
CAN communication
by stopping the hand and closing the CAN channel using the provate methods
*command_can_stop()* and *command_can_close()*, respectively.

## Key Methods

Below, all the class public methods are discussed. The overall structure
of these methods is identical for *CAN_AH_V3* and *CAN_AH_V4*. These
methods can be divided into the ones related to reading the current
joint positions and the ones related to sending the desired torque
commands towards the Allegro Hand.

-   **Reading methods.** The methods related to retrieving the current
    hand configuration are listed below.

    1.  **readDataTimeStamp():** Reads CAN messages, that contain the whole
        hand information, this is the current 16 joint position values,
        and updates the timestamp information. The *CAN_Read()* function
        is used to read the received messages. This method ensures that
        each one of the four messages belong to each one of the fingers.
        It also verifies that the four messages belong to the same
        periodic reading by checking the ID of the first message.
        Additionally, it checks the validity of the messages, in case
        any message fails an error is logged into the console.
        Otherwise, the read messages and the corresponding timestamp are
        stored in a TPCANMsg and a TPCANTimestamp private variables,
        called *lmsgs* and *timeStamp*, respectively.

    2.  **readDevices():** Parses the read messages to extract the
        information and store the joint position values of the hand.
        This method parses the messages stored in the *lmsgs* TPCANMsg
        variable using the *parseCANMessage()* function. The
        *parseCANMessage()* function, starts by extracting the data and
        the ID of each message. Then it ensures that the message ID
        corresponds to a message containing the joint position values of
        a finger. If so the data is processed, which involves separating
        the raw data of each one of the four joints of the finger. And
        then converting the raw data of each joint from encoder units to
        joint position values in radians. 
        
        The conversion equation used
        depends on the Allegro Hand version as it has been shown in the
        Communications Protocol section, up until version 3 the
        conversion is given by equation:
        $$ joint[rad] = (encoderDirection \cdot data - 32768.0 - encoderOffset) \cdot (333.3/65536.0) \cdot (\pi/180.0) $$
        The conversion for Allegro Hand version 4 is given by equation:
        $$ joint[rad] = data \cdot (333.3/65536.0) \cdot (\pi/180.0) $$ 
        
        The joint position values obtained with the *parseCANMessage()* method are stored
        in a private variable called *position*. In case any message
        fails an error is logged into the console. For optimal results,
        *readDataTimeStamp()* method should have been called first.

    3.  **getPosition():** Retrieves the current joint position values
        of the hand, copying it from the private variable *position*.
        For optimal results, *readDevices()* method should have been
        called first.

    4.  **getTimeStamp():** This method retrieves the timestamp of the
        read messages copying it from the private variable *timeStamp*.
        Specifically the timestamp obtained corresponds to the timestamp
        of the first of the four messages read. For optimal results,
        *readDataTimeStamp()* method should have been called first.

    The workflow for obtaining the current hand configuration, the
    method *readDataTimeStamp()* has to be called first, in order to
    read and store the messages containing the hand information. Then,
    the method *readDevices()* has to be called to parse those messages
    and update the private variable *position*. Finally, the
    *getPosition()* method will retrieve the hand joint position values.

-   **Writing methods.** The methods related to retrieving the current
    hand configuration are listed below.

    1.  **setTorque():** Sets the torque values of all the joints of the
        hand by copying them into the private variable *torque*. These
        torque desired values are the ones that are going to be sent
        towards the hand.

    2.  **writeDevices():** Moves the Allegro hand by sending torque
        commands towards the robotic hand. It creates and sends four
        messages, one per finger, to the Allegro Hand with the desired
        torque values. To send a batch of messages with the desired
        torques, first the values of each joint need to be converted
        using the tau conversion constant. 
        $$I = motorDirection \cdot k \cdot \tau$$
        Then,
        the messages are sent using the method *command_set_torque()*,
        this method generates the message containing the desired joint
        data and sends it towards the hand using the *canSendMsg()*
        function. For optimal results, *setTorque()* method should have
        been called first.

    3.  **getTorqueConversion():** Retrieves the tau conversion constant
        of the Allegro Hand. This value depends on the Allegro Hand
        version being used.

    To move the hand, the desired torque values of each joint of the
    Allegro Hand need to be sent. The workflow to do so, starts by
    calling the method *setTorque()*, in order to update the desired
    torque values into the private variable *torque*. Then, the
    *writeDevices()* method has to be called in order to generate the
    messages and send them to the Allegro Hand.

The algorithm below presents the optimal workflow for obtaining the
current joint positions of the Allegro Hand and sending the desired torque to the
Allegro Hand. This is the
optimal workflow because the classes depend on each other for updating
the data that we want to read/write. As a result if the methods are not
called following this workflow, the data will not correspond to the
current data.


**Optimal Workflow for Reading Hand Configuration:** 
- **Step 1:** Call
`readDataTimeStamp()` to read and store CAN messages 
- **Step 2:** Call `readDevices()`
to parse the messages and update joint positions 
- **Step 3:** Call
`getPosition()` to retrieve current joint positions 
- **Step 4:**
Optionally, call `getTimeStamp()` to retrieve timestamp information

**Optimal Workflow for Sending Torque Commands:** 
- **Step 1:** Call
`setTorque()` to set the desired torque values for each joint 
- **Step 2:** Call `writeDevices()` to generate messages and send the torque
commands to the hand

---
> - 🔙 Go back to the main README file: [0 - Main](../README.md#the-can-communications-class-can_ah)
> - ⬅️ Previous section: [3 - Directory Organization](3_DirectoryOrganization.md)
> - ➡️ Next section: [5 - The Controller Class: AllegroHandController](5_ControllerClass.md)
---