# Miscellaneous

---
> - 🔙 Go back to the main README file: [0 - Main](../README.md)
> - ⬅️ Previous section: [11 - The Official Software](11_TheOfficialSoftware.md)
> - ➡️ Next section: [13 - Additional Information](13_AdditionalInformation.md) 
---

## 1. Adding new controllers

The *AllegroHandController* class contains algorithms to control the
Allegro Hand. The software has five
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

The parameters for both Joint Position PD and Joint Position PID for each hand can be found at the folder conf/controllers.
Each file contains the control parameters for a specific hand version. The naming convention of the files follows `hand_name + ’_’ + controller_name.yaml`.

### 1.1 Adding an existing controller controller
To do so, copy one existing file and change the name accordingly.

### 1.2 Varying Control parameters of the PD and PID controller 
Change the values of the file `conf/controllers/hand_name + ’_’ + controller_name.yaml` and save the file.

As a reminder, every time the controller is switched, the previous control instance is deleted. 
Thus, to evaluate the change of parameters, there is no need to restart the program, but only switch twice between controllers (switch to a differnt controller and then come back).

### 1.3 Add new customized controller
The software has a modular design to easily add new controllers.
The new controller class will extend the class *AllegroHandController*.
In this section a detailed procedure will be explained:

- **Step 1**: Create a new class that extends the *AllegroHandController* class. 
    Create the .h and .cpp files, it is recommended to use some other controller as an example, for instance, the Torque controller since it is pretty simple and straightforward.

    The minimum requirements that this class must fulfill are:
    - The Constructor that initiates the *AllegroHandController* class.
    - A method called *control()* that overrides the *control()* method of the *AllegroHandController* class:
        
        ```c++
        //.h file
        void control() override;
        ```
        Inside this method the developer must add the desired control algorithm.
        The calculated torque has to be stored in the *torque* variable using mutex:

        ```c++
        //.cpp file
        void control() {
            // Control algorithm code
            mutex.lock();
            torque = 'Desired torque values';
            mutex.unlock();
        }
        ```
        Note: that torque is a variable
        ` typedef Eigen::Matrix<double, n_fingers*finger_n_dofs, 1>`

- **Step 2**: Add the controller.
    The class *AllegroHand* has a variable that stores the names of the available controllers.
    ```std::vector<std::string> controllers_names```

    There are two options to add the controller into the *controllers_names* variable:
    - **Option 1**. Create a yaml file in the *conf/controllers* folder with the naming convention `hand_name + ’_’ + controller_name.yaml`.
        Automatically the *AllegroHand* function will read all the available controllers for each hand at the initialization step, when executing the method  *searchControllers()*.
    
    - **Option 2**. Add the name at the initialization of the *AllegroHand* class, specifically at the *SetUpAH()* function.
        ```c++
         controllers_names.push_back("controller_name");
        ```

    Once you have done either option 1 or 2, you should modify the method *createController()* of the *AllegroHand* class located [here](/src/AllegroHandControl.cpp)to relate the class that you created with the *controller_name*, by just adding a new else, following the previous structure.
    

## 2. Limit values and ranges of the joints.
### 2.1 Problem
Empirically we have seen that the range of the joints of the different hands is not the same. 
For context, we have tested this code with the hands R283 and R284, that belong to the 4th version of the Allegro hand, as well as, with the hands SAH020BR014 and SAH020AL011, that belong to the 2nd version. 

We have seen that the ranges of the joints are not the same, and moreover the minimum and maximum values of the joints do not match. Fortunately, both version 4 have similar values, wereass both hand version 2 have different values between eachother and also compared to the version 4. 
More over, the physical joint values obtained with all the hands, do not match the limit values provided by the official URDF.

### 2.2 Solution
In this section we will detail how we solved this problem. As well as explain the procedure.
#### STEP 1.  Obtain the physical joint limits of the hand
In order to solve this problem, the first thing that we did is develop a simple program.This program allows to store all the minimum and maximum values of all the joints in a yaml file. This program can be found in the folder examples/calculate_min_max_joints.cpp. To run it you just need to (in the build folder after the make is made):
```
$ examples/calc_min_max <yaml_configuration_file>
```
Recall that: yaml_configuration_file: can either be the file name or the full path
Some examples are shown below:

```
$ examples/calc_min_max V4R283
$ examples/calc_min_max V4R283.yaml
$ examples/calc_min_max ../conf/configuration_files/V4R284.yaml
```

We advise everyone to test what are the actual joint limits of the hand (specially with version 1, 2 and 3). From our experience version 4 hands have consistent values within the version, wereass the previous version's values differ within the version.

The resulting yaml file with all the joint limits can be found in the folder /conf/joints_limits

#### STEP 2. Compare the obtained values and select new URDF values
We proceed to compare the joint limit values of the hands between them and with the official URDF limit values.
This comparison can be found in the LibreOffice Calc file located at /conf/joints_limits/joint_values_comparison_with_urdf.fods

Since the values obtained with version 4 hands are consistent within the version, we decided to select the new URDF joint limit values based on those values. To do so, and since the Index, Middle and Anular finger should have the same limits (because they are equal) we calculated the mean values of each joint. As an example the new values of j00, j10 and j20 is calculated as (j00_V4R283 + j10_V4R283 + j20_V4R283 + j00_V4R284 + j10_V4R284 + j20_V4R284)/6.
The values of the Thumb are the mean of the values obtained with both version 4 hands. For example for j30 = (j30_V4R283 + j30_V4R284)/2.

The new values of the URDF can also be found in the joint_values_comparison_with_urdf.fods file.

For whom may be interested in using this program their own Allegro hand. We advise to compare the values obtained in STEP 1 with the new urdf values. (Recommend to pass from yaml file to JSON first and then paste it automatically into the Calc file).

#### STEP 3. Adapt configuration file of versions previous so the joint limits match the URDF
One of the main differences between version 4 and the previous ones is the way to communications protocol.
Since we decided that the new URDF values are the ones obtained with the version 4, we can only focus on fixing the previous versions.

When we initiallize an Allegro hand the first thing we do is read the configuration file located in conf/configuration_files. These file provides the encoderOffset, encoderDirection and motorDirection values of each of the 16 joints, among other valuable information.

Following the official communications protocol, version 4 does not use any of the previous parameters. Wereass, previous versions use encoderOffset and encoderDirection when reading a message send from the hand to the computer. And the motorDirecion when sending a torque towards the hand.

The formula for converting the message data of each joint to radians is given by the manufactor and used in the function parseCANMessage() in the CAN_AH class. 
The forumla for each joint is:

\[  joint_value[rad] = (encoderDirection * data - 32768.0 - encoderOffset) * (333.3/65536.) * (M_PI/180.) \]

- Note: encoderDirection can either be -1 or 1.

We can calculate a new encoderOffset so that with the same message data we obtain a joint value that matches the new urdf value. To do so we can:
\[ encoderOffset_new_min = encoderOffset_old + (joint_value_min - joint_value_min_desired) / (333.3/65536.) / (M_PI/180.) \]
\[ encoderOffset_new_max = encoderOffset_old + (joint_value_max - joint_value_max_desired) / (333.3/65536.) / (M_PI/180.) \]
where: 
- joint_value_min and joint_value_max are the limits obtained with the encoderOffset_old value (values obtained in STEP 1)
- joint_value_min_desired and joint_value_max_desired are the desired limits that we would like to obtain, in this case the limits of the URDF.

Since the range [joint_value_min - joint_value_max] is different for the versions but it does not differ much. We will select as a new encoderOffset value the mean value, so despite the range difference it is centered. Thus,

\[ encoderOffset_new = (encoderOffset_new_min + encoderOffset_new_max)/2 \]

#### STEP 4. Check that everything works 

The last step is to check that everything is working right. We proceed todo the same as in STEP 1 and get the minimum and maximum values of all the joints and then compare them with the urdf limit values.

### 2.3 Additional possible problems and their solution
#### What if the ranges differ too much?
We have not implemented the solution that we will propose because empirically we have not encountered this problem. In our case the ranges difference is always lower than 0.1. 
In case the reader has encounter this problem we will provide an idea to solve this problem:

First and foremost, make sure that the hand wiring is well connected and working properly. If it is and the problem keeps occuring then:

Recall the previous equation:
\[  joint_value_min[rad] = (encoderDirection * data_min - 32768.0 - encoderOffset) * (333.3/65536.) * (M_PI/180.) \]
\[  joint_value_max[rad] = (encoderDirection * data_max - 32768.0 - encoderOffset) * (333.3/65536.) * (M_PI/180.) \]

Thus the range is:  
\[  joint_value_max[rad] - joint_value_min[rad] = encoderDirection * (data_max - data_min) * (333.3/65536.) * (M_PI/180.) \]
Since all the parameters are multiplying, the only way to modify the range is by adding or substracting another parameter: let's name it rangeDifference
\[  joint_value_max[rad] - joint_value_min[rad] = (encoderDirection * (data_max - data_min) * (333.3/65536.) * (M_PI/180.)) + rangeDifference \]

This is the idea, in order to try it out the reader should:
- Create a new parameter in the configuration file: 'rangeOffset'
- Store this parameter when reading the configuration file.
- In the parseCANMessage() function of the CAN_AH class modify the function so that the equation takes into account this parameter.
The main problem that the reader may encounter is that the version3Handler::parseCANMessage() function reads all the messages of all the fingers in the same way. It allows to treat j00, j01, j02 and j03 differently, but it does not allow to treat each finger separately, for instance it treats j00, j10, j20, j30 the same way. 
There exists a solution for this problem, and it is to first identify the finger that is sending the message and then apply the corresponding rangeOffset.
- Verify that the result is the desired one.

#### What if the values of a hand of the 4th version do not match the URDF values?
First and foremost, make sure that the hand wiring is well connected and working properly. If it is and the problem keeps occuring then:
- If you only have one hand the easiest way to solve this problem is to change the URDF acordingly.
- Otherwise:
This is the current function that works with version 4, extracted from the communication protocol provided by the manufacturer:

\[  joint_value[rad] = data * (333.3/65536.) * (M_PI/180.) \]

Change the  version4Handler::parseCANMessage(), where the previous function is used and add the encoderOffset to the function that reads the values. 
\[  joint_value[rad] = (data - encoderOffset) * (333.3/65536.) * (M_PI/180.) \]

Finally, follow the instructions presented in the solution.

---
> - 🔙 Go back to the main README file: [0 - Main](../README.md)
> - ⬅️ Previous section: [11 - The Official Software](11_TheOfficialSoftware.md)
> - ➡️ Next section: [13 - Additional Information](13_AdditionalInformation.md) 
---