# Allegro hand description

Repository for the Allegro hand description. It displays the left or right hand in rviz2.

## Requirements

To build the library you need:

- [Ros Humble](https://docs.ros.org/en/humble/Installation.html)
- [joint_state_publisher_gui](http://wiki.ros.org/joint_state_publisher_gui)
```
sudo apt-get install joint-state-publisher-gui
```
- [Xacro](http://wiki.ros.org/xacro)
```
sudo apt install ros-[ros_version, for instance: humble]-xacro
```


## Display the Allegro Hand in Rviz2

- **Step 1**. Build the package and source the environment. In the workspace folder:
```
$ colcon build --packages-select allegro_hand_description
$ source install/setup.bash
```
- **Step 2**. Run the Launch file
```
$ ros2 launch allegro_hand_description display_allegro_joint_state_publisher.launch.py
```
## Parameters

There are some parameters that we can change such as:

||       **Parameters**    |                              **Description**                              |
|-|------------------------|---------------------------------------------------------------------------|
|1|   *joint_prefix*       | Prefix of the joint names, useful for multi-robot setup.                  |
|2|    *link_prefix*       | Prefix of the link names, useful for multi-robot setup.                   |
|3|       *tip*            | Type of tip sensor: Weiss or optoforce names.                             |
|4|      *hand*            | Choose which hand to display either 'right' or 'left' hand.               |
|5|  *description_file*    | Name of the xacro file.                                                   |
|6|*rviz_config_file_name* | File name of the rviz configuration file                                  |

## Additional launch file
There exist another launch file, display_allegro.launch.py, in case we do not want to use the joint state publisher. 
It is very similar to the display_allegro_joint_state_publisher.launch.py, it has some additional parameters:

||       **Parameters**    |                              **Description**                              |
|-|------------------------|---------------------------------------------------------------------------|
|1|   *joint_prefix*       | Prefix of the joint names, useful for multi-robot setup.                  |
|2|    *link_prefix*       | Prefix of the link names, useful for multi-robot setup.                   |
|3|       *tip*            | Type of tip sensor: Weiss or optoforce names.                             |
|4|      *hand*            | Choose which hand to display either 'right' or 'left' hand.               |
|5|  *description_file*    | Name of the xacro file.                                                   |
|6|*namespace_jointstates* | Namespace of the joint_states. We will remap them                         |
|7| *remapped_jointstates* | Remap of the joint_states                                                 |
|8|*rviz_config_file_path* | Path of the rviz configuration file                                       |

