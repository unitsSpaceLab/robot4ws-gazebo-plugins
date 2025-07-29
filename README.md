# robot4ws_gazebo_plugins
The ***robot4ws_gazebo_plugins*** package implements a series of custom plugins for the ***Gazebo Sim Harmonic*** simulator.

Before launching gazebo, in order to make it find your plugins, run
```
export GZ_SIM_SYSTEM_PLUGIN_PATH=path/to/your/workspace/install/robot4ws_gazebo_plugins/lib
```
from the terminal, or add it to your ~/.bashrc file. 


## Installation
change <ros2_ws> with name of the ROS2 workspace
```
cd ~/<ros2_ws>/src
git clone -b ros2_humble https://github.com/unitsSpaceLab/robot4ws-gazebo-plugins.git
cd ..
colcon build    # or: colcon build --packages-select robot4ws_gazebo_plugins
```


## Plugins description
### JointsControllerPlugin
The ***JointsControllerPlugin*** plugin implements a controller that gets the **low level commands** for the robot **joints** from a ros topic and applies them in gazebo.

The joints can be controlled either in **position** or **velocity**. In case of velocity control the commands are applied directly, while position controls are applied via a PID.

The commands are taken as ***actuator_msgs/msg/Actuators*** messages from a ros topic. The joints are associated to the corresponding commands throught their index in the message.

**Usage**: add to your robot URDF:
```
<gazebo>
    <plugin name="my_namespace::JointsControllerPlugin"
            filename="libJointsControllerPlugin.so">
        <topic>your_topic_name</topic>
        <config_path>/path/to/config_file.yaml</config_path>
    </plugin>
<gazebo>
```

**URDF fields**
* ***topic***: name of ros topic that publishes the joints commands
* ***config_path***: path to the configuration file

**Configuration file**: *.yaml* file with the following structure
```
joints:
  - name: "joint_1_name"
    msg_field: "field_name_1"
    msg_index: 0
    control_type: "velocity"
    coeff: -1.0

  - name: "joint_2_name"
    msg_field: "field_name_2"
    msg_index: 1
    control_type: "velocity"
    coeff: -1.0

  - name: "joint_3_name"
    msg_field: "field_name_3"
    msg_index: 0
    control_type: "position"
    coeff: 1.0
    pid:
      p: 50.0
      i: 0
      d: 0

  ...
```
where:
* ***name***: name of the joint
* ***msg_field*** : unnecessary, not used with this plugin 
* ***msg_index*** : index of the joint in the *actuator_msgs/msg/Actuators/velocity* or *actuator_msgs/msg/Actuators/position* vector
* ***control_type*** : "velocity" or "position"
* ***coeff*** : coefficient that multiplies the received command when acquired (e.g. -1 if the joint axis is inverted)
* ***pid*** : only used if *control_type* is set to position



### JointsControllerPluginDynamixel
**NOT TESTED** (might be removed) - use ***JointsControllerPlugin***

The ***JointsControllerPluginDynamixel*** does the same things as the *JointsControllerPlugin* plugin, but subscribes to a topic with ***robot4ws_msgs/msg/DynamixelParameters1*** messages (see the *robot4ws_msgs* package) instead of *actuator_msgs/msg/Actuators*.

Since *robot4ws_msgs/msg/DynamixelParameters1* does not have a predefined associated gz.msg, this plugin starts its own a ros node for the subscription, instead of using the *ros_gz_bridge* like *JointsControllerPlugin*.

**Usage**, **URDF fields** and **configuration file** are the same as the *JointsControllerPlugin* plugin, with the following exceptions:
* plugin ***name*** and ***filename*** in URDF
* the ***msg_field*** field of the configuration *.yaml* configuration file is used, instead of the *msg_index* one, to map the joints with the corresponding *robot4ws_msgs/msg/DynamixelParameters1* message fields.

## TO DO
The following plugins must be update to ROS2:
* terramechanic plugin
* terrain ml plugin
