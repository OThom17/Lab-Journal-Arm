# Lab Sheet Report

## What is the /rosout topic?

rosout subscribes to the standard /rosout topic, records these messages in a textual log file, and rebroadcasts the messages on /rosout_agg

/rosout_agg is an aggregated feed for subscribing to console logging messages. This aggregated topic is offered as a performance improvement: instead of connecting to individual ROS nodes to receive their console messages, the aggregated message feed can instead be received directly from the rosout node. 


Source: http://wiki.ros.org/rosout

# Generating a Node within the RViz platform

To communicate between the Arduino and the ROS platform a serial bridge is needed. For this the following command is ran.

```
sudo apt install ros-kinetic-rosserial-python ros-kinetic-rosserial-arduino
```

This installs the ROS library allowing it to be used within the Arduino IDE.


# Writing the code for the ROS node - Full Comments

```
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <Servo.h>

using namespace ros;

NodeHandle nh;                                  // Initialise the NodeHandle 'nh'
Servo servo;                                    // Initilaise the Servo Object

// Call back function for the subscriber 
void cb( const std_msgs::UInt16& msg){          // Detailed Explanation of this line in Journal
  servo.write(msg.data);                        // Message Contains values 0 - 180
}

Subscriber<std_msgs::UInt16> sub("servo", cb);  // Initialising the 'Sub' Object consisting of an unsigned 16 bit integer message. 'servo' is the topic name with 'cb' being the call back function

void setup(){
    nh.initNode();                              // Initialise the NodeHande Object nh
    nh.subscribe(sub);                          // Advertising any topics being published

    servo.attach(9);
}


void loop(){
  nh.spinOnce();  //Loops the call back function for the full 180 - Once
  delay(1);       //1ms Delay
}
```

The code must then be complied and uploaded to the Arduino. To initiate the ROS code the serial bridge must be started using the following code.

```
rosrun rosserial_python serial_node.py /dev/ttyACM0
``

# Developing the 3D model within RViz

To generate the 3D model a URDF file is used. (Unified Robot Description Format Document)

The following is the example code with comments

```

<robot name="roco_arm">
<link name="base_link">
<visual>
<geometry>
<cylinder length="0.06" radius="0.1"/>
</geometry>
</visual>
</link>
<link name="first_segment">
<visual>
<geometry>
<box size="0.6 0.05 0.1"/>
</geometry>
<origin rpy="0 0 0" xyz="-0.3 0 0"/>
</visual>
</link>
<joint name="base_to_first" type="revolute">
<axis xyz="0 1 0"/>
<limit effort="1000" lower="0" upper="3.14" velocity="0.5"/>
<parent link="base_link"/><child link="first_segment"/>
<origin xyz="0 0 0.03"/>
</joint>
</robot>

```

To move the servo motor use the following command

```

rostopic pub /servo std_msgs/UInt16 10

``

Where the topic is /servo, /type is std_msgs/UInt16 with argument 10.


To load the file with the description 'robot_description' the following command is used

```
rosparam set robot_description -t ~/robot-project/models/robot-arm.urdf
```

To launch the publisher

```
rosrun robot_state_publisher robot_state_publisher
```

To initiate the publishers

```
rosrun joint_state_publisher joint_state_publisher _use_gui:=true
```



# Develop a custom URDF




































