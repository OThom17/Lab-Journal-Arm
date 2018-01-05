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
```

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

```

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


# Document detailing the process of importing a STL file into RVIZ's gui

Stock URDF File

```
<robot name="roco_arm">

<link name="base_link">
<visual><geometry>
<cylinder length="0.06" radius="0.1"/>
</geometry>
</visual>
</link>


<link name="second_segment">
<visual>
<geometry>
<box size="0.6 0.05 0.1"/>
</geometry>
<origin rpy=" 0 0" xyz="-0.3 0 0"/>
</visual>
</link>


<link name="first_segment">
<visual>
geometry>
<box size="0.6 0.05 0.1"/>
</geometry>
<origin rpy="0 0 0" xyz="-0.3 0 0"/>
</visual>
</link>


<joint name="base_to_first" type="revolute">
<axis xyz="0 1 0"/>
<limit effort="1000" lower="0" upper="3.14" velocity="0.5"/>
<parent link="base_link"/>
<child link="first_segment"/>
<origin xyz="0 0 0.03"/>
</joint>
</robot>
```

## URDF Breakdown

The URDF file is written in XML and is comprised of two components; the links (Members) and the joints. 

## The following paramters can be defined for a link:

### Visual Breakdown

- Inertial: A 3x3 rotational interial matrix descriing how the component will react to motion.
- Origin:   The local origin in relation to the member.
- Geometry: Outlines the dimensions of the component. This can be described explicitly written in terms of either rectangular, cylindrical of spherical shapes. Alternatively mesh files can be imported from packages such as fusion 360 or solidworks.
- Material: Purely asthetic property used to degine texture and colour of components.

### Joint Breakdown

Listing of the possible variations of settable parameters

- Type:     Revolute, Continous, prismatic, fixed, floating and planar
- Origin:   The global location of the joint at the 'centre' positions of all members.
- Parent:   The base element of the joint
- Child:    The secondary element.
- Axis:     Sets the axis in which the joint will rotate around.
- Limits:   The maximum and miniimum angle of rotation.
- Velocity: Maximum joint velocity with units m/s for prismatic and rad's per second for revolute. 




## Importing complex STL Mesh files

To include more complex forms which can't be made through the use of the primary shapes (Rectangles, spheres and squares) meshes can be exported from 3D CAD packages like 360 fusion and solidworks and imported into the model. As the design featured multiple components to each element such as either side of the primary arm member I've used solidworks to merge them into four main sections, these are as follows:

- Base Unit
- Base Mounting point
- Primary Arm Section
- Secondary Arm section

With ROS not supporting a 'snap' positional function this will simplify the contruction when imported into the model and manually determing the origin points. 

Using the ROS documentation the STL files is included in the model through the use of the following code:

```
  <geometry>
    <mesh filename="package://auriga_model/auriga_base.stl"/>
  </geometry>
  
```

However the ROS filesystem hasn't been made into a package so the exact filepath has to be described as follows.

```

<?xml version= "1.0"?>

<robot name="ROBO-ARM">
  <link
    name="Base_Assembly">

    <visual>
      <origin
        xyz="-0.021 -0.020 0.05"
        rpy="1.58 0 0" />
      <geometry>
        <mesh
          filename="file:///home/othom/ROSSTL/Master/Base_Assembly.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.79216 0.81961 0.93333 1" />
      </material>
    </visual>
  </link>


  <link
    name="Base_Mount_Ass">

    <visual>
      <origin
        xyz="-0.016 0.0001 0"
        rpy="1.56 0 0.7" />
      <geometry>
        <mesh
          filename="file:///home/othom/ROSSTL/Master/Base_Mount_Ass.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.79216 0.81961 0.93333 1" />
      </material>
    </visual>
  </link>

  <link
    name="Primary_Assem">
    <visual>
      <origin
        xyz="0 -0.007 -0.009"
        rpy="0 0 -0.015" />
      <geometry>
        <mesh
          filename="file:///home/othom/ROSSTL/Master/Primary_Assem.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.79216 0.81961 0.93333 1" />
      </material>
    </visual>
  </link>

  <link
    name="Secondary_Ass">
    <visual>
      <origin
        xyz="0.001 0.064 -0.0046"
        rpy="1.6 0 0" />
      <geometry>
        <mesh
          filename="file:///home/othom/ROSSTL/Master/Secondary_Ass.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.79216 0.81961 0.93333 1" />
      </material>
    </visual>
  </link>

</robot>

```
In the URDF file above each component (link) is referencing a STL mesh. The positional data (origin) has been determined from the original solidworks model as well as using the axis provided by ROS to refine the positions.

## Defining the joint positions and types
- Origin
The joint co-ordinates are to be set as if the model is in the central position. These values were roughly determined using the solidworks model but largely are a result of trail and error through the use of the axis provided by ROS. 

- Type
Both of the arm joints have been set to the type 'revolute' as they are both rotational and will have defined contraints. The joint connecting the base to the mount however will not be constained and therefore will be of type 'contiunous'.

- Axis
Both arm joints' 'axis' has been set to 'x' as this is the vertical plane. The mount has therefore been set to 'z' as this will enable horizontal rotation.

- Limits
To contrain the rotation of the joints limits have been included. Since the two servo motors are physically contrained from 0 - 180 degrees they've had their lower and upper limits set to 0 and 1.57 respectively. Since the stepper motor connecting the base to the mount piece is continuous no limitation is needed. 

```

<?xml version= "1.0"?>

  <joint name="Base_to_Mount" type="revolute">
    <axis xyz="0 0 1" />
    <limit effort="1000" lower="0" upper="3.14" velocity="0.5" />
    <parent link="Base_Assembly"/>
    <child link="Base_Mount_Ass"/>
    <origin xyz="0 0 0.125" />
  </joint>


  <joint name="Mount_to_Primary" type="revolute">
    <axis xyz="1 0 0" />
    <limit effort="1000" lower="0" upper="3.14" velocity="0.5" />
    <parent link="Base_Mount_Ass"/>
    <child link="Primary_Assem"/>
    <origin xyz="0.025 -0.001 0.005" />
  </joint>

  <joint name="Primary_to_Secondary" type="revolute">
    <axis xyz="1 0 0" />
    <limit effort="1000" lower="-1.57" upper="1.57" velocity="0.5" />
    <parent link="Primary_Assem"/>
    <child link="Secondary_Ass"/>
    <origin xyz="-0.025 0.136 -0.00032" />
  </joint>

```

*** URDF has both servo joints with a full 360 degree rotation.


# Development of control code

## Arduino and URDF Code respectively

```
#include <Servo.h> 

#include <ros.h>

#include <sensor_msgs/JointState.h>



using namespace ros;



NodeHandle  nh;



Servo MountPrimary;

Servo PrimarySecondary;



void cb(const sensor_msgs::JointState&  states){

  int angle1 = (int)(ssttion[0] * 180/3.14);

  int angle2 = (int)(msg.position[1] * 180/3.14);



  MountPrimary.write(angle1);

  PrimarySecondary.write(angle2);

  

  Serial.print("ROS Output Mount: ");

  Serial.println(angle1);

  Serial.print("ROS Output Primary: ");

  Serial.println(angle2);



}





Subscriber<sensor_msgs::JointState>

                        sub("joint_states", cb);



void setup(){



  nh.initNode();

  nh.subscribe(sub);

  Serial.begin(9600);

  Serial.print("Start");

  MountPrimary.attach(9); //attach it to pin 9

  PrimarySecondary.attach(9); //attach it to pin 9

 }



void loop(){

  nh.spinOnce();

  delay(10);

}
```




```
<?xml version= "1.0"?>

<robot name="ROBO-ARM">
  <link
    name="Base_Assembly">

    <visual>
      <origin
        xyz="-0.021 -0.020 0.05"
        rpy="1.58 0 0" />
      <geometry>
        <mesh
          filename="file:///home/othom/ROSSTL/Master/Base_Assembly.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.79216 0.81961 0.93333 1" />
      </material>
    </visual>
  </link>


  <link
    name="Base_Mount_Ass">

    <visual>
      <origin
        xyz="-0.016 0.0001 0"
        rpy="1.56 0 0.7" />
      <geometry>
        <mesh
          filename="file:///home/othom/ROSSTL/Master/Base_Mount_Ass.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.79216 0.81961 0.93333 1" />
      </material>
    </visual>
  </link>

  <link
    name="Primary_Assem">
    <visual>
      <origin
        xyz="0 -0.007 -0.009"
        rpy="0 0 -0.015" />
      <geometry>
        <mesh
          filename="file:///home/othom/ROSSTL/Master/Primary_Assem.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.79216 0.81961 0.93333 1" />
      </material>
    </visual>
  </link>

  <link
    name="Secondary_Ass">
    <visual>
      <origin
        xyz="0.001 0.064 -0.0046"
        rpy="1.6 0 0" />
      <geometry>
        <mesh
          filename="file:///home/othom/ROSSTL/Master/Secondary_Ass.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.79216 0.81961 0.93333 1" />
      </material>
    </visual>
  </link>


  <joint name="Base_to_Mount" type="revolute">
    <axis xyz="0 0 1" />
    <limit effort="1000" lower="0" upper="3.14" velocity="0.5" />
    <parent link="Base_Assembly"/>
    <child link="Base_Mount_Ass"/>
    <origin xyz="0 0 0.125" />
  </joint>


  <joint name="Mount_to_Primary" type="revolute">
    <axis xyz="1 0 0" />
    <limit effort="1000" lower="0" upper="3.14" velocity="0.5" />
    <parent link="Base_Mount_Ass"/>
    <child link="Primary_Assem"/>
    <origin xyz="0.025 -0.001 0.005" />
  </joint>

  <joint name="Primary_to_Secondary" type="revolute">
    <axis xyz="1 0 0" />
    <limit effort="1000" lower="-1.57" upper="1.57" velocity="0.5" />
    <parent link="Primary_Assem"/>
    <child link="Secondary_Ass"/>
    <origin xyz="-0.025 0.136 -0.00032" />
  </joint>

</robot>

```

## Developments

- Due to torque inaccuracies some of the degree's of rotation present an issue, therefore the specific joint and degrees of rotation will be limited more than what's necessary for simple element collision avoidance.

- Instead of the base being a direct mapping to the degree's of rotation the slider will be centred and either side determine the direction of rotation. The further the slider is from the centre the greater the angle per second rotation.

**** The finer torque limitation will have to be done exprimentally




















