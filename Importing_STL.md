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
<geometry>
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

## Visual Breakdown

Inertial: A 3x3 rotational interial matrix descriing how the component will react to motion.
Origin - The local origin in relation to the member.
Geometry - Outlines the dimensions of the component. This can be described explicitly written in terms of either rectangular, cylindrical of spherical shapes. Alternatively mesh files can be imported from packages such as fusion 360 or solidworks.
Material - Purely asthetic property used to degine texture and colour of components.

## Joint Breakdown

Listing of the possible variations of settable parameters

Type - Revolute, Continous, prismatic, fixed, floating and planar
Origin - The global location of the joint at the 'centre' positions of all members.
Parent - The base element of the joint
Child - The secondary element.
Axis - Sets the axis in which the joint will rotate around.
Limits -  The maximum and miniimum angle of rotation.
Velocity - maximum joint velocity with units m/s for prismatic and rad's per second for revolute. 




## Importing STL Mesh files

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
























