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

##The following paramters can be defined for a link:

##Visual Breakdown

Inertial: A 3x3 rotational interial matrix descriing how the component will react to motion.
Origin - The local origin in relation to the member.
Geometry - Outlines the dimensions of the component. This can be described explicitly written in terms of either rectangular, cylindrical of spherical shapes. Alternatively mesh files can be imported from packages such as fusion 360 or solidworks.
Material - Purely asthetic property used to degine texture and colour of components.

##Joint Breakdown
Type - Revolute, Continous, prismatic, fixed, floating and planar
Origin - The global location of the joint at the 'centre' positions of all members.
Parent - The base element of the joint
Child - The secondary element.
Axis - Sets the axis in which the joint will rotate around.
Limits -  The maximum and miniimum angle of rotation.
Velocity - maximum joint velocity with units m/s for prismatic and rad's per second for revolute. 




## Importing STL Mesh files

<robot name="roco_arm">

<link name="base_link">
<visual>
  <origin rpy="0 0 0" xyz="0.15 0 0.35"/>
  <geometry>
     <mesh filename="/home/othom/Downloads/OliverThompsonSTLFiles-ROCO222v1/OliverThompsonSTLFiles-ROCO222v1"/>
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

























