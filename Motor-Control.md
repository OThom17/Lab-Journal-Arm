# Development of motor control code

## Original code with direct mapping between slider and servo position

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



## Planned Developments

- Due to torque inaccuracies some of the degree's of rotation present an issue, therefore the specific joint and degrees of rotation will be limited more than what's necessary for simple element collision avoidance.

- Instead of the base being a direct mapping to the degree's of rotation the slider will be centred and either side determine the direction of rotation. The further the slider is from the centre the greater the angle per second rotation.


**** The finer torque limitation will have to be done exprimentally













