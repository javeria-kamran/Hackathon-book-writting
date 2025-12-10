# Week 6: Robot Kinematics and Control

## 6.1: Forward and Inverse Kinematics

Kinematics is the study of motion without considering the forces that cause it. In robotics, kinematics is crucial for understanding how a robot's joints and links move to achieve a desired pose (position and orientation) of its end-effector (e.g., a gripper or tool). We differentiate between two main types of kinematics problems: Forward Kinematics and Inverse Kinematics.

### Forward Kinematics (FK)

**Forward Kinematics** is the process of calculating the position and orientation (pose) of a robot's end-effector given the known joint angles or positions of all its links. It's a relatively straightforward calculation, often involving a series of matrix multiplications (homogeneous transformations) that describe the transformation from one link's coordinate frame to the next.

**Input:** Joint angles/positions.
**Output:** End-effector pose (position and orientation).

#### Why is FK important?

*   **Visualization:** When you command a robot's joints, FK allows you to visualize where the end-effector will be.
*   **Sensor Fusion:** If you have sensors on the end-effector, FK helps in understanding their position relative to the robot's base.
*   **Collision Detection:** Knowing the exact pose of all links is essential for checking potential collisions in the robot's environment.

### Inverse Kinematics (IK)

**Inverse Kinematics** is the inverse problem: calculating the joint angles or positions required to achieve a desired pose of the end-effector. This problem is significantly more complex than FK and can have:

*   **Multiple Solutions:** A robot arm might be able to reach a target pose in several different configurations (e.g., "elbow up" vs. "elbow down").
*   **No Solutions:** The target pose might be physically unreachable due to joint limits or the robot's workspace boundaries.
*   **Singularities:** Certain joint configurations (e.g., fully extended arm) can lead to mathematical singularities where small changes in end-effector pose require large, or even infinite, changes in joint angles.

**Input:** Desired end-effector pose (position and orientation).
**Output:** Joint angles/positions.

#### Why is IK important?

*   **Task-Space Control:** Most robot tasks are defined in terms of end-effector poses (e.g., "move gripper to pick up this object at X,Y,Z"). IK translates these task-space goals into the joint-space commands that the robot can execute.
*   **Path Planning:** Robot motion planners typically generate a path for the end-effector in 3D space, and IK is then used to find the corresponding joint trajectories.

### Solving Kinematics Problems

*   **Analytical Solutions:** Possible for simpler robot geometries (e.g., 3-DOF planar arms). These involve direct mathematical equations.
*   **Numerical Solutions:** Required for complex robots with many degrees of freedom. These involve iterative optimization algorithms to find a solution that minimizes the error between the current and desired end-effector poses. Libraries like KDL (Kinematics and Dynamics Library) and MoveIt (which uses KDL and other solvers) provide robust numerical IK solutions for ROS 2.

## 6.2: Understanding URDF and xacro

In Week 4, we briefly introduced URDF (Unified Robot Description Format) as the standard for describing robot models in ROS. Now, we'll delve deeper into its structure and introduce **xacro**, an XML macro language that greatly enhances URDF's flexibility and reusability.

### URDF Revisited:

As a reminder, URDF defines a robot as a tree-like structure of:

*   **`<link>`:** Represents a rigid body (e.g., a robot's base, arm segment, wheel). Each link has:
    *   **Inertial Properties:** Mass, center of mass, and inertia tensor, critical for physics simulation.
    *   **Visual Properties:** Geometry (box, cylinder, sphere, mesh) and material (color, texture) for rendering.
    *   **Collision Properties:** Geometry used for collision detection in simulations.
*   **`<joint>`:** Defines how two links are connected and their relative motion. Key attributes include:
    *   `name`: Unique identifier for the joint.
    *   `type`: (e.g., `revolute`, `prismatic`, `fixed`, `continuous`).
    *   `parent`, `child`: Specifies the two links connected by the joint.
    *   `origin`: The pose of the child link relative to the parent link.
    *   `axis`: The axis of rotation for revolute joints or translation for prismatic joints.
    *   `limit`: Defines software limits for joint position, velocity, and effort.

### The Problem with Plain URDF:

While powerful, plain URDF can become verbose and redundant for complex robots, especially those with many similar parts (e.g., a legged robot with identical leg segments). Imagine copying and pasting the same link definition multiple times, only changing a few parameters. This is where `xacro` comes in.

### Introducing xacro: XML Macros for ROS

**xacro** (XML Macros) is an XML preprocessor that allows you to use macros, variables, and mathematical expressions within your URDF files. It helps to:

*   **Reduce Redundancy:** Define common components once and reuse them throughout the robot description.
*   **Improve Readability:** Make your robot descriptions more concise and easier to understand.
*   **Parametrize Models:** Easily change robot dimensions, masses, or other properties by modifying variables at the top of the file.

#### Basic xacro Concepts:

1.  **`<xacro:macro>`:** Defines a reusable block of XML.
    ```xml
    <xacro:macro name="basic_link" params="link_name size mass">
      <link name="${link_name}">
        <visual><geometry><box size="${size} ${size} ${size}" /></geometry></visual>
        <collision><geometry><box size="${size} ${size} ${size}" /></geometry></collision>
        <inertial>
          <mass value="${mass}" />
          <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001" />
        </inertial>
      </link>
    </xacro:macro>
    ```
    *   `name`: Name of the macro.
    *   `params`: List of parameters the macro accepts.
    *   `${param_name}`: Syntax for using parameters within the macro.

2.  **`<xacro:arg>`:** Defines arguments that can be passed to the xacro file from the command line or a launch file.
    ```xml
    <xacro:arg name="arm_length" default="0.3" />
    ```

3.  **`<xacro:property>`:** Defines variables that can be used throughout the xacro file.
    ```xml
    <xacro:property name="M_PI" value="3.1415926535897931" />
    ```

4.  **Usage:** To use a macro, simply call it:
    ```xml
    <xacro:basic_link link_name="left_leg_link" size="0.1" mass="1.0" />
    ```

### Example: Converting a Simple URDF to xacro

Let's convert our `my_arm.urdf` to `my_arm.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot name="my_simple_arm" xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- Properties -->
  <xacro:property name="base_size" value="0.1" />
  <xacro:property name="link1_length" value="0.2" />
  <xacro:property name="joint_limit" value="1.57" />

  <!-- Macro for a generic box link -->
  <xacro:macro name="box_link" params="name size mass color_r color_g color_b">
    <link name="${name}">
      <visual>
        <geometry><box size="${size} ${size} ${size}" /></geometry>
        <material name="${name}_mat"><color rgba="${color_r} ${color_g} ${color_b} 1" /></material>
      </visual>
      <collision><geometry><box size="${size} ${size} ${size}" /></geometry></collision>
      <inertial>
        <mass value="${mass}" />
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001" />
      </inertial>
    </link>
  </xacro:macro>

  <!-- Macro for a generic cylindrical link -->
  <xacro:macro name="cylinder_link" params="name radius length mass color_r color_g color_b">
    <link name="${name}">
      <visual>
        <geometry><cylinder radius="${radius}" length="${length}" /></geometry>
        <material name="${name}_mat"><color rgba="${color_r} ${color_g} ${color_b} 1" /></material>
      </visual>
      <collision><geometry><cylinder radius="${radius}" length="${length}" /></geometry></collision>
      <inertial>
        <mass value="${mass}" />
        <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001" />
      </inertial>
    </link>
  </xacro:macro>

  <!-- Base Link using macro -->
  <xacro:box_link name="base_link" size="${base_size}" mass="0.5" color_r="0" color_g="0" color_b="1" />

  <!-- Joint 1 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link" />
    <child link="link1" />
    <origin xyz="0 0 ${base_size/2 + link1_length/2 - 0.05}" rpy="0 0 0" /> <!-- Example of math -->
    <axis xyz="0 0 1" />
    <limit lower="-${joint_limit}" upper="${joint_limit}" effort="100" velocity="0.5" />
  </joint>

  <!-- Link 1 using macro -->
  <xacro:cylinder_link name="link1" radius="0.02" length="${link1_length}" mass="0.2" color_r="1" color_g="0" color_b="0" />

</robot>
```

To process this `xacro` file into a plain URDF, you would typically use:
```bash
ros2 run xacro xacro my_arm.urdf.xacro > my_arm_processed.urdf
```
xacro is a crucial tool for managing complex robot descriptions, making your URDF files more maintainable, scalable, and easier to read.

## 6.3: Implementing a Simple Arm Controller

Controlling a robot arm, whether real or simulated, requires a well-defined control system. In ROS 2, `ros2_control` is the standard framework for this. It provides a generic and reusable interface for robot hardware, abstracting away the low-level details of motor drivers and sensors.

### Understanding `ros2_control`:

`ros2_control` is a set of packages that collectively provide a control framework for robots. Its key components include:

*   **Hardware Interfaces:** These are software interfaces that communicate directly with your robot's hardware (e.g., motor drivers, encoders). They provide a standardized way to read joint states and command joint efforts, velocities, or positions.
*   **Controllers:** These are software components that implement specific control algorithms (e.g., PID controllers for joint position, impedance controllers for force control). They read data from the hardware interfaces, compute control outputs, and send commands back to the hardware interfaces.
*   **Controller Manager:** A ROS 2 node that manages the lifecycle of controllers (loading, starting, stopping, unloading). It orchestrates the flow of data between hardware interfaces and controllers.

### Steps to Implement a Controller:

1.  **Define Hardware Interface in URDF:**
    Your robot's URDF (or xacro) file needs to include a `<ros2_control>` tag. This tag specifies:
    *   The type of hardware (e.g., `DiffDriveController`, `JointGroupPositionController`).
    *   The hardware interface plugins that communicate with your specific motors/actuators.
    *   The joints that this hardware interface controls.

    *Example snippet for a simple arm joint:*
    ```xml
    <ros2_control name="my_arm_controller" type="system">
      <hardware>
        <plugin>mock_components/GenericSystem</plugin> <!-- For simulation or simple hardware -->
      </hardware>
      <joint name="joint1">
        <command_interface name="position">
          <param name="min">-${joint_limit}</param>
          <param name="max">${joint_limit}</param>
        </command_interface>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
    </ros2_control>
    ```

2.  **Write Controller Configuration (`.yaml`):**
    You need a YAML file to configure your controllers. This specifies which type of controller to use for each joint, its PID gains (if applicable), and other parameters.

    *Example `controller_params.yaml`:*
    ```yaml
    controller_manager:
      ros__parameters:
        update_rate: 100 # Hz

    joint_trajectory_controller:
      ros__parameters:
        command_interfaces:
          - position
        state_interfaces:
          - position
          - velocity
        joints:
          - joint1
    ```

3.  **Launch the Controller:**
    Typically, a ROS 2 launch file is used to:
    *   Load your robot description (URDF/xacro) to the ROS parameter server.
    *   Start the `controller_manager` node.
    *   Load and start your desired controllers (e.g., `joint_trajectory_controller`).

    *Example launch file snippet:*
    ```python
    from launch import LaunchDescription
    from launch_ros.actions import Node
    from launch_ros.parameter_descriptions import ParameterValue
    from launch.substitutions import Command, FindExecutable
    import os

    def generate_launch_description():
        robot_description = ParameterValue(
            Command([FindExecutable(name='xacro'), ' ', os.path.join(
                get_package_share_directory('my_robot_pkg'), 'urdf', 'my_arm.urdf.xacro'
            )])
        )

        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        )

        joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        )

        my_arm_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        )

        return LaunchDescription([
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            my_arm_controller_spawner
        ])
    ```

4.  **Send Commands:**
    Once the controller is active, you can send commands to it using ROS 2 topics. For `joint_trajectory_controller`, you'd publish to `/joint_trajectory_controller/joint_trajectory`.

    *Example command via `ros2 topic pub` (for position control):*
    ```bash
    ros2 topic pub /joint_trajectory_controller/joint_trajectory \
      trajectory_msgs/msg/JointTrajectory "{header: {stamp: {sec: 0}}, \
      joint_names: ['joint1'], points: [{positions: [1.0], time_from_start: {sec: 1}}]}" -1
    ```

This systematic approach using `ros2_control` is fundamental for building modular and reusable control solutions for any robot platform, whether in simulation or on real hardware.