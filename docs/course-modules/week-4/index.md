# Week 4: Simulating Robots with Gazebo

## 4.1: Introduction to Gazebo and SDF

**Gazebo** is a powerful 3D robotics simulator that allows you to accurately and efficiently test your robot algorithms in virtual environments. It provides robust physics engines, high-quality graphics, and convenient programmatic interfaces for simulating robots, sensors, and objects. Using a simulator like Gazebo is crucial for rapid prototyping, testing edge cases, and developing behaviors that might be dangerous or expensive to test on physical hardware.

The simulated world in Gazebo is described using **SDF (Simulation Description Format)** files. SDF is an XML format used to describe robots, static objects (like walls or furniture), and entire environments. It's designed to be a universal format for describing objects in any robot simulator, although its primary usage is with Gazebo.

### Key Features of Gazebo:

*   **Physics Engine:** Gazebo integrates with various physics engines (e.g., ODE, Bullet, DART, Simbody) to provide realistic rigid-body dynamics, friction, and collision detection.
*   **3D Graphics:** High-fidelity rendering allows for realistic visualization of your robot and its environment, which is particularly useful for visual perception tasks.
*   **Sensor Simulation:** Simulate a wide array of sensors, including cameras (monocular, stereo, depth), LIDAR, IMUs, force-torque sensors, and more. This allows you to develop sensor processing algorithms without needing physical hardware.
*   **Plugin Architecture:** Gazebo's modular design allows users to create custom plugins to control robot dynamics, interact with sensors, or modify the environment using C++ or Python. ROS 2 integration is often achieved through Gazebo plugins.

### Understanding SDF:

An SDF file typically defines:

*   **`<world>`:** The top-level element that contains all entities in the simulation (models, lights, sensors, physics properties).
*   **`<model>`:** Represents a robot or any other movable object. A model is composed of `<link>` and `<joint>` elements.
*   **`<link>`:** A rigid body representing a part of the robot (e.g., a limb, a wheel). Links have inertial properties (mass, inertia), visual properties (shape, color, texture), and collision properties (shape for physics interaction).
*   **`<joint>`:** Connects two links, defining their relative motion (e.g., revolute, prismatic, fixed).
*   **`<sensor>`:** Defines a sensor (e.g., camera, lidar) and its properties.
*   **`<plugin>`:** Allows for custom behavior and interaction with the simulator, often used for ROS 2 integration.

While writing complex SDF files manually can be tedious, understanding their structure is vital for customizing existing models and debugging simulation issues. Often, ROS 2 robot descriptions (URDF) are converted to SDF for use in Gazebo.

## 4.2: Building a Robot Model: URDF to SDF

Creating a detailed and accurate robot model is fundamental for effective simulation. In ROS 2, robot models are typically described using **URDF (Unified Robot Description Format)**. URDF is an XML format for describing the kinematic and dynamic properties of a robot. For use in Gazebo, URDF models are often converted to SDF, or an SDF wrapper is used around the URDF.

### Understanding URDF:

URDF defines a robot as a tree-like structure of:

*   **`<link>`:** Represents a rigid body of the robot, similar to SDF. It includes inertial, visual, and collision properties.
*   **`<joint>`:** Describes the connection between two links, defining their relative motion. URDF supports various joint types (revolute, prismatic, fixed, continuous, planar, floating).

### Why URDF and not just SDF?

*   **ROS Standard:** URDF is the standard format for robot descriptions within the ROS ecosystem, used by many ROS tools for visualization (RViz), motion planning (MoveIt), and more.
*   **Simplicity for Kinematics:** URDF is generally simpler to define for robot kinematics (how parts move relative to each other) compared to the more comprehensive SDF which also handles simulation-specific properties like physics and sensor definitions.

### Building a Simple URDF Model:

Let's consider a simple two-link arm.

1.  **Create a new `urdf` directory** within your ROS 2 package (`my_robot_pkg`):
    ```bash
    mkdir -p ~/ros2_ws/src/my_robot_pkg/urdf
    ```
2.  **Create `my_arm.urdf`:**
    ```xml
    <?xml version="1.0"?>
    <robot name="my_simple_arm">
      <!-- Base Link -->
      <link name="base_link">
        <visual>
          <geometry><box size="0.1 0.1 0.1" /></geometry>
          <material name="blue"><color rgba="0 0 1 1" /></material>
        </visual>
        <collision><geometry><box size="0.1 0.1 0.1" /></geometry></collision>
        <inertial>
          <mass value="0.5" />
          <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001" />
        </inertial>
      </link>

      <!-- Joint 1 -->
      <joint name="joint1" type="revolute">
        <parent link="base_link" />
        <child link="link1" />
        <origin xyz="0 0 0.1" rpy="0 0 0" />
        <axis xyz="0 0 1" />
        <limit lower="-1.57" upper="1.57" effort="100" velocity="0.5" />
      </joint>

      <!-- Link 1 -->
      <link name="link1">
        <visual>
          <geometry><cylinder radius="0.02" length="0.2" /></geometry>
          <material name="red"><color rgba="1 0 0 1" /></material>
        </visual>
        <collision><geometry><cylinder radius="0.02" length="0.2" /></geometry></collision>
        <inertial>
          <mass value="0.2" />
          <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001" />
        </inertial>
      </link>
    </robot>
    ```
    This defines a `base_link`, a `link1`, and a `revolute` joint connecting them.

### Integrating URDF with Gazebo:

When launching a robot in Gazebo, you typically use `ros2_gazebo` launch files. These launch files handle the conversion of URDF to SDF (if needed) and spawn the robot into the simulation. You will learn more about launch files in a later week. For now, understand that your URDF model is the primary description, and Gazebo uses it via conversion or direct plugin loading.

## 4.3: Spawning and Controlling a Robot in a Simulated World

Once you have a robot model (either a custom URDF or an existing SDF model), the next step is to spawn it into a Gazebo world and begin interacting with it. This involves using Gazebo's ROS 2 interfaces and typically a ROS 2 launch file to automate the process.

### Spawning a Robot with ROS 2:

To spawn a robot, you generally use the `spawn_entity.py` script provided by the `ros_gz_sim` package (or `gazebo_ros` in older setups). This script takes your robot description (e.g., URDF or SDF) and inserts it into the running Gazebo simulation.

1.  **Ensure Gazebo is Running:** First, you need to launch Gazebo itself. This is often done via a ROS 2 launch file.
    ```bash
    ros2 launch gazebo_ros gazebo.launch.py # Launches an empty Gazebo world
    ```

2.  **Spawn Your Robot:** In a new terminal, you can spawn your URDF model. You typically pass the robot description as a string.
    ```bash
    ros2 run ros_gz_sim create --entity my_robot --topic robot_description
    ```
    *   `--entity my_robot`: The name of the robot in Gazebo.
    *   `--topic robot_description`: The ROS 2 topic where the robot's URDF/SDF description is published. This is usually handled by a `robot_state_publisher` node.

### Basic Robot Control:

Once spawned, you can control your robot in several ways:

1.  **ROS 2 Control (Recommended):** The most common and robust way to control robots in Gazebo is through `ros2_control`. This framework allows you to define hardware interfaces (e.g., joint position controllers, velocity controllers) and exposes them as ROS 2 topics or services.
    *   You would typically add `<ros2_control>` tags to your URDF to specify these interfaces.
    *   Then, you load and start controllers using `ros2_control`'s controller manager.

2.  **Direct Gazebo Plugins:** You can write custom Gazebo plugins that directly interact with the physics engine to apply forces or torques to your robot's joints. This offers fine-grained control but might bypass some of the ROS 2 ecosystem benefits.

### Example: Controlling a Simple Joint

Let's assume you've configured `ros2_control` for a joint named `joint1` in your `my_arm` robot. You would publish commands to its controller topic.

1.  **Load and Start Controller (Example):**
    ```bash
    ros2 control load_controller my_joint_position_controller --set-state active
    ```

2.  **Publish a Command:** You can send a position command to the joint.
    ```bash
    ros2 topic pub /my_joint_position_controller/commands std_msgs/msg/Float64 "{data: 1.0}" -1
    ```
    This would attempt to move `joint1` to a position of 1.0 radians.

This process of spawning and controlling robots in Gazebo forms the backbone of robot simulation, allowing you to iterate quickly on your designs and control algorithms.