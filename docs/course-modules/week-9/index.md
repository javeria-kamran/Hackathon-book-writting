# Week 9: Navigation and Path Planning

## 9.1: Occupancy Grids and Mapping

For a robot to navigate autonomously in an unknown or partially known environment, it must first build a representation of that environment. **Mapping** is the process by which a robot constructs this representation. One of the most common and robust forms of environmental representation in robotics is the **Occupancy Grid Map**.

### What is an Occupancy Grid Map?

An Occupancy Grid Map is a 2D or 3D probabilistic representation of an environment, discretized into a grid of cells. Each cell stores a probability value representing whether that area of the environment is occupied by an obstacle, is free space, or is unknown.

*   **Occupied:** High probability (e.g., close to 1.0) of containing an obstacle.
*   **Free:** Low probability (e.g., close to 0.0) of being clear space.
*   **Unknown:** Often represented by a 0.5 probability, indicating no information.

This probabilistic approach is crucial because sensor readings are inherently noisy and uncertain.

### How are Occupancy Grids Built?

Occupancy grids are typically built using data from range sensors like LiDAR or depth cameras. The process involves:

1.  **Sensor Readings:** The robot takes measurements of its surroundings (e.g., distance to obstacles).
2.  **Inverse Sensor Model:** For each sensor reading, an inverse sensor model is used to update the probabilities of the cells in the grid.
    *   Cells along the ray from the sensor to the obstacle are marked as "free" (probability decreases towards 0).
    *   The cell at the endpoint of the ray (where an obstacle is detected) is marked as "occupied" (probability increases towards 1).
    *   Cells beyond the detected obstacle remain "unknown" or are updated based on previous measurements.
3.  **Probabilistic Update:** Each cell's probability is continuously updated as new sensor readings come in. The Bayesian inference approach is commonly used, where the new evidence is combined with the prior probability of a cell being occupied.
    ```
    P(occupied | measurement) = P(measurement | occupied) * P(occupied) / P(measurement)
    ```
    This allows the map to converge over time as more data is collected.

### Key Characteristics:

*   **Resolution:** The size of each grid cell. A finer resolution provides more detail but requires more memory and computational power.
*   **Static vs. Dynamic Environments:** Occupancy grids are best suited for static environments. In dynamic environments, moving obstacles can corrupt the map, requiring dynamic mapping techniques or frequent map updates.
*   **2D vs. 3D:** While 2D occupancy grids are common for planar navigation, 3D occupancy grids (often voxel grids) provide richer information but are more computationally intensive.

### Occupancy Grids in ROS 2:

*   The `nav_msgs/msg/OccupancyGrid` message type is the standard for representing 2D occupancy grid maps in ROS 2.
*   The `slam_toolbox` (for 2D) and `rtabmap` (for 3D) packages in ROS 2 are popular choices for building occupancy grid maps using various sensors. These packages implement full **SLAM (Simultaneous Localization and Mapping)** capabilities, allowing the robot to build a map while simultaneously keeping track of its own position within that map.

Occupancy grids provide a clear, intuitive, and robust way for robots to understand their spatial surroundings, forming a critical basis for path planning and navigation.

## 9.2: The Nav2 Stack in ROS 2

**Nav2 (ROS 2 Navigation Stack)** is the premier navigation framework for mobile robots in ROS 2. It's a collection of modular ROS 2 packages designed to help a robot autonomously navigate from a starting pose to a goal pose while avoiding obstacles. Nav2 is highly configurable, allowing it to be adapted to a wide range of robot platforms and environments.

### Architecture Overview:

Nav2 is not a single node but rather a coordinated suite of nodes and plugins that work together. Key components include:

1.  **Robot Localization:**
    *   **AMCL (Adaptive Monte Carlo Localization):** A probabilistic localization system for a 2D mobile robot moving in a known map. It uses a particle filter to track the robot's pose.
    *   **LiDAR-based Localization:** Methods that use LiDAR scans to match against a pre-built map for highly accurate localization.
    *   **Odometry Fusion:** Often integrated with `robot_localization` (as discussed in Week 8) to fuse data from multiple sensors for robust pose estimation.

2.  **Mapping:**
    *   While Nav2 itself focuses on navigation in *known* maps, it integrates seamlessly with mapping solutions like `slam_toolbox` (for building 2D occupancy grids) or `rtabmap` (for 3D) to create the maps it needs.

3.  **Path Planning:** Nav2 separates global and local planning:
    *   **Global Planner:** Generates an optimal (e.g., shortest, safest) path from the robot's current location to the goal location on the static (known) map. Examples include A* and Dijkstra's algorithms, often implemented as plugins.
    *   **Local Planner (Controller):** Takes the global path and generates velocity commands for the robot to follow the path while dynamically avoiding unforeseen obstacles (e.g., people, moving carts) and adapting to local changes in the environment. Examples include DWA (Dynamic Window Approach), TEB (Timed Elastic Band), and RPP (Regulated Pure Pursuit). These are also implemented as plugins.

4.  **Behavior Tree:** Nav2 uses **Behavior Trees** to orchestrate the high-level navigation logic. The behavior tree defines sequences of actions (e.g., "navigate to goal," "recover from collision," "clear costmap") and conditions that determine the robot's overall behavior. This makes the navigation logic modular, readable, and easy to extend.

5.  **Costmaps:**
    *   **Global Costmap:** A representation of the static environment, derived from the occupancy grid map, with inflated obstacles to account for the robot's size and safety margins.
    *   **Local Costmap:** A smaller, dynamic map around the robot that incorporates real-time sensor readings to detect and avoid dynamic obstacles. Both costmaps are used by the planners.

### Nav2 Workflow:

1.  **Map Creation:** A map of the environment is created (e.g., using SLAM).
2.  **Localization:** The robot continuously estimates its position on this map.
3.  **Goal Setting:** A user or another agent sets a navigation goal (e.g., a pose message to `/goal_pose`).
4.  **Behavior Tree Execution:** The behavior tree orchestrates the navigation:
    *   A global path is planned.
    *   A local controller follows the global path while avoiding local obstacles.
    *   Recovery behaviors are triggered if the robot gets stuck.
5.  **Velocity Commands:** The local planner sends velocity commands (e.g., to `/cmd_vel`) to the robot's base controller.

Nav2 provides a powerful and extensible framework for achieving autonomous navigation, a cornerstone capability for any mobile robot.

## 9.3: Configuring and Launching a Navigation System

Bringing together all the components of the Nav2 stack into a functional navigation system requires careful configuration and the use of ROS 2 launch files. This section will walk you through the general process of setting up and launching a complete Nav2 system for a mobile robot.

### 1. Prerequisite: Robot Description

Before you can configure Nav2, your robot needs a complete URDF/xacro description that includes:

*   All links and joints, including the base link, wheels, and any sensors.
*   `ros2_control` configuration for your robot's actuators.
*   TF (Transformations) definitions for all links and sensors.

### 2. Nav2 Configuration Files (`.yaml`)

Nav2 is configured primarily through YAML parameter files. You'll typically have several configuration files, each for a different aspect of the stack:

*   **`params.yaml` (Main Configuration):** Defines global parameters for the Nav2 stack, such as the `use_sim_time` flag (crucial for simulation), the `autostart` flag for nodes, and paths to other configuration files.
*   **`amcl.yaml` (Localization):** Configures the Adaptive Monte Carlo Localization node, including parameters like initial pose, particle filter settings, and sensor noise models.
*   **`costmap_common_params.yaml` (Costmap Layers):** Defines common parameters for both global and local costmaps, including sensor sources (e.g., LiDAR, depth camera point clouds), obstacle inflation radii, and data layers (e.g., static map layer, obstacle layer, inflation layer).
*   **`global_costmap.yaml`:** Specific parameters for the global costmap, such as its resolution, update frequency, and whether to use a static map.
*   **`local_costmap.yaml`:** Specific parameters for the local costmap, such as its size, resolution, and how frequently to clear old obstacles.
*   **`planner_server.yaml` (Global Planner):** Configures the global planner, including the plugin to use (e.g., `NavFnPlanner`, `SmacPlanner`) and its parameters (e.g., interpolation, smoothing).
*   **`controller_server.yaml` (Local Planner/Controller):** Configures the local planner, including the plugin to use (e.g., `DWAPlannerROS`, `TEBController`) and its parameters (e.g., max velocities, accelerations, obstacle avoidance settings).
*   **`behavior_server.yaml` (Behavior Tree):** Configures the behavior tree, including the XML file that defines the tree's structure and the available recovery behaviors.

### 3. Creating a Nav2 Launch File:

A ROS 2 launch file (typically Python-based) orchestrates the startup of all Nav2 nodes and applies their respective configurations. A typical Nav2 launch file will:

1.  **Declare Launch Arguments:** For flexibility (e.g., `use_sim_time`, `map_yaml_file`).
2.  **Include Robot Description:** Load the robot's URDF/xacro and start `robot_state_publisher` and `joint_state_publisher`.
3.  **Launch Nav2 Nodes:** Use `include` actions to launch the main Nav2 `bringup` launch file, passing in all your custom configuration files.

    *Example snippet for a Nav2 bringup launch file:*
    ```python
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
    from ament_index_python.packages import get_package_share_directory
    import os

    def generate_launch_description():
        nav2_bringup_dir = get_package_share_directory('nav2_bringup')
        my_robot_pkg_dir = get_package_share_directory('my_robot_pkg')

        # Declare launch arguments
        use_sim_time = DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'
        )
        map_yaml_file = DeclareLaunchArgument(
            'map_yaml_file', default_value=os.path.join(my_robot_pkg_dir, 'maps', 'my_map.yaml'),
            description='Full path to map file to load'
        )
        params_file = DeclareLaunchArgument(
            'params_file', default_value=os.path.join(my_robot_pkg_dir, 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use for all Nav2 nodes'
        )

        return LaunchDescription([
            use_sim_time,
            map_yaml_file,
            params_file,

            # Include the Nav2 bringup launch file
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch/bringup_launch.py']),
                launch_arguments={
                    'map': LaunchConfiguration('map_yaml_file'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'params_file': LaunchConfiguration('params_file')
                }.items(),
            )
        ])
    ```

### 4. Running the Navigation System:

1.  **Launch Gazebo (if simulating):**
    ```bash
    ros2 launch gazebo_ros gazebo.launch.py # or your custom world
    ```
2.  **Launch Your Robot's URDF/xacro:**
    ```bash
    ros2 launch my_robot_pkg robot_description.launch.py
    ```
3.  **Launch Nav2:**
    ```bash
    ros2 launch my_robot_pkg navigation_launch.py map:=/path/to/my_map.yaml params_file:=/path/to/nav2_params.yaml use_sim_time:=true
    ```
4.  **Set Initial Pose and Navigation Goal:**
    Use RViz (the ROS 2 visualization tool) to set the robot's initial pose (2D Pose Estimate) and then set a navigation goal (2D Nav Goal).

Configuring and launching Nav2 is a complex but essential step for any autonomous mobile robot. The modularity of Nav2 allows for extensive customization and optimization for specific robot platforms and environments.