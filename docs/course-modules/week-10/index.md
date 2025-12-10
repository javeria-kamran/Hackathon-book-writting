# Week 10: Manipulation and Grasping

## 10.1: Grasping Pipelines and Strategies

Robotic manipulation, particularly grasping, is a complex problem that lies at the intersection of perception, planning, and control. For a robot to successfully pick up an object, it needs to first identify the object, determine a suitable grasp pose, plan the arm trajectory to reach that pose, and then execute the grasp. This section introduces common grasping pipelines and strategies.

### The Grasping Pipeline:

A typical grasping pipeline involves several sequential steps:

1.  **Object Perception:**
    *   **Detection:** Using computer vision (as discussed in Week 7), identify the target object and its 2D location in an image.
    *   **3D Localization:** Using depth sensors (LiDAR, depth camera), estimate the object's 3D pose (position and orientation) in the robot's environment. This often involves point cloud processing and segmentation (Week 8).

2.  **Grasp Pose Generation:**
    Once the object's 3D pose is known, the robot needs to determine *how* to grasp it. A grasp pose defines the position and orientation of the gripper relative to the object.
    *   **Analytic Grasp Planning:** For known objects (e.g., from a CAD model), pre-defined grasp points or analytical methods can be used to calculate stable grasps. This might involve checking for force closure or form closure.
    *   **Data-Driven Grasp Planning:** For novel or unknown objects, data-driven approaches using deep learning are increasingly popular. These models are trained on large datasets of successful grasps and can predict valid grasp poses from raw sensor data (e.g., RGB-D images).
        *   **Grasp Detection Networks:** Models like GraspNet or contact-point prediction networks output a set of candidate grasp poses.
    *   **Grasp Quality Metrics:** Evaluate potential grasp poses based on factors like stability, robustness to disturbances, and accessibility.

3.  **Motion Planning (Arm Trajectory):**
    With a desired grasp pose, the robot arm needs to move from its current configuration to approach and execute the grasp. This involves solving the inverse kinematics (IK) problem (Week 6) to find the joint angles for the arm and then planning a collision-free trajectory.
    *   **Collision Avoidance:** The trajectory must avoid collisions with the environment, the robot's own body, and the target object itself (until the grasp is complete).
    *   **Path Optimization:** Trajectories are optimized for smoothness, speed, and energy efficiency.

4.  **Grasp Execution and Control:**
    The planned trajectory is sent to the robot's arm controllers (Week 6).
    *   **Pre-grasp Approach:** The arm moves to a position just before the grasp.
    *   **Gripper Closure:** The gripper closes around the object. This might involve force control to ensure a firm but not damaging grip.
    *   **Post-grasp Retreat:** The arm moves the grasped object to a safe position.

### Grasping Strategies:

Different objects and tasks require different grasping strategies:

*   **Parallel-Jaw Grippers:** Common for industrial robots. Require precise positioning and known object geometry.
*   **Suction Cups:** Effective for flat, smooth, non-porous surfaces.
*   **Soft Grippers:** Compliant grippers that can adapt to the shape of irregular objects, often used for delicate items.
*   **Multi-Fingered Hands:** Provide high dexterity but are very complex to control.

Effective robotic grasping is a critical skill for any physical AI agent that needs to interact physically with its world, laying the groundwork for complex manipulation tasks.

## 10.2: Introduction to MoveIt 2

**MoveIt 2** is the most widely used software for mobile manipulation and is the standard for robotic manipulation in ROS 2. It provides an easy-to-use framework for motion planning, manipulation, and control of robotic arms and mobile manipulators. MoveIt 2 integrates various components necessary for complex manipulation tasks, including kinematics solvers, collision checkers, planning algorithms, and a user-friendly RViz plugin.

### Key Features of MoveIt 2:

*   **Motion Planning:** MoveIt 2 integrates various state-of-the-art motion planning algorithms (e.g., OMPL - Open Motion Planning Library, STOMP - Stochastic Trajectory Optimization for Motion Planning) to find collision-free paths for robotic arms.
*   **Kinematics:** Provides a robust interface to kinematics solvers (Forward and Inverse Kinematics, as discussed in Week 6) to translate between joint space and Cartesian space.
*   **Collision Checking:** Continuously monitors for collisions between the robot's links, the environment, and any attached objects, using highly optimized collision checking libraries (e.g., FCL - Flexible Collision Library).
*   **State Monitoring:** Monitors the robot's current joint states and end-effector pose.
*   **Perception Integration:** Can integrate with perception sensors (e.g., cameras, depth sensors) to build and update a representation of the environment, including obstacles.
*   **Planning Scene:** Maintains a comprehensive model of the robot and its environment, including attached objects and known obstacles.
*   **RViz Plugin:** A powerful visualization plugin for RViz that allows users to interactively plan motions, set goals, and monitor the robot's state.

### MoveIt 2 Architecture:

MoveIt 2 is a meta-package composed of several key components:

1.  **`move_group` Node:** The central hub of MoveIt 2. It provides a ROS 2 interface for users to interact with MoveIt 2, offering actions (like `MoveGroup` actions for planning and executing trajectories) and services.
2.  **Kinematics Solvers:** Plug-ins that compute FK and IK for the robot.
3.  **Planning Algorithms:** Plug-ins that implement various motion planning strategies.
4.  **Collision Detection:** Components that check for collisions.
5.  **Motion Controllers:** Interfaces with `ros2_control` (Week 6) to execute the planned trajectories on the real or simulated robot.
6.  **Planning Scene Monitor:** Subscribes to sensor data (e.g., point clouds) to keep the planning scene (robot + environment) up-to-date with current obstacle information.

### Setting up MoveIt 2 for Your Robot:

The process of setting up MoveIt 2 for a new robot typically involves using the **MoveIt Setup Assistant**, a graphical user interface (GUI) that guides you through:

1.  **Loading Your URDF:** Importing your robot's URDF/xacro model.
2.  **Defining Kinematic Chains:** Specifying which joints form a kinematic chain (e.g., "right_arm," "left_leg").
3.  **Adding End-Effectors:** Defining tool frames for grippers or other end-effectors.
4.  **Generating SRDF:** Creating the **SRDF (Semantic Robot Description Format)** file, which augments the URDF with semantic information (e.g., groups of joints, passive joints, end-effectors, default poses, self-collision matrices).
5.  **Generating Configuration Files:** Creating a set of YAML and other configuration files specific to your robot, including joint limits, kinematics solvers, and planning parameters.

### MoveIt 2 Workflow:

1.  **Initialize Planning Scene:** MoveIt 2 loads the robot's URDF/SRDF and monitors sensor data for obstacles.
2.  **Set Goal:** A user or an external agent provides a desired end-effector pose or joint configuration as a goal.
3.  **Plan:** MoveIt 2 uses its motion planners to find a collision-free path (joint trajectory) from the current state to the goal state.
4.  **Execute:** The planned trajectory is sent to the robot's `ros2_control` interface for execution.

MoveIt 2 significantly simplifies the development of complex manipulation tasks, providing a robust and versatile framework that is essential for agentic robots interacting with their physical surroundings.

## 10.3: Planning and Executing a Pick-and-Place Task

A **pick-and-place** task is a quintessential manipulation challenge for robots, combining perception, motion planning, and control to move an object from one location to another. This section outlines the typical pipeline for planning and executing such a task using ROS 2 and MoveIt 2.

### The Pick-and-Place Pipeline:

1.  **Perceive the Object:**
    *   **Object Detection & Localization (Week 7, 8):** The robot uses its vision sensors (e.g., RGB-D camera) to detect the target object and determine its precise 3D pose (position and orientation) in the world. This step may involve segmentation to isolate the object from its surroundings.

2.  **Generate Grasp Poses (Week 10.1):**
    *   Based on the object's 3D pose and geometry, potential grasp poses are calculated. These poses define where and how the gripper should approach and secure the object. Grasp quality metrics are used to select the most stable and robust grasp.

3.  **Plan the "Pick" Motion:**
    The "pick" operation is often broken down into several sub-steps, each requiring motion planning:
    *   **Pre-grasp Approach:** Move the end-effector to a safe position above the object, avoiding collisions.
    *   **Grasp Approach:** Move the end-effector towards the object, preparing to grasp it.
    *   **Close Gripper:** Actuate the gripper to secure the object.
    *   **Post-grasp Retreat:** Lift the object slightly away from its original position to ensure it's clear of any obstacles.

4.  **Plan the "Place" Motion:**
    Similarly, the "place" operation involves multiple motion planning steps:
    *   **Transport:** Move the grasped object from the pick location to a safe position above the desired place location.
    *   **Place Approach:** Move the end-effector with the object towards the target placement pose.
    *   **Open Gripper:** Release the object.
    *   **Post-place Retreat:** Move the end-effector away from the placed object.

### MoveIt 2 Integration for Pick-and-Place:

MoveIt 2 provides a high-level API to simplify the planning and execution of pick-and-place tasks. The `move_group` interface offers `pick` and `place` actions that encapsulate the complexities of multi-stage motion planning, collision checking, and controller interaction.

#### Example MoveIt 2 Python Script (Conceptual):

```python
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import Grasp, PlaceLocation
from moveit_msgs.srv import GetPlanningScene
from geometry_msgs.msg import PoseStamped, TransformStamped
from moveit_msgs.action import Pick, Place
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
import tf2_ros
import time

class PickAndPlaceCommander(Node):
    def __init__(self):
        super().__init__('pick_and_place_commander')
        self.pick_action_client = ActionClient(self, Pick, 'pick')
        self.place_action_client = ActionClient(self, Place, 'place')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info('Pick and Place Commander initialized.')

    def send_pick_goal(self, object_name, object_pose, gripper_frame):
        # Create a Grasp message
        grasp = Grasp()
        grasp.pre_grasp_approach.direction.vector.z = 1.0 # Approach from above
        grasp.pre_grasp_approach.min_distance = 0.09
        grasp.pre_grasp_approach.desired_distance = 0.12

        grasp.post_grasp_retreat.direction.vector.z = 1.0 # Retreat upwards
        grasp.post_grasp_retreat.min_distance = 0.1
        grasp.post_grasp_retreat.desired_distance = 0.25

        grasp.grasp_pose = object_pose # The pose of the gripper relative to the object for grasping
        grasp.id = "my_grasp"
        grasp.pre_grasp_posture.joint_names = ["gripper_finger_joint_l", "gripper_finger_joint_r"]
        grasp.pre_grasp_posture.points.append(JointTrajectoryPoint(positions=[0.04, 0.04], time_from_start=rclpy.duration.Duration(seconds=0.5).to_msg()))
        grasp.grasp_posture.joint_names = ["gripper_finger_joint_l", "gripper_finger_joint_r"]
        grasp.grasp_posture.points.append(JointTrajectoryPoint(positions=[0.0, 0.0], time_from_start=rclpy.duration.Duration(seconds=0.5).to_msg()))

        goal_msg = Pick.Goal()
        goal_msg.object_name = object_name
        goal_msg.group_name = "arm" # Planning group name
        goal_msg.possible_grasps.append(grasp)
        # Add other pick goal parameters (e.g., support surfaces)

        self.get_logger().info(f'Sending Pick goal for {object_name}...')
        self.pick_action_client.wait_for_server()
        self._send_pick_goal_future = self.pick_action_client.send_goal_async(goal_msg)
        self._send_pick_goal_future.add_done_callback(self.pick_goal_response_callback)

    def pick_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Pick goal rejected.')
            return
        self.get_logger().info('Pick goal accepted.')
        self._get_pick_result_future = goal_handle.get_result_async()
        self._get_pick_result_future.add_done_callback(self.pick_result_callback)

    def pick_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Pick successful!')
            # Now proceed to place
        else:
            self.get_logger().error(f'Pick failed with status: {status}')

    # Similar functions for send_place_goal, place_goal_response_callback, place_result_callback

def main(args=None):
    rclpy.init(args=args)
    pick_place_node = PickAndPlaceCommander()

    # Example usage:
    # Assume object_pose is obtained from perception and is in the robot's base frame
    object_pose_stamped = PoseStamped()
    object_pose_stamped.header.frame_id = "robot_base_link"
    object_pose_stamped.pose.position.x = 0.5
    object_pose_stamped.pose.position.y = 0.0
    object_pose_stamped.pose.position.z = 0.1
    object_pose_stamped.pose.orientation.w = 1.0 # Identity orientation

    pick_place_node.send_pick_goal("red_block", object_pose_stamped, "gripper_link")

    rclpy.spin(pick_place_node)
    pick_place_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    ```

### Considerations for Robust Pick-and-Place:

*   **Error Handling:** Implement robust error handling for failed plans, collisions, or dropped objects.
*   **Perception Feedback Loop:** Integrate continuous perception to re-localize objects if they move or to verify successful grasps.
*   **Compliance:** For delicate objects, use compliant control strategies to prevent damage.
*   **Sim-to-Real Transfer:** Thoroughly test your pick-and-place pipeline in simulation before deploying to a physical robot.

A well-implemented pick-and-place pipeline is a cornerstone for robots that need to perform useful work in unstructured environments.