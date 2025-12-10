# Week 3: Fundamentals of ROS 2

## 3.1: ROS 2 Architecture: Nodes, Topics, Services, Actions

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's not an operating system in the traditional sense, but rather a set of software libraries and tools that help you build robot applications. Its distributed, asynchronous architecture is built around several core concepts:

*   **Nodes:** The fundamental building blocks of a ROS 2 system. A node is essentially an executable process that performs computation (e.g., a node to control motors, a node to read sensor data, a node for path planning). Nodes should ideally be designed to perform a single, well-defined task.
*   **Topics:** The primary mechanism for asynchronous, one-way communication between nodes. Nodes can publish data to a topic, and other nodes can subscribe to that topic to receive the data. This is ideal for streaming data like sensor readings (e.g., camera images, lidar scans) or control commands.
    *   **Publishers:** Nodes that send data to a topic.
    *   **Subscribers:** Nodes that receive data from a topic.
*   **Services:** Used for synchronous, request-response communication between nodes. When a client node sends a request to a service server, the client typically blocks (waits) until it receives a response. Services are suitable for tasks that require an immediate answer, like querying a robot's state or triggering a specific action (e.g., "get current joint angles," "toggle a light").
    *   **Service Servers:** Nodes that provide a service and respond to requests.
    *   **Service Clients:** Nodes that request a service.
*   **Actions:** Provide a long-running, asynchronous, goal-feedback-result communication pattern. Unlike services, actions allow a client to send a goal, receive continuous feedback while the action is executing, and eventually get a result. This is perfect for tasks like "navigate to a point" or "pick up an object," where you want to monitor progress and potentially cancel the operation.
    *   **Action Servers:** Nodes that offer an action and provide feedback.
    *   **Action Clients:** Nodes that request an action and receive feedback.

This modular architecture allows for the decomposition of complex robotic systems into smaller, manageable, and independently executable units that can communicate effectively.

## 3.2: Creating a ROS 2 Workspace and Packages

In ROS 2, a **workspace** is a directory where you collect, build, and install your ROS 2 packages. A **package** is the fundamental unit for organizing ROS 2 code. It contains source files, scripts, configurations, and other resources related to a specific piece of robot functionality.

### Setting up a ROS 2 Workspace:

1.  **Create the Workspace Directory:** Choose a convenient location for your workspace, typically in your home directory.
    ```bash
    mkdir -p ~/ros2_ws/src
    ```
    *   `~/ros2_ws`: The root of your workspace.
    *   `src`: This directory will contain the source code of your packages.

2.  **Initialize `colcon`:** `colcon` is the build tool used in ROS 2. It needs to be initialized in your workspace.
    ```bash
    cd ~/ros2_ws
    colcon build --symlink-install
    ```
    This command will build any packages found in the `src` directory (initially none) and create `install`, `log`, and `build` directories. The `--symlink-install` option is useful during development as it allows you to modify source code without rebuilding.

3.  **Source the Workspace:** To make your workspace packages available to your shell, you need to source the setup file.
    ```bash
    source install/setup.bash
    ```
    It's common practice to add this line to your `~/.bashrc` file so that your workspace is sourced automatically every time you open a new terminal:
    ```bash
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

### Creating Your First ROS 2 Package:

ROS 2 provides tools to easily create new packages. We'll use `ros2 pkg create` for this.

1.  **Navigate to the `src` directory:**
    ```bash
    cd ~/ros2_ws/src
    ```

2.  **Create a new package:**
    ```bash
    ros2 pkg create --build-type ament_python my_robot_pkg
    ```
    *   `--build-type ament_python`: Specifies that this will be a Python package. For C++ packages, you would use `ament_cmake`.
    *   `my_robot_pkg`: The name of your new package.

3.  **Examine the Package Structure:** After creation, `my_robot_pkg` will have a basic directory structure:
    ```
    my_robot_pkg/
    ├── package.xml          # Package metadata (dependencies, version, etc.)
    ├── setup.py             # Python package build configuration
    ├── setup.cfg            # Python package configuration
    ├── resource/            # Contains marker file for package identification
    └── my_robot_pkg/        # Python module directory
        └── __init__.py      # Marks the directory as a Python package
    ```

4.  **Build the New Package:** After creating a new package, you need to build your workspace again.
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_robot_pkg
    source install/setup.bash # Re-source to make the new package available
    ```
    `--packages-select` is useful to build only specific packages, saving time.

This foundation of workspaces and packages is essential for organizing your robotics projects in a scalable and collaborative manner.

## 3.3: Writing a Simple Publisher and Subscriber in Python

Understanding how to create ROS 2 nodes that communicate using topics is fundamental. This section will guide you through writing a basic publisher and subscriber in Python. The publisher will send "Hello ROS 2!" messages, and the subscriber will receive and print them.

### Prerequisites:

*   A ROS 2 workspace (e.g., `~/ros2_ws`)
*   A ROS 2 Python package (e.g., `my_robot_pkg`)

### 1. The Publisher Node (`simple_publisher.py`):

Create a new file `~/ros2_ws/src/my_robot_pkg/my_robot_pkg/simple_publisher.py` with the following content:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Standard ROS 2 message type for strings

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher') # Initialize the node with the name 'simple_publisher'
        self.publisher_ = self.create_publisher(String, 'chatter', 10) # Create a publisher
        # Arguments: message type, topic name, QoS profile depth
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # Create a timer to publish periodically
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.i}' # Create the message
        self.publisher_.publish(msg) # Publish the message
        self.get_logger().info(f'Publishing: "{msg.data}"') # Log the published message
        self.i += 1

def main(args=None):
    rclpy.init(args=args) # Initialize ROS 2 communications
    simple_publisher = SimplePublisher() # Create an instance of our publisher node
    rclpy.spin(simple_publisher) # Keep the node alive and processing events
    simple_publisher.destroy_node() # Cleanly destroy the node
    rclpy.shutdown() # Shutdown ROS 2 communications

if __name__ == '__main__':
    main()
```

### 2. The Subscriber Node (`simple_subscriber.py`):

Create a new file `~/ros2_ws/src/my_robot_pkg/my_robot_pkg/simple_subscriber.py` with the following content:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Standard ROS 2 message type for strings

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber') # Initialize the node with the name 'simple_subscriber'
        self.subscription = self.create_subscription(
            String,
            'chatter', # Subscribe to the 'chatter' topic
            self.listener_callback, # Callback function when a message is received
            10) # QoS profile depth
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"') # Log the received message

def main(args=None):
    rclpy.init(args=args) # Initialize ROS 2 communications
    simple_subscriber = SimpleSubscriber() # Create an instance of our subscriber node
    rclpy.spin(simple_subscriber) # Keep the node alive and processing events
    simple_subscriber.destroy_node() # Cleanly destroy the node
    rclpy.shutdown() # Shutdown ROS 2 communications

if __name__ == '__main__':
    main()
```

### 3. Update `setup.py`:

To make these Python scripts executable ROS 2 nodes, you need to update your `~/ros2_ws/src/my_robot_pkg/setup.py` file. Add the following to the `entry_points` dictionary:

```python
# ... (existing setup.py content) ...

entry_points={
    'console_scripts': [
        'simple_publisher = my_robot_pkg.simple_publisher:main',
        'simple_subscriber = my_robot_pkg.simple_subscriber:main',
    ],
},
```

### 4. Build and Run:

1.  **Build the workspace:**
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_robot_pkg
    ```
2.  **Source the workspace:**
    ```bash
    source install/setup.bash
    ```
3.  **Run the publisher (in a new terminal):**
    ```bash
    ros2 run my_robot_pkg simple_publisher
    ```
4.  **Run the subscriber (in another new terminal):**
    ```bash
    ros2 run my_robot_pkg simple_subscriber
    ```

You should see the publisher sending messages and the subscriber receiving them in real-time. This demonstrates the fundamental topic-based communication in ROS 2.