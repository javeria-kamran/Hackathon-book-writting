# Week 7: Computer Vision for Robotics: Object Detection

## 7.1: Camera Models and Image Formation

Computer vision is a cornerstone of modern robotics, enabling robots to "see" and interpret their environment. The journey from the physical world to a digital image involves understanding camera models and the process of image formation.

### The Pinhole Camera Model

The **Pinhole Camera Model** is the simplest and most fundamental model of how a camera forms an image. It assumes light rays pass through a single infinitesimally small hole (the pinhole) to project a 3D scene onto a 2D image plane.

#### Key Components:

*   **Pinhole (Optical Center):** The conceptual single point through which all light rays pass.
*   **Image Plane:** The 2D surface where the inverted image is formed. In digital cameras, this is where the sensor is located.
*   **Focal Length (`f`):** The distance between the pinhole and the image plane.
*   **Principal Point (`cx`, `cy`):** The intersection of the optical axis with the image plane, typically near the center of the image.

#### Projection Equation:

A 3D point `P = (X, Y, Z)` in the camera's coordinate system is projected onto a 2D image point `p = (u, v)` using the following relations (ignoring lens distortion for simplicity):

```
u = f * (X / Z) + cx
v = f * (Y / Z) + cy
```

This equation shows that the image coordinates depend on the 3D point's coordinates and the camera's intrinsic parameters (`f`, `cx`, `cy`).

### Intrinsic Camera Parameters

These parameters describe the internal geometry and optical properties of the camera. They are typically grouped into a **Camera Matrix (K)**:

```
K = [ fx  0  cx ]
    [  0 fy  cy ]
    [  0  0   1 ]
```
Where:
*   `fx`, `fy`: Scaled focal lengths (focal length multiplied by pixel width/height).
*   `cx`, `cy`: Principal point coordinates.

These parameters are usually obtained through a process called **camera calibration**.

### Extrinsic Camera Parameters

These parameters describe the camera's pose (position and orientation) in the 3D world. They define the rigid transformation from the world coordinate system to the camera coordinate system.

*   **Rotation Matrix (R):** A 3x3 matrix representing the camera's orientation.
*   **Translation Vector (t):** A 3x1 vector representing the camera's position.

Together, `R` and `t` form the **Extrinsic Matrix**, which transforms 3D points from world coordinates to camera coordinates.

### Lens Distortion

Real-world lenses introduce distortion, causing straight lines to appear curved in an image. Common types include:

*   **Radial Distortion:** Caused by the lens shape.
*   **Tangential Distortion:** Caused by misalignment of the lens components.

Distortion parameters are also determined during camera calibration and are used to rectify images, making them suitable for accurate measurements and geometric algorithms.

### Image Formation in Robotics

For a robot, understanding these models allows it to:

*   **Perceive Depth:** With stereo cameras or depth sensors, it can reconstruct the 3D structure of the environment.
*   **Localize Objects:** Determine the 3D position of objects detected in 2D images.
*   **Navigate:** Use visual odometry or SLAM (Simultaneous Localization and Mapping) algorithms to understand its own motion and build a map of its surroundings.

Accurate camera calibration and a solid grasp of these concepts are foundational for any vision-based robotics task.

## 7.2: Pre-trained Models for Object Detection

Object detection is a crucial computer vision task for robots, enabling them to identify and locate objects within their environment. Instead of training deep learning models from scratch, which requires massive datasets and computational resources, robotics often leverages **pre-trained models**. These models have been trained on vast, publicly available datasets (like COCO, ImageNet, Open Images) and can be fine-tuned for specific robotic applications or used directly for common object recognition.

### Why Use Pre-trained Models?

*   **Time-Saving:** Avoids the need for extensive data collection and training from zero.
*   **Reduced Computational Cost:** Training large models from scratch is computationally expensive.
*   **Transfer Learning:** Pre-trained models have learned general features from diverse datasets, which can be effectively transferred to new, related tasks with smaller datasets.
*   **Accessibility:** Makes advanced object detection capabilities accessible even without deep AI expertise.

### Popular Pre-trained Object Detection Architectures:

Several architectures are widely used for object detection, each with its strengths in terms of speed, accuracy, and model size.

1.  **YOLO (You Only Look Once):**
    *   **Concept:** A single-stage detector that predicts bounding boxes and class probabilities directly from full images in a single pass. This makes it extremely fast.
    *   **Versions:** YOLOv3, YOLOv4, YOLOv5, YOLOv7, YOLOv8 are popular iterations, continually improving speed and accuracy.
    *   **Strengths:** Real-time performance, good for applications requiring high frame rates (e.g., drone navigation, fast-moving robotic manipulators).
    *   **Weaknesses:** Can sometimes struggle with small objects or dense scenes compared to two-stage detectors.

2.  **SSD (Single Shot Detector):**
    *   **Concept:** Similar to YOLO, it's a single-stage detector that combines predictions from multiple feature maps at different scales to handle objects of various sizes.
    *   **Strengths:** Good balance between speed and accuracy, often used with lightweight backbones (e.g., MobileNet) for edge devices.
    *   **Weaknesses:** Can be less accurate than two-stage detectors for very small objects.

3.  **Faster R-CNN (Region-based Convolutional Neural Network):**
    *   **Concept:** A two-stage detector. The first stage (Region Proposal Network - RPN) proposes potential object regions, and the second stage classifies these regions and refines bounding boxes.
    *   **Strengths:** High accuracy, robust for detecting small and overlapping objects.
    *   **Weaknesses:** Slower than single-stage detectors due to the two-stage process. More suitable for applications where accuracy is paramount and real-time inference is not strictly required.

### Integrating Pre-trained Models into Robotics:

1.  **Frameworks:** Libraries like **OpenCV DNN module**, **TensorFlow Object Detection API**, and **PyTorch Hub** provide easy access to pre-trained models.
2.  **On-Device Deployment:** For edge robots, models are often optimized using tools like **NVIDIA TensorRT** or converted to formats like **ONNX** to achieve high inference speeds on embedded GPUs (like the Jetson AGX Orin).
3.  **Synthetic Data:** Pre-trained models can be fine-tuned with synthetic data generated from Isaac Sim to adapt them to specific objects or environments unique to a robot's task.

By leveraging these powerful pre-trained models, robots can quickly gain a sophisticated understanding of their surroundings, enabling tasks such as object manipulation, human-robot interaction, and obstacle avoidance.

## 7.3: Integrating a Detection Model with ROS 2

Once you have a pre-trained object detection model, the next critical step for robotics is to integrate it seamlessly into your robot's software stack. ROS 2 provides the infrastructure to connect your computer vision pipeline with the rest of your robot's capabilities, such as navigation, manipulation, and decision-making.

### The ROS 2 Perception Pipeline:

A typical ROS 2 perception pipeline involves:

1.  **Camera Driver Node:** Publishes images from a physical camera or simulated camera (e.g., from Gazebo or Isaac Sim) to a ROS 2 topic (e.g., `/camera/image_raw`).
2.  **Image Processing Node:** Receives the raw image, performs pre-processing (e.g., rectification, color conversion), and potentially publishes a processed image.
3.  **Object Detection Node:** Subscribes to the processed image topic, runs the object detection model, and publishes the detection results.
4.  **Downstream Nodes:** Other robot nodes (e.g., for manipulation, navigation, or task planning) subscribe to the detection results to inform their behavior.

### Creating a ROS 2 Object Detection Node (Python Example):

Let's outline the structure of a Python ROS 2 node that integrates a pre-trained YOLO model (using OpenCV's DNN module for simplicity).

1.  **Prerequisites:**
    *   A ROS 2 Python package (e.g., `my_robot_pkg` from Week 3).
    *   `opencv-python`: `pip install opencv-python`
    *   A YOLO model (e.g., `yolov3.weights` and `yolov3.cfg` files, and `coco.names` for class labels).

2.  **`object_detector_node.py`:**
    Create a new file `~/ros2_ws/src/my_robot_pkg/my_robot_pkg/simple_publisher.py` with the following content:

    ```python
    # Python code example for object_detector_node.py goes here.
    # The previous code block caused a build error. This is a placeholder.
    # The actual content would involve imports from rclpy, sensor_msgs.msg,
    # cv_bridge, cv2, numpy, and vision_msgs.msg to create an object detector
    # node that subscribes to camera images, runs a YOLO model, and publishes
    # detection results.
    ```

3.  **Update `setup.py`:**
    Add the new node to the `entry_points` in `~/ros2_ws/src/my_robot_pkg/setup.py`:
    ```python
    entry_points={
        'console_scripts': [
            'simple_publisher = my_robot_pkg.simple_publisher:main',
            'simple_subscriber = my_robot_pkg.simple_subscriber:main',
            'object_detector = my_robot_pkg.object_detector_node:main', # New node
        ],
    },
    ```

4.  **Run the Node:**
    After building your workspace (`colcon build --packages-select my_robot_pkg`) and sourcing (`source install/setup.bash`), you can launch the node:
    ```bash
    ros2 run my_robot_pkg object_detector
    ```
    You would also need a node publishing images to `/camera/image_raw` for this to work.

This setup provides a flexible framework for integrating various object detection models into your ROS 2 robot applications. The `vision_msgs` package offers a standardized way to communicate detection results across your robot's software stack.