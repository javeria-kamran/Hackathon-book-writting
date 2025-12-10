# Week 11: Behavior Trees and State Machines

## 11.1: Designing Complex Robot Behaviors

As robots become more autonomous and are tasked with increasingly complex missions in dynamic environments, simply chaining together basic actions becomes insufficient. Designing how a robot should behave, especially when faced with uncertainty, failures, or multiple competing goals, requires structured and robust approaches. This section introduces two primary paradigms for designing complex robot behaviors: Finite State Machines and Behavior Trees.

### Challenges in Robot Behavior Design:

*   **Reactivity vs. Deliberation:** How does a robot balance quick responses to immediate threats (reactivity) with long-term planning (deliberation)?
*   **Concurrency:** How does a robot manage multiple tasks happening simultaneously (e.g., navigating while monitoring sensors)?
*   **Error Handling:** How does a robot gracefully recover from unexpected events or failures?
*   **Modularity and Reusability:** How can behavior components be designed to be easily reused and combined?
*   **Scalability:** How can behaviors be managed as the robot's capabilities and task complexity grow?

### Finite State Machines (FSMs)

A **Finite State Machine (FSM)** is a mathematical model of computation that describes the behavior of a system based on its current state and a set of transitions between states. At any given time, the system is in one of a finite number of states.

#### Key Concepts:

*   **States:** Represent distinct modes of operation or situations (e.g., `Idle`, `Navigating`, `Grasping`, `Error`).
*   **Transitions:** Rules that specify how the system moves from one state to another, triggered by events or conditions (e.g., "goal reached," "obstacle detected," "gripper closed").
*   **Events/Conditions:** Inputs or internal changes that cause a state transition.
*   **Actions:** Tasks performed when entering a state, exiting a state, or during a state.

#### Advantages:

*   **Simplicity:** Easy to understand and implement for simple, sequential behaviors.
*   **Determinism:** Predictable behavior for a given set of inputs.
*   **Well-defined states:** Clear operational modes.

#### Disadvantages:

*   **State Explosion:** Can become very complex and difficult to manage as the number of states and transitions grows.
*   **Lack of Concurrency:** Difficult to represent parallel actions or asynchronous events directly.
*   **Reusability Issues:** States and transitions are often tightly coupled, making components hard to reuse.

#### Example: Simple Navigation FSM

*   **States:** `Idle`, `PlanningPath`, `ExecutingPath`, `ObstacleDetected`, `GoalReached`, `Error`.
*   **Transitions:**
    *   `Idle` -> `PlanningPath` (on "new goal received" event)
    *   `ExecutingPath` -> `ObstacleDetected` (on "collision warning" event)
    *   `ObstacleDetected` -> `PlanningPath` (on "path re-planned" event)
    *   `ExecutingPath` -> `GoalReached` (on "goal reached" event)

While FSMs are powerful for simpler, sequential control, their limitations in handling complex, concurrent, and fault-tolerant behaviors have led to the exploration of other paradigms, such as Behavior Trees.

## 11.2: Implementing Behavior Trees with BehaviorTree.CPP

**Behavior Trees (BTs)** are a powerful, modular, and graphical formalism for designing complex agent behaviors. Originating in the game AI industry, they have found significant adoption in robotics, notably becoming a core component of the Nav2 stack (Week 9). BTs offer a more flexible and scalable alternative to traditional Finite State Machines (FSMs) for managing complex robot decision-making.

### Fundamental Concepts of Behavior Trees:

A Behavior Tree is a directed acyclic graph (DAG) composed of different types of nodes:

*   **Root Node:** The starting point of the tree.
*   **Control Flow Nodes:** Define how children nodes are executed.
    *   **Sequences (`->`):** Execute children from left to right. If a child succeeds, it moves to the next. If a child fails or is running, the sequence does the same. Succeeds if all children succeed.
    *   **Fallbacks (`?` or `->` with a question mark):** Execute children from left to right. If a child succeeds or is running, the fallback does the same. Succeeds if any child succeeds. Fails if all children fail.
    *   **Parallels (`=>`):** Execute all children simultaneously. They can be configured to succeed if a certain number of children succeed or fail.
*   **Decorator Nodes:** Modify the return status or execution of a single child node (e.g., `Inverter`, `Retry`, `Timeout`, `ForceSuccess/Failure`).
*   **Leaf Nodes:** These are the actual "actions" the robot performs or "conditions" it checks. They do not have children.
    *   **Action Nodes:** Execute a task (e.g., `NavigateToGoal`, `PickObject`). They return `SUCCESS`, `FAILURE`, or `RUNNING`.
    *   **Condition Nodes:** Check a condition (e.g., `IsBatteryLow`, `IsObjectDetected`). They return `SUCCESS` (condition met) or `FAILURE` (condition not met).

### Return Status:

Every node in a Behavior Tree returns one of three statuses:
*   **`SUCCESS`:** The node's execution completed successfully.
*   **`FAILURE`:** The node's execution failed.
*   **`RUNNING`:** The node is currently executing and needs more time.

### Advantages over FSMs:

*   **Modularity:** Nodes are highly decoupled and reusable.
*   **Reactive and Deliberative:** Easily combine reactive behaviors (e.g., "avoid obstacle immediately") with deliberative planning (e.g., "plan a long-term path").
*   **Error Handling:** Natural way to handle failures and implement recovery behaviors (e.g., "if navigation fails, retry, then replan").
*   **Scalability:** Trees can grow very large without the "state explosion" problem of FSMs.
*   **Readability:** The graphical representation is often intuitive and easy to understand.

### BehaviorTree.CPP: A C++ Library for Behavior Trees

**BehaviorTree.CPP** is a popular, open-source C++ library for creating and executing Behavior Trees. It's widely used in ROS 2 (especially Nav2) and provides:

*   **XML Definition:** BTs can be defined in XML, allowing for easy serialization, visualization, and modification without recompiling code.
*   **Node Registration:** Custom C++ or Python functions can be registered as `Action` or `Condition` nodes, allowing you to connect your BT to your robot's existing ROS 2 functionalities.
*   **Tick-based Execution:** The tree is "ticked" periodically (e.g., at 10 Hz), and nodes are executed based on their return status.
*   **Blackboard:** A shared memory system (key-value store) that allows nodes to share data and communicate with each other.

#### Basic BehaviorTree.CPP Structure (Conceptual):

1.  **Define Custom Nodes:** Implement your robot-specific actions and conditions as C++ classes inheriting from `BT::ActionNodeBase` or `BT::ConditionNodeBase`.
    ```cpp
    // Example Action Node
    class NavigateToGoal : public BT::SyncActionNode
    {
    public:
        NavigateToGoal(const std::string& name, const BT::NodeConfig& config)
          : BT::SyncActionNode(name, config) {}

        BT::NodeStatus tick() override {
            // Logic to send a navigation goal
            // ...
            std::cout << "Navigating to goal!" << std::endl;
            return BT::NodeStatus::SUCCESS; // Or FAILURE, or RUNNING if async
        }
    };
    ```

2.  **Register Nodes:** In your main program, register your custom nodes with the BT factory.
    ```cpp
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<NavigateToGoal>("NavigateToGoal");
    ```

3.  **Load and Execute Tree:** Load your BT from an XML file and tick it.
    ```cpp
    std::string xml_tree = R"(
        <root BTCPP_format="4">
            <BehaviorTree>
                <Sequence>
                    <NavigateToGoal/>
                    <SayPhrase phrase="Goal reached!"/>
                </Sequence>
            </BehaviorTree>
        </root>
    )";
    auto tree = factory.createTreeFromText(xml_tree);
    tree.tickWhileRunning();
    ```

Behavior Trees provide a highly visual, modular, and powerful way to structure complex robot behaviors, making them an indispensable tool for agentic robotics development.

## 11.3: Integrating Behavior Trees with ROS 2 Actions

While BehaviorTree.CPP provides the core logic for defining and executing Behavior Trees, for a robot to actually *do* anything, these trees need to interface with the robot's software stack. In ROS 2, **Actions** (as discussed in Week 3) are the ideal communication pattern for integrating Behavior Tree nodes with long-running, goal-oriented robot capabilities.

### Why ROS 2 Actions for BT Integration?

*   **Asynchronous Execution:** Actions are inherently asynchronous, perfectly matching the `RUNNING` state of a Behavior Tree Action node. The BT can send a goal, receive feedback, and doesn't block while the robot is executing the action.
*   **Goal-Feedback-Result:** This pattern allows the BT to monitor the progress of an action (feedback), make decisions based on that progress, and receive a final outcome (result), which can then be mapped to `SUCCESS` or `FAILURE` in the BT.
*   **Preemption/Cancellation:** Actions can be canceled, which is crucial for dynamic robot behaviors (e.g., canceling a navigation goal if a higher-priority task emerges). The BT can send a cancellation request to the Action Server.
*   **Standardized Interface:** Using ROS 2 Actions means your BT nodes can communicate with any ROS 2 Action Server, promoting modularity and reusability of robot capabilities.

### Building a ROS 2 Action BT Node (Conceptual):

To integrate a ROS 2 Action with a Behavior Tree, you typically create a custom Action node in BehaviorTree.CPP that acts as a client to a ROS 2 Action Server.

1.  **Define a Custom `ActionClient` Node:**
    This node will inherit from `BT::StatefulActionNode` (for asynchronous actions) and manage the interaction with the ROS 2 Action Client library.

    ```cpp
    #include <behaviortree_cpp/bt_factory.h>
    #include <rclcpp_action/rclcpp_action.hpp>
    #include <your_action_package/action/your_action.hpp> // Replace with your actual action message

    // This node will send a goal to a ROS 2 Action Server and wait for the result
    class SendGoalAction : public BT::StatefulActionNode
    {
    public:
        SendGoalAction(const std::string& name, const BT::NodeConfig& config)
            : BT::StatefulActionNode(name, config)
        {
            // Initialize ROS 2 Node and Action Client
            node_ = rclcpp::Node::make_shared("bt_action_client_node");
            action_client_ = rclcpp_action::create_client<your_action_package::action::YourAction>(
                node_,
                "your_action_server_name"
            );
            // Get goal from BT blackboard or input port
            // getInput("goal_value", goal_value_);
        }

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<std::string>("goal_value") };
        }

        // Method called when the node is ticked
        BT::NodeStatus onStart() override
        {
            if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
                RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
                return BT::NodeStatus::FAILURE;
            }

            auto goal_msg = your_action_package::action::YourAction::Goal();
            // Populate goal_msg using input ports

            auto send_goal_options = rclcpp_action::Client<your_action_package::action::YourAction>::SendGoalOptions();
            send_goal_options.feedback_callback =
                std::bind(&SendGoalAction::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            send_goal_options.result_callback =
                std::bind(&SendGoalAction::result_callback, this, std::placeholders::_1);

            future_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);
            return BT::NodeStatus::RUNNING;
        }

        // Method called if the node is in RUNNING state
        BT::NodeStatus onRunning() override
        {
            if (future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                // Goal handle received, now wait for result
                auto goal_handle = future_goal_handle_.get();
                if (!goal_handle) {
                    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
                    return BT::NodeStatus::FAILURE;
                }
                // Check if result is ready
                if (future_result_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                    // Result received
                    auto result = future_result_.get().result;
                    if (result->success) { // Assuming your action result has a 'success' field
                        return BT::NodeStatus::SUCCESS;
                    } else {
                        return BT::NodeStatus::FAILURE;
                    }
                }
            }
            rclcpp::spin_some(node_); // Process ROS 2 callbacks
            return BT::NodeStatus::RUNNING;
        }

        // Method called if the node is halted (e.g., parent sequence failed)
        void onHalted() override
        {
            if (future_goal_handle_.valid() &&
                future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                auto goal_handle = future_goal_handle_.get();
                if (goal_handle) {
                    // Send cancellation request
                    action_client_->async_cancel_goal(goal_handle);
                }
            }
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<your_action_package::action::YourAction>::SharedPtr action_client_;
        std::shared_future<rclcpp_action::Client<your_action_package::action::YourAction>::GoalHandle::SharedPtr> future_goal_handle_;
        std::shared_future<rclcpp_action::Client<your_action_package::action::YourAction>::Result::SharedPtr> future_result_;

        void feedback_callback(
            rclcpp_action::Client<your_action_package::action::YourAction>::GoalHandle::SharedPtr,
            const std::shared_ptr<const your_action_package::action::YourAction::Feedback> feedback)
        {
            // Process feedback (e.g., update blackboard)
            RCLCPP_INFO(node_->get_logger(), "Received feedback: %s", feedback->current_status.c_str());
        }

        void result_callback(const rclcpp_action::ClientGoalHandle<your_action_package::action::YourAction>::WrappedResult & result)
        {
            // Store result for onRunning to process
            future_result_ = std::make_shared<rclcpp_action::Client<your_action_package::action::YourAction>::Result::SharedPtr>(result.result);
        }
    };
    ```

### Integrating with Nav2 Behavior Trees:

Nav2 leverages this integration extensively. Actions like `NavigateToPose`, `FollowPath`, and `ComputePathToPose` are all implemented as Behavior Tree Action nodes that communicate with corresponding Nav2 Action Servers. You can extend Nav2's capabilities by writing your own custom Action Servers for specific robot tasks and then creating BT nodes to interact with them.

This robust integration of Behavior Trees and ROS 2 Actions provides a powerful framework for building complex, reactive, and intelligent robot behaviors that can handle the uncertainties of real-world operation.