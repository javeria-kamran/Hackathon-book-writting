# Week 12: Integrating Voice Commands and NLP

## 12.1: Speech-to-Text and Intent Recognition

Enabling a robot to understand spoken commands is a significant step towards more natural and intuitive human-robot interaction. This involves two primary stages: **Speech-to-Text (STT)**, which converts spoken audio into written text, and **Intent Recognition**, which extracts the user's underlying goal or purpose from that text.

### Speech-to-Text (STT): Converting Audio to Text

STT systems, also known as Automatic Speech Recognition (ASR), translate human speech into transcribed text. Modern STT systems are predominantly built using deep learning models.

#### Key Aspects:

*   **Acoustic Model:** Maps audio signals (phonemes) to phonetic representations.
*   **Pronunciation Model (Lexicon):** Maps phonetic sequences to words.
*   **Language Model:** Predicts the likelihood of word sequences, helping to disambiguate similar-sounding words (e.g., "recognize speech" vs. "wreck a nice beach").

#### Options for Robotics:

1.  **Cloud-based APIs:**
    *   **Examples:** Google Cloud Speech-to-Text, AWS Transcribe, Microsoft Azure Speech Service.
    *   **Pros:** High accuracy, supports many languages, handles various accents, powerful models.
    *   **Cons:** Requires internet connectivity, latency can be an issue, privacy concerns for sensitive data, cost implications.
    *   **Integration:** Typically involves sending audio snippets (or a stream) to the API and receiving JSON responses.

2.  **On-device / Edge STT:**
    *   **Examples:** NVIDIA Riva (for Jetson/GPU), Vosk (open-source), Kaldi (open-source).
    *   **Pros:** Low latency, works offline, enhanced privacy, no recurring costs.
    *   **Cons:** Requires more computational resources on the edge device, potentially lower accuracy than cloud solutions, models are generally specialized.
    *   **Integration:** Often provided as SDKs or libraries that can be directly integrated into ROS 2 nodes.

For robotics, especially in applications where real-time response and offline capability are crucial (like an autonomous humanoid in varied environments), on-device STT solutions like NVIDIA Riva are often preferred.

### Intent Recognition: Understanding the User's Goal

Once speech is converted to text, the next step is to understand what the user *wants* the robot to do. **Intent recognition**, a sub-field of Natural Language Understanding (NLU), analyzes the transcribed text to identify the user's underlying intention and extract relevant entities.

#### Key Concepts:

*   **Intents:** Represent the high-level goals or commands (e.g., `Navigate`, `PickAndPlace`, `ReportStatus`).
*   **Entities (Slots):** Specific pieces of information required to fulfill the intent (e.g., for a `Navigate` intent, entities might be `location`="kitchen", `speed`="slow").

#### Options for Intent Recognition:

1.  **Rule-based Systems:**
    *   **Concept:** Uses predefined patterns, keywords, and grammatical rules to match intents and extract entities.
    *   **Pros:** Easy to implement for simple, constrained domains; high precision for matched rules.
    *   **Cons:** Brittle, doesn't generalize well, difficult to maintain as complexity grows.

2.  **Machine Learning Models (NLU Frameworks):**
    *   **Examples:** Rasa NLU, Microsoft LUIS, Google Dialogflow.
    *   **Concept:** Trains models (often using neural networks) on example utterances to learn how to classify intents and extract entities.
    *   **Pros:** Generalizes well, handles natural language variations, can learn from data, more scalable.
    *   **Cons:** Requires training data, performance depends on model quality and data, can be resource-intensive.
    *   **Integration:** Typically exposed as APIs that a ROS 2 node can query.

### ROS 2 Integration for Voice Commands:

A common ROS 2 workflow for voice command processing involves:

1.  **Audio Capture Node:** Captures audio from a microphone and publishes it as a ROS 2 audio message.
2.  **STT Node:** Subscribes to the audio message, performs speech-to-text conversion (using either a cloud API or an on-device engine), and publishes the transcribed text as a `std_msgs/msg/String` message.
3.  **NLU Node:** Subscribes to the text message, performs intent recognition, and publishes structured intent messages (e.g., a custom ROS 2 message defining `intent_name` and `entity_values`).
4.  **Behavior Manager Node:** Subscribes to the intent messages and triggers the appropriate robot behaviors (e.g., sends a `NavigateToPose` action goal if a `Navigate` intent is detected).

This layered approach allows for robust voice command processing, transforming human speech into actionable commands for autonomous robots.

## 12.2: Using LLMs for Task Planning

While traditional task planners in robotics rely on symbolic representations and hand-engineered rules, the advent of Large Language Models (LLMs) offers a powerful new paradigm for high-level robot task planning. LLMs can interpret natural language instructions, break down complex goals into executable sub-tasks, and even adapt plans in response to unexpected events.

### LLMs as High-Level Planners:

Instead of defining every possible state and transition, LLMs can leverage their vast knowledge encoded during pre-training to:

*   **Interpret Ambiguous Instructions:** Translate human-centric commands (e.g., "clean the table") into robot-centric actions.
*   **Decompose Complex Goals:** Break down a high-level task into a sequence of smaller, achievable sub-tasks that can be mapped to robot primitives (e.g., "clean the table" -> "find sponge", "grasp sponge", "wipe surface", "put sponge away").
*   **Generate Action Sequences:** Output a sequence of robot actions that can be executed by the robot's lower-level control systems (e.g., calling ROS 2 actions like `navigate_to_pose`, `pick_object`).
*   **Handle Constraints:** Understand and incorporate various constraints (e.g., "don't spill the water," "avoid moving objects").
*   **Adapt and Replanning:** Modify plans in real-time based on sensory feedback or user interventions.

### Integrating LLMs into a Robotics Stack:

A common approach to integrate LLMs involves:

1.  **User Input:** The robot receives a high-level natural language command (e.g., via Speech-to-Text).
2.  **LLM Prompt Engineering:** The natural language command, along with the robot's current state, available tools (robot capabilities/functions), and environmental context, are formatted into a prompt for the LLM.
    *   **Available Tools:** The LLM is "told" what actions the robot can perform (e.g., `navigate(location)`, `pick(object_name)`, `say(phrase)`). This is akin to providing function signatures.
    *   **Current State:** Information about the robot's location, known objects, and status.
    *   **Memory/Context:** Past interactions, knowledge about the environment.
3.  **LLM Inference:** The LLM processes the prompt and generates a plan. The plan could be:
    *   A sequence of robot actions directly executable by lower-level controllers.
    *   A Behavior Tree XML string.
    *   A Python script.
    *   A series of sub-goals.
4.  **Execution Module:** A dedicated module (often a ROS 2 node) parses the LLM's output and translates it into executable commands for the robot's control system. This module also handles feedback from the robot to update the LLM's context or trigger replanning.

#### Challenges:

*   **Grounding:** Ensuring the LLM's generated actions correspond to real-world robot capabilities and the physical environment.
*   **Safety:** LLMs can sometimes generate unsafe or nonsensical plans. Robust validation and arbitration layers are necessary.
*   **Computational Cost:** Running large LLMs on-device for real-time planning can be computationally expensive. Cloud inference or smaller, specialized models may be required.
*   **Long-Term Planning:** LLMs excel at short-term goal decomposition but may struggle with very long-horizon, complex tasks without additional planning mechanisms.

Despite these challenges, LLMs represent a transformative technology for robotics, offering unprecedented flexibility and generalization capabilities for high-level task planning in complex, human-centric environments. This integration moves robots closer to truly understanding and fulfilling human intentions.

## 12.3: Creating a Voice-Controlled Robot Interface

Combining Speech-to-Text (STT) for understanding natural language commands and Large Language Models (LLMs) for task planning enables the creation of powerful **voice-controlled robot interfaces**. Such interfaces allow users to interact with robots in a highly intuitive and human-like manner, moving beyond traditional joystick or GUI controls.

### Architecture of a Voice-Controlled Robot Interface:

The interface typically involves a chain of ROS 2 nodes (or similar modular components) to process voice commands and translate them into robot actions:

1.  **Audio Input Node:**
    *   **Function:** Captures audio from a microphone (either on the robot or a remote device).
    *   **Output:** Publishes raw audio data to a ROS 2 topic.

2.  **Speech-to-Text (STT) Node (Week 12.1):**
    *   **Function:** Subscribes to the audio data, converts it into text using an STT engine (cloud-based or on-device like NVIDIA Riva).
    *   **Output:** Publishes the transcribed text to a ROS 2 topic.

3.  **Natural Language Understanding (NLU) / LLM Orchestrator Node (Week 12.2):**
    *   **Function:** Subscribes to the transcribed text.
        *   For simpler systems, an NLU component extracts intent and entities.
        *   For more advanced systems, an LLM acts as an orchestrator, receiving the text, current robot state, and a list of available robot "tools" (ROS 2 actions, services).
    *   **Output:** Publishes a structured command or a sequence of executable robot actions. This might be a custom ROS 2 message (e.g., `RobotCommand.msg`) or a series of calls to standard ROS 2 Action Clients.

4.  **Behavior Execution Node (Behavior Tree Manager):**
    *   **Function:** Subscribes to the structured commands from the NLU/LLM Orchestrator.
    *   If the LLM outputs a sequence of actions, this node would trigger the corresponding ROS 2 Action Clients (e.g., for navigation, manipulation).
    *   If a Behavior Tree is used for high-level control, this node would "tick" the BT, which in turn would activate the appropriate sub-trees or actions.

5.  **Robot Control Nodes:**
    *   These are the low-level controllers (e.g., `ros2_control` for arm joints, Nav2 for mobile base) that receive and execute commands from the Behavior Execution Node, translating them into physical motion.

### Implementing Basic Voice Control with ROS 2:

*   **Microphone Setup:** Ensure your microphone is correctly recognized by your Ubuntu system. `arecord -l` can list capture devices.
*   **ROS 2 Audio Bridge:** You might use or create a ROS 2 node that captures audio from the microphone and publishes it as `audio_common_msgs/msg/AudioData`.
*   **STT Integration:** If using a cloud API, your STT node would send audio packets to the cloud service and publish the returned text. If using an on-device solution like Riva, you would integrate its SDK.
*   **NLU/LLM Integration:**
    *   For basic commands: Map keywords to pre-defined ROS 2 actions (e.g., "go home" triggers a `NavigateToPose` action to a predefined "home" pose).
    *   For advanced commands: Your LLM orchestrator would consume the text and generate a sequence of calls to your robot's existing ROS 2 actions/services.

### Considerations for Real-World Deployment:

*   **Noise Robustness:** Real-world environments are noisy. Your STT system needs to be robust to background noise.
*   **Speaker Independence:** The system should ideally work for different speakers.
*   **Natural Language Understanding:** The NLU/LLM component needs to be sophisticated enough to handle variations in phrasing, slang, and context.
*   **Error Recovery:** How does the robot clarify ambiguous commands or recover from misinterpretations?
*   **Safety:** Critical commands should have confirmation mechanisms or physical overrides.
*   **Context Management:** The robot needs a "memory" of past interactions and environmental state to understand follow-up commands and perform coherent actions.

A voice-controlled interface fundamentally changes how humans can interact with complex robotic systems, making them more accessible and user-friendly for a wider range of applications.