---
sidebar_position: 4
title: Chapter 3 - Cognitive Planning with LLMs and ROS 2
---

# Chapter 3: Cognitive Planning with LLMs and ROS 2

## Using LLMs for Goal-Based Planning

Large Language Models (LLMs) serve as powerful cognitive planners in robotic systems, bridging the gap between high-level human goals expressed in natural language and low-level robot actions executed through ROS 2. Unlike traditional rule-based planning systems, LLMs bring general knowledge, reasoning capabilities, and flexibility to robot planning tasks.

### The Planning Role of LLMs

LLMs function as cognitive planners by:

- **Goal Interpretation**: Understanding natural language goals and intentions
- **Knowledge Integration**: Leveraging world knowledge to inform planning decisions
- **Reasoning**: Applying logical reasoning to decompose goals into executable steps
- **Adaptation**: Adjusting plans based on environmental conditions and constraints
- **Communication**: Explaining planning decisions and requesting clarification when needed

### Advantages of LLM-Based Planning

#### Flexibility and Generalization
LLMs can handle novel situations and goals they weren't explicitly programmed for, using their general knowledge to reason about appropriate actions.

#### Natural Language Interface
LLMs provide a natural interface between human goals and robot actions, eliminating the need for users to learn robot-specific command languages.

#### Context Awareness
LLMs can incorporate contextual information to make more informed planning decisions, considering the current situation and past experiences.

#### Explainability
LLMs can provide explanations for their planning decisions, improving transparency and trust in robotic systems.

## Translating Natural Language into ROS 2 Action Sequences

### The Translation Process

Converting natural language to ROS 2 action sequences involves several key steps:

```
Natural Language Goal: "Go to the kitchen and bring me a glass of water"
                    ↓
┌─────────────────────────────────┐
│  Semantic Parsing               │
│  - Identify goal structure      │
│  - Extract entities             │
│  - Determine action sequence    │
└─────────────────────────────────┘
                    ↓
┌─────────────────────────────────┐
│  Action Decomposition           │
│  - Navigate → Kitchen           │
│  - Locate → Glass              │
│  - Locate → Water source       │
│  - Grasp → Glass               │
│  - Fill → Glass with water     │
│  - Navigate → User             │
└─────────────────────────────────┘
                    ↓
┌─────────────────────────────────┐
│  ROS 2 Action Mapping           │
│  - move_base_msgs/MoveBaseGoal  │
│  - object_detection_msgs/Find   │
│  - manipulation_msgs/Grasp      │
│  - navigation_msgs/Pour         │
└─────────────────────────────────┘
                    ↓
ROS 2 Action Sequence
```

### Mapping Strategies

#### Template-Based Mapping
Using predefined templates to map common phrases to ROS 2 actions. For example, "go to X" maps to navigation goals, "pick up Y" maps to manipulation actions.

#### Semantic Frame Mapping
Identifying semantic roles (agent, action, patient, location) and mapping them to appropriate ROS 2 action parameters.

#### Knowledge Graph Integration
Using structured knowledge about objects, locations, and actions to inform the mapping process.

### Handling Ambiguity

Natural language often contains ambiguity that must be resolved during translation:

- **Referential Ambiguity**: "Bring me that" - which object is "that"?
- **Spatial Ambiguity**: "Go over there" - what location does "there" refer to?
- **Action Ambiguity**: "Get the object" - how should the robot interpret this?

Resolution strategies include:
- Using perceptual information to identify referenced objects
- Employing spatial reasoning based on deictic references
- Applying default assumptions based on context
- Requesting clarification when needed

## The Difference Between High-Level Task Planning and Low-Level Execution

### High-Level Task Planning

High-level task planning focuses on strategic decision-making and goal decomposition:

#### Strategic Reasoning
- **Goal Decomposition**: Breaking complex goals into manageable subtasks
- **Resource Allocation**: Determining how to best use available capabilities
- **Temporal Planning**: Sequencing actions over time to achieve goals
- **Contingency Planning**: Preparing alternative approaches for different scenarios

#### Knowledge Utilization
- **World Knowledge**: Using general knowledge to inform planning decisions
- **Domain Knowledge**: Applying robotics-specific knowledge to planning
- **Experience Integration**: Incorporating past experiences and learned patterns

#### Abstract Representation
- Working with abstract representations of the world and goals
- Reasoning about high-level concepts rather than specific sensor values
- Focusing on what needs to be achieved rather than how to achieve it

### Low-Level Execution

Low-level execution focuses on precise control and real-time adaptation:

#### Precise Control
- **Motor Control**: Directing specific robot actuators and movements
- **Sensor Processing**: Processing real-time sensor data for immediate decisions
- **Timing Control**: Managing precise timing for coordinated actions

#### Real-Time Adaptation
- **Feedback Control**: Adjusting actions based on real-time sensor feedback
- **Error Recovery**: Handling unexpected situations and disturbances
- **Safety Management**: Maintaining safety in real-time operation

#### Concrete Implementation
- Working with specific sensor values and actuator commands
- Executing precise movements and manipulations
- Focusing on how actions are performed rather than what needs to be achieved

### The Planning-Execution Interface

The interface between high-level planning and low-level execution is critical for successful robot operation:

```
┌─────────────────────────┐    ┌─────────────────────────┐
│   High-Level Planner    │    │   Low-Level Executor    │
│    (LLM-based)          │ -> │    (ROS 2 Actions)      │
├─────────────────────────┤    ├─────────────────────────┤
│ • Goal: "Go to kitchen" │ -> │ • MoveBaseGoal:         │
│ • Abstract location:    │ -> │   x=5.2, y=3.1, theta=0 │
│   "kitchen"             │ -> │ • Action: move_base     │
│ • Sequence: Navigate    │ -> │ • Parameters: speed,    │
│                        │ -> │   safety_thresholds     │
└─────────────────────────┘    └─────────────────────────┘
```

## Coordinating Perception, Navigation, and Manipulation Systems

### System Architecture for Coordination

Successful cognitive planning requires tight coordination between perception, navigation, and manipulation systems:

#### Perception-Navigation Coordination
- **Environmental Mapping**: Using perception data to build and update navigation maps
- **Obstacle Detection**: Identifying dynamic obstacles that affect navigation plans
- **Landmark Recognition**: Using visual landmarks for navigation assistance
- **Terrain Analysis**: Assessing ground conditions for safe navigation

#### Perception-Manipulation Coordination
- **Object Recognition**: Identifying objects for manipulation tasks
- **Pose Estimation**: Determining object poses for grasp planning
- **Surface Analysis**: Assessing surfaces for placement and support
- **Material Properties**: Identifying object properties that affect manipulation

#### Navigation-Manipulation Coordination
- **Approach Planning**: Navigating to appropriate locations for manipulation
- **Workspace Access**: Positioning the robot for effective manipulation
- **Path Planning**: Coordinating navigation and manipulation paths
- **Hand-Eye Coordination**: Coordinating manipulator positioning with navigation

### Coordination Mechanisms

#### ROS 2 Action Servers
Using ROS 2 action servers to coordinate complex tasks that involve multiple subsystems:

- **Navigation Actions**: `move_base_msgs/MoveBaseAction`
- **Manipulation Actions**: `control_msgs/GripperCommandAction`
- **Perception Actions**: `object_recognition_msgs/ObjectRecognitionAction`

#### Service Calls
Using ROS 2 services for synchronous coordination when immediate responses are needed:

- **Object Location Queries**: Requesting current object positions
- **Capability Checks**: Verifying system availability before task execution
- **Status Requests**: Getting system status for planning decisions

#### Message Passing
Using ROS 2 topics for asynchronous coordination and state sharing:

- **Sensor Data**: Sharing perception data across systems
- **Transforms**: Sharing coordinate frame transformations
- **Status Updates**: Broadcasting system status changes

### Example Coordination Scenario

Consider the task "Bring me the red cup from the table":

1. **Perception Phase**:
   - Object recognition identifies "red cup" on "table"
   - Pose estimation determines cup location relative to robot

2. **Planning Phase**:
   - LLM decomposes task into navigation + manipulation subtasks
   - Navigation plan computed to reach table
   - Manipulation plan computed to grasp cup

3. **Execution Phase**:
   - Navigation system executes approach to table
   - Perception system monitors approach progress
   - Manipulation system executes grasp action
   - Navigation system executes return to user

4. **Coordination Points**:
   - Navigation pauses for manipulation to complete
   - Manipulation adjusts based on perception feedback
   - Systems coordinate to ensure safe execution

## Preparing for the Autonomous Humanoid Capstone

### Integration with Previous Learning

The VLA approach synthesizes concepts from previous modules:

- **ROS 2 Foundation**: Leverages ROS 2 communication patterns and action structures
- **Perception Understanding**: Integrates with Isaac perception concepts for object understanding
- **Simulation Experience**: Builds on simulation concepts for safe planning and testing

### Cognitive Architecture Patterns

The cognitive planning approach establishes patterns that extend to autonomous humanoid systems:

#### Hierarchical Planning
- High-level goals in natural language
- Mid-level task decomposition
- Low-level action execution

#### Multi-Modal Integration
- Vision processing for environmental understanding
- Language processing for goal interpretation
- Action execution for physical interaction

#### Adaptive Behavior
- Learning from experience to improve future planning
- Adapting to new situations and environments
- Handling uncertainty and ambiguity gracefully

### Capstone Preparation

This module prepares you for the autonomous humanoid capstone by:

- Establishing cognitive planning principles that scale to complex behaviors
- Providing experience with multi-system coordination challenges
- Developing understanding of natural language interfaces for autonomous systems
- Building foundation for advanced perception-action integration

<details>
<summary>Advanced: LLM-ROS 2 Integration Patterns</summary>

Several architectural patterns are commonly used for LLM-ROS 2 integration:

- **Planner-Controller Pattern**: LLM acts as high-level planner, ROS 2 systems as low-level controllers
- **Behavior Tree Integration**: LLM generates behavior tree structures for ROS 2 execution
- **Reactive Planning**: LLM continuously replans based on ROS 2 system feedback
- **Hierarchical Task Networks**: LLM decomposes tasks into ROS 2-executable subnetworks

Each pattern has trade-offs in terms of flexibility, efficiency, and robustness that should be considered for specific applications.

</details>

In this chapter, you've learned about cognitive planning with LLMs and ROS 2. You now understand how LLMs serve as goal-based planners, how natural language is translated into ROS 2 action sequences, the difference between high-level task planning and low-level execution, and how perception, navigation, and manipulation systems are coordinated. You've also learned how this prepares you for the autonomous humanoid capstone project.

This completes the Vision-Language-Action module, providing you with a comprehensive understanding of how language models can serve as cognitive planners for robotic systems, bridging natural language goals with physical robot actions through coordinated perception, planning, and execution systems.