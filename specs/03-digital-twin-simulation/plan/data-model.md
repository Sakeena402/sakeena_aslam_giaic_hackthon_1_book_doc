# Data Model: Digital Twin Module

**Feature**: 03-digital-twin-simulation
**Date**: 2025-12-25

## Module Structure

### Entity: DigitalTwinModule
- **name**: "Module 02 — The Digital Twin (Gazebo & Unity)"
- **description**: Introduction to Digital Twin concepts for humanoid robots
- **chapters**: [Chapter1, Chapter2, Chapter3]
- **learningObjectives**: List of learning outcomes from spec
- **targetAudience**: ["AI students", "Robotics developers", "Simulation engineers"]
- **navigation**: Sidebar configuration for module

### Entity: Chapter
- **title**: String (chapter title)
- **content**: Markdown content
- **diagrams**: List of Mermaid or SVG diagrams
- **codeSnippets**: List of configuration examples
- **learningObjectives**: Specific objectives for this chapter
- **prerequisites**: Prerequisites from previous chapters
- **nextChapter**: Reference to subsequent chapter

#### Chapter 1: DigitalTwinConceptsChapter
- **title**: "Digital Twins in Robotics"
- **content**: Definition, purpose, simulation importance, Gazebo/Unity overview
- **diagrams**: Digital Twin architecture diagram, simulation flow diagram
- **codeSnippets**: None (conceptual chapter)
- **learningObjectives**: Explain Digital Twin definition, importance of simulation, differences between real/simulated robots

#### Chapter 2: PhysicsSimulationChapter
- **title**: "Physics Simulation with Gazebo"
- **content**: Gravity, collisions, friction, joints, environment building
- **diagrams**: Physics simulation diagram, environment building illustration
- **codeSnippets**: Gazebo configuration examples
- **learningObjectives**: Understand physics simulation concepts, environment building, ROS 2 integration

#### Chapter 3: SensorSimulationChapter
- **title**: "Sensors and Interaction in Simulation"
- **content**: LiDAR, depth cameras, IMUs, sensor data flow, Unity visualization
- **diagrams**: Sensor simulation diagram, data flow illustration
- **codeSnippets**: Sensor configuration examples
- **learningObjectives**: Understand sensor simulation, data flow to ROS 2, Unity's role

## Content Elements

### Entity: Diagram
- **type**: "mermaid" | "svg" | "image"
- **title**: Description of the diagram
- **content**: The actual diagram code or file reference
- **placement**: Where the diagram appears in content
- **purpose**: What concept the diagram illustrates

### Entity: CodeSnippet
- **language**: "xml" | "yaml" | "bash" | "python" | etc.
- **title**: Brief description of the code purpose
- **content**: The actual code/configuration
- **explanation**: Text explaining the code
- **context**: Where this code would be used

### Entity: CollapsibleSection
- **title**: Title for the collapsible section
- **summary**: Brief summary shown when collapsed
- **content**: Detailed content shown when expanded
- **type**: "advanced-note" | "technical-detail" | "example"
- **placement**: Where in the document this appears

## Navigation Structure

### Entity: SidebarItem
- **type**: "category" | "doc"
- **label**: Display name for the navigation item
- **items**: Children for category type
- **id**: Document ID for doc type
- **link**: Navigation link
- **priority**: Order in navigation

### Entity: ModuleSidebar
- **id**: "digital-twin-module"
- **items**: [SidebarItem for index, Chapter1, Chapter2, Chapter3]
- **configuration**: Navigation configuration for the Digital Twin module
- **integration**: How this module connects to main navigation