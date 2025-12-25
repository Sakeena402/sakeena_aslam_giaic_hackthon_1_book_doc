# Data Model: ROS 2 Module

**Feature**: 01-ros2-nervous-system
**Date**: 2025-12-25

## Module Structure

### Entity: ROS2Module
- **name**: "ROS 2 as the Robotic Nervous System"
- **description**: Introduction to ROS 2 as the core nervous system of humanoid robots
- **chapters**: [Chapter1, Chapter2, Chapter3]
- **learningObjectives**: List of learning outcomes from spec
- **targetAudience**: ["Computer science students", "AI practitioners", "Robotics beginners"]
- **navigation**: Sidebar configuration for module

### Entity: Chapter
- **title**: String (chapter title)
- **content**: Markdown content
- **diagrams**: List of Mermaid or SVG diagrams
- **codeSnippets**: List of code examples
- **learningObjectives**: Specific objectives for this chapter
- **prerequisites**: Prerequisites from previous chapters
- **nextChapter**: Reference to subsequent chapter

#### Chapter 1: ROS2ArchitectureChapter
- **title**: "ROS 2 as the Robotic Nervous System"
- **content**: Conceptual overview, nervous system analogy, nodes/topics/services
- **diagrams**: ROS 2 architecture diagram, message passing illustration
- **codeSnippets**: None (conceptual chapter)
- **learningObjectives**: Explain ROS 2 architecture, identify components, understand message passing

#### Chapter 2: PythonCommunicationChapter
- **title**: "Python Agents and ROS 2 Communication"
- **content**: rclpy usage, node creation, pub/sub, services
- **diagrams**: Node communication diagram, publisher-subscriber pattern
- **codeSnippets**: Python code examples using rclpy
- **learningObjectives**: Create ROS 2 nodes, publish/subscribe to topics, call/expose services

#### Chapter 3: URDFChapter
- **title**: "Humanoid Robot Description with URDF"
- **content**: URDF purpose, links/joints/frames, kinematics, simulation
- **diagrams**: Robot structure diagram, coordinate frames illustration
- **codeSnippets**: URDF XML examples
- **learningObjectives**: Understand URDF, read robot descriptions, prepare for simulation

## Content Elements

### Entity: Diagram
- **type**: "mermaid" | "svg" | "image"
- **title**: Description of the diagram
- **content**: The actual diagram code or file reference
- **placement**: Where the diagram appears in content
- **purpose**: What concept the diagram illustrates

### Entity: CodeSnippet
- **language**: "python" | "xml" | "bash" | etc.
- **title**: Brief description of the code purpose
- **content**: The actual code
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
- **id**: "ros2-module"
- **items**: [SidebarItem for index, Chapter1, Chapter2, Chapter3]
- **configuration**: Manual sidebar configuration for the ROS2 module
- **integration**: How this sidebar connects to main navigation