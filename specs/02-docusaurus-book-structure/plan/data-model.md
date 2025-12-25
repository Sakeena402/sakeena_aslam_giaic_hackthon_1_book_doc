# Data Model: Docusaurus Book Structure

**Feature**: 02-docusaurus-book-structure
**Date**: 2025-12-25

## Book Structure

### Entity: Book
- **name**: "AI/Spec-Driven Book"
- **description**: The main educational book structure
- **introduction**: Introduction page explaining the book purpose and navigation
- **modules**: List of course modules
- **navigation**: Sidebar configuration for book structure

### Entity: BookIntroduction
- **title**: "Book Introduction"
- **content**: Overview of the book, its purpose, and navigation instructions
- **targetAudience**: Students, developers, and AI practitioners
- **navigation**: Link to first module or modules overview
- **layout**: Main entry point for the book structure

### Entity: Module
- **title**: Module title (e.g., "Module 01 - The Robotic Nervous System")
- **description**: Brief description of the module content
- **chapters**: List of chapters in the module
- **prerequisites**: Any prerequisites for the module
- **learningObjectives**: Learning objectives for the module
- **navigation**: Links to previous/next modules or chapters

#### Module 01: ROS2Module
- **title**: "Module 01 - The Robotic Nervous System"
- **description**: Introduction to ROS 2 as the core nervous system of humanoid robots
- **chapters**: [Module01_Index, Chapter1, Chapter2, Chapter3]
- **prerequisites**: Basic Python knowledge
- **learningObjectives**: Explain ROS 2 architecture, create ROS 2 nodes, understand URDF, etc.

## Content Elements

### Entity: DocumentationPage
- **title**: Page title
- **content**: Markdown content for the page
- **sidebarPosition**: Position in sidebar navigation
- **frontmatter**: Docusaurus frontmatter configuration
- **links**: Internal links to other pages
- **type**: "introduction", "module", "chapter", etc.

### Entity: SidebarConfiguration
- **id**: Unique identifier for the sidebar
- **items**: List of navigation items
- **type**: "category", "doc", etc.
- **label**: Display label for the navigation item
- **children**: Child items for categories

## Navigation Structure

### Entity: BookSidebar
- **id**: "book-sidebar"
- **items**: [BookIntroduction, ModulesCategory]
- **configuration**: Manual sidebar configuration for the book structure
- **integration**: Main navigation for the book-only structure

### Entity: ModulesCategory
- **type**: "category"
- **label**: "Course Modules"
- **items**: [Module01Category, Module02Category, ...]
- **collapsed**: Whether the category is collapsed by default
- **collapsible**: Whether the category can be collapsed

### Entity: ModuleCategory
- **type**: "category"
- **label**: Module title
- **items**: [ModuleIndex, Chapter1, Chapter2, Chapter3]
- **collapsed**: Whether the category is collapsed by default
- **collapsible**: Whether the category can be collapsed