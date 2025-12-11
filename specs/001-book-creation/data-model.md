# Data Model: Physical AI & Humanoid Robotics Book

## Book Entity
- **title**: string - "Physical AI & Humanoid Robotics"
- **author**: string - Author(s) of the book
- **description**: string - A brief description of the book's content covering hardware, robotics, and AI
- **publicationDate**: date - When the book was published or last updated
- **modules**: array of Module entities - The 7 modules that make up the book
- **metadata**: object - Additional metadata like tags, categories, prerequisites, etc.

## Module Entity
- **id**: string - Unique identifier for the module (e.g., "hardware-requirements", "ros2-nervous-system")
- **title**: string - The title of the module
- **number**: integer - The sequential number of the module (1-7)
- **description**: string - A brief description of the module content
- **topics**: array of Topic entities - The topics within the module
- **prerequisites**: array of string - Prerequisites needed before starting this module
- **learningObjectives**: array of string - Learning objectives for the module
- **bookId**: string - Reference to the parent book

## Topic Entity
- **id**: string - Unique identifier for the topic
- **title**: string - The title of the topic
- **number**: integer - The sequential number of the topic within the module
- **content**: string - The main content of the topic (Markdown format)
- **subtopics**: array of Subtopic entities - The subtopics within the topic
- **codeExamples**: array of CodeExample entities - Code examples in this topic
- **diagrams**: array of Diagram entities - Technical diagrams for this topic
- **moduleId**: string - Reference to the parent module

## Subtopic Entity
- **id**: string - Unique identifier for the subtopic
- **title**: string - The title of the subtopic
- **number**: integer - The sequential number of the subtopic within the topic
- **content**: string - The content of the subtopic (Markdown format)
- **resources**: array of Resource entities - Additional resources for this subtopic
- **topicId**: string - Reference to the parent topic

## HardwareSpec Entity
- **id**: string - Unique identifier for the hardware specification
- **name**: string - Name of the hardware component (e.g., "NVIDIA RTX 4070 Ti", "Jetson Orin Nano")
- **category**: enum (gpu, cpu, ram, os, compute, vision, balance, voice, robot) - Category of hardware
- **specifications**: object - Detailed specifications (VRAM, cores, memory, etc.)
- **purpose**: string - The role/purpose of this hardware in the context
- **cost**: string - Estimated cost
- **pros**: array of string - Advantages of this hardware
- **cons**: array of string - Disadvantages of this hardware
- **relatedTopics**: array of string - Topics that reference this hardware
- **moduleId**: string - Reference to the module that discusses this hardware

## CodeExample Entity
- **id**: string - Unique identifier for the code example
- **title**: string - Brief title describing the code example
- **language**: string - Programming language (e.g., "Python", "C++", "ROS2")
- **code**: string - The actual code content
- **description**: string - Explanation of what the code does
- **topicId**: string - Reference to the topic that contains this example
- **moduleId**: string - Reference to the module that contains this example

## Diagram Entity
- **id**: string - Unique identifier for the diagram
- **title**: string - Title of the diagram
- **type**: enum (architecture, workflow, hardware-setup, simulation, ai-flow) - Type of diagram
- **description**: string - Description of what the diagram illustrates
- **imageUrl**: string - Path to the diagram image file
- **topicId**: string - Reference to the topic that contains this diagram
- **moduleId**: string - Reference to the module that contains this diagram

## Assessment Entity
- **id**: string - Unique identifier for the assessment
- **title**: string - Title of the assessment
- **type**: enum (project, implementation, pipeline, capstone) - Type of assessment
- **description**: string - Detailed description of the assessment
- **requirements**: array of string - Requirements for completing the assessment
- **moduleId**: string - Reference to the module (typically module 7 for assessments)
- **relatedModules**: array of string - Modules whose concepts are tested in this assessment

## Theme Entity
- **id**: string - Unique identifier for the theme
- **name**: string - The name of the theme (e.g., "technical-documentation", "dark-mode")
- **colors**: object - Color scheme optimized for technical documentation
- **fonts**: object - Font configuration for code, headers, and body text
- **layout**: object - Layout options optimized for technical content readability
- **animation**: object - Animation settings for transitions and effects
- **bookId**: string - Reference to the book that uses this theme

## DeploymentConfig Entity
- **id**: string - Unique identifier for the deployment configuration
- **repository**: string - GitHub repository name for deployment
- **branch**: string - Target branch for GitHub Pages deployment
- **buildScript**: string - Script to run for building the technical documentation site
- **customDomain**: string - Optional custom domain for the book
- **bookId**: string - Reference to the book that uses this configuration