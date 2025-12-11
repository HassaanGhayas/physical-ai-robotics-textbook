# Quickstart Guide: Physical AI & Humanoid Robotics Book

## Prerequisites
- Node.js v18+ installed
- npm or yarn package manager
- Git for version control
- GitHub account for deployment
- Basic understanding of robotics concepts (helpful but not required)

## Setup Instructions

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Install dependencies**
   ```bash
   npm install
   # or
   yarn install
   ```

3. **Start development server**
   ```bash
   npm start
   # or
   yarn start
   ```

4. **Navigate the book structure**
   The book is organized into 7 modules:
   - Module 1: Hardware Requirements
   - Module 2: Introduction
   - Module 3: The Robotic Nervous System (ROS 2)
   - Module 4: The Digital Twin (Gazebo & Unity)
   - Module 5: The AI-Robot Brain (NVIDIA Isaac™)
   - Module 6: Vision-Language-Action (VLA)
   - Module 7: Assessments

## Adding Book Content

1. **Organize content by module**
   ```
   docs/
   ├── intro.md
   ├── hardware-requirements/
   │   ├── index.md
   │   ├── digital-twin-workstation.md
   │   ├── edge-kit.md
   │   └── ...
   ├── introduction/
   │   └── index.md
   ├── robotic-nervous-system/
   │   ├── index.md
   │   └── ...
   └── ... (other modules)
   ```

2. **Add frontmatter to your Markdown files**
   ```markdown
   ---
   title: Digital Twin Workstation
   sidebar_position: 1
   description: Hardware requirements for simulation and training
   ---

   # Digital Twin Workstation

   This section covers the hardware requirements for the Digital Twin workstation...
   ```

3. **Update sidebar navigation**
   Edit `sidebars.js` to include your new content in the module-based navigation structure.

## Technical Content Features

1. **Hardware Specifications Tables**
   Use standard Markdown tables to display hardware specs:
   ```markdown
   | Component | Minimum | Ideal | Reasoning |
   |-----------|---------|-------|-----------|
   | GPU | RTX 4070 Ti (12GB VRAM) | RTX 3090/4090 (24GB) | High VRAM for USD assets and VLA models |
   ```

2. **Code Examples**
   Use code blocks with appropriate language specification:
   ```python
   # ROS 2 code example
   import rclpy
   from rclpy.node import Node
   ```

3. **Mathematical Equations**
   Use LaTeX syntax for equations:
   ```markdown
   $$
   \text{Force} = \text{Mass} \times \text{Acceleration}
   $$
   ```

## Custom Components

1. **Hardware Specification Component**
   - Located in `src/components/HardwareSpecs/`
   - Displays hardware specifications in a structured format
   - Includes cost, pros, cons, and purpose

2. **Technical Diagram Component**
   - Located in `src/components/TechnicalDiagrams/`
   - Optimized for displaying technical diagrams
   - Supports zoom and annotation features

## Deploying to GitHub Pages

1. **Configure deployment settings**
   - Set the correct repository and branch in deployment configuration
   - Ensure your `docusaurus.config.js` has the correct baseUrl and deployment settings

2. **Deploy**
   ```bash
   GIT_USER=<your-github-username> npm run deploy
   # or
   USE_SSH=true npm run deploy
   ```

## Running Tests

1. **Unit tests**
   ```bash
   npm test
   # or
   yarn test
   ```

2. **End-to-end tests**
   ```bash
   npm run e2e
   # or
   yarn e2e
   ```

## Helpful Commands

- `npm start` - Start development server
- `npm run build` - Build static files for deployment
- `npm run serve` - Serve built files locally for testing
- `npm run deploy` - Deploy to GitHub Pages
- `npm run docusaurus` - Show all Docusaurus commands

## Module-Specific Guidelines

- **Module 1 (Hardware Requirements)**: Focus on clear specification tables and cost breakdowns
- **Module 3 (ROS 2)**: Emphasize code examples and node architecture diagrams
- **Module 5 (NVIDIA Isaac)**: Include technical setup instructions and configuration files
- **Module 6 (VLA)**: Provide practical examples of LLM integration with robotics
- **Module 7 (Assessments)**: Include clear project requirements and evaluation criteria