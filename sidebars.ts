import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Sidebar for the Physical AI & Humanoid Robotics Book
  bookSidebar: [
    {
      type: 'category',
      label: 'Documentation',
      collapsed: false,
      items: [
        'documentation/components-guide',
        'documentation/accessibility',
        'documentation/performance',
      ],
    },
    {
      type: 'category',
      label: '1. Hardware Requirements',
      collapsed: false,
      items: [
        'book/hardware-requirements/index',
        'book/hardware-requirements/digital-twin-workstation',
        'book/hardware-requirements/edge-kit',
        'book/hardware-requirements/robot-lab-options',
        'book/hardware-requirements/cloud-alternatives',
      ],
    },
    {
      type: 'category',
      label: '2. Introduction',
      collapsed: false,
      items: [
        'book/introduction/index',
      ],
    },
    {
      type: 'category',
      label: '3. Robotic Nervous System (ROS 2)',
      collapsed: false,
      items: [
        'book/robotic-nervous-system/index',
        'book/robotic-nervous-system/ros2-nodes-topics-services',
        'book/robotic-nervous-system/bridging-python-agents',
        'book/robotic-nervous-system/urdf-for-humanoids',
      ],
    },
    {
      type: 'category',
      label: '4. Digital Twin (Gazebo & Unity)',
      collapsed: false,
      items: [
        'book/digital-twin/index',
        'book/digital-twin/gazebo-simulation',
        'book/digital-twin/unity-rendering',
        'book/digital-twin/sensor-simulation',
      ],
    },
    {
      type: 'category',
      label: '5. AI-Robot Brain (NVIDIA Isaac™)',
      collapsed: false,
      items: [
        'book/ai-robot-brain/index',
        'book/ai-robot-brain/isaac-sim',
        'book/ai-robot-brain/isaac-ros',
        'book/ai-robot-brain/nav2-path-planning',
      ],
    },
    {
      type: 'category',
      label: '6. Vision-Language-Action (VLA)',
      collapsed: false,
      items: [
        'book/vision-language-action/index',
        'book/vision-language-action/voice-to-action',
        'book/vision-language-action/cognitive-planning',
        'book/vision-language-action/capstone-project',
      ],
    },
    {
      type: 'category',
      label: '7. Assessments',
      collapsed: false,
      items: [
        'book/assessments/index',
        'book/assessments/ros2-package-project',
        'book/assessments/gazebo-implementation',
        'book/assessments/comprehensive-capstone',
      ],
    },
  ],
};

export default sidebars;
