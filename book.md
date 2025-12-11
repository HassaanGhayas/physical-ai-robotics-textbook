
# Book Content

1 - Hardware Requirements: This course sits at the intersection of Physics Simulation, Visual Perception, and Generative AI. The hardware is categorized into three tiers:

The Digital Twin Workstation (Simulation), The Edge Kit (Physical Deployment), and The Robot Lab.1. The "Digital Twin" Workstation

Required for every student for simulation and training.

GPU (Critical Bottleneck):

Minimum: NVIDIA RTX 4070 Ti (12GB VRAM).

Ideal: RTX 3090 or 4090 (24GB VRAM) for smoother Sim-to-Real training.

Reasoning: High VRAM is required to load Universal Scene Description (USD) assets and run Vision-Language-Action (VLA) models simultaneously.

CPU: Intel Core i7 (13th Gen+) or AMD Ryzen 9.

Reasoning: Needed for CPU-intensive physics calculations (Rigid Body Dynamics).

RAM: 64 GB DDR5.

Note: 32 GB is the absolute minimum but may crash during complex scene rendering.

Operating System: Ubuntu 22.04 LTS.

Note: Dual-booting or dedicated Linux machines are mandatory; ROS 2 is native to Linux.2. The "Physical AI" Edge Kit

Required to learn "Physical AI" by deploying the nervous system on a desk before using a full robot.

The Brain (Compute): NVIDIA Jetson Orin Nano (8GB) or Orin NX (16GB).

Role: Industry standard for embodied AI; used to run ROS 2 nodes.

The Eyes (Vision): Intel RealSense D435i or D455.

Role: Provides RGB and Depth data for VSLAM and Perception.

The Inner Ear (Balance): Generic USB IMU (BNO055).

Note: Often built into RealSense D435i, but a separate module is useful for calibration training.

Voice Interface: USB Microphone/Speaker array (e.g., ReSpeaker).

Role: For "Voice-to-Action" Whisper integration.

The Economy Jetson Student Kit (Breakdown)

Nano Super Dev Kit (8GB) | ~$249 | Capable of 40 TOPS. |

| The Eyes | Intel RealSense D435i | ~$349 | Must be the "i" version (includes IMU). |

| The Ears | ReSpeaker USB Mic Array v2.0 | ~$69 | Far-field mic for voice commands. |

| Misc | 128GB High-Endurance SD Card + Jumper Wires | ~$30 | Required for OS and connections. |

| TOTAL |  | ~$700 |  |.

The Robot Lab

Options for the physical robot, depending on budget.

Option A: The "Proxy" Approach (Budget Recommended)

Hardware: Unitree Go2 Edu (~$1,800 - $3,000).

Pros: Durable, excellent ROS 2 support.

Cons: Quadruped (dog), not a biped.

Option B: The "Miniature Humanoid" Approach

Hardware: Unitree G1 (~$16k) or Robotis OP3 (~$12k).

Budget Alternative: Hiwonder TonyPi Pro (~$600).

Warning: Cheap kits often run on Raspberry Pi, which cannot run NVIDIA Isaac ROS efficiently.

Option C: The "Premium" Lab (Sim-to-Real)

Hardware: Unitree G1 Humanoid.

Why: Capable of dynamic walking with an open SDK for custom ROS 2 controllers.4. Cloud Alternative (High OpEx)For students without access to RTX Workstations.

Instance: AWS g5.2xlarge (A10G GPU, 24GB VRAM) or g6e.xlarge.Software: NVIDIA Isaac Sim on Omniverse Cloud.Est. Cost: ~$205 per quarter (based on ~120 hours of usage).Constraint: You still need the local Edge AI Kit and Physical Robot to deploy code, as controlling robots from the cloud introduces dangerous latency.

2 - Introduction

3 - Module 1: The Robotic Nervous System (ROS 2)

 	- Focus: Middleware for robot control.

 	- ROS 2 Nodes, Topics, and Services.

 	- Bridging Python Agents to ROS controllers using rclpy.

 	- Understanding URDF (Unified Robot Description Format) for humanoids.

4 - Module 2: The Digital Twin (Gazebo \& Unity)

 	- Focus: Physics simulation and environment building.

 	- Simulating physics, gravity, and collisions in Gazebo.

 	- High-fidelity rendering and human-robot interaction in Unity.

 	- Simulating sensors: LiDAR, Depth Cameras, and IMUs.

5 - Module 3: The AI-Robot Brain (NVIDIA Isaac™)

 	- Focus: Advanced perception and training.

 	- NVIDIA Isaac Sim: Photorealistic simulation and synthetic data generation.

 	- Isaac ROS: Hardware-accelerated VSLAM (Visual SLAM) and navigation.

 	- Nav2: Path planning for bipedal humanoid movement.

6 - Module 4: Vision-Language-Action (VLA)

 	- Focus: The convergence of LLMs and Robotics.

 	- Voice-to-Action: Using OpenAI Whisper for voice commands.

 	- Cognitive Planning: Using LLMs to translate natural language ("Clean the room") into a sequence of ROS 2 actions.

 	- Capstone Project: The Autonomous Humanoid. A final project where a simulated robot receives a voice command, plans a path, navigates obstacles, identifies an object using computer vision, and manipulates it.

7 - Assessments

 	- ROS 2 package development project

 	- Gazebo simulation implementation

 	- Isaac-based perception pipeline

 	- Capstone: Simulated humanoid robot with conversational AI
