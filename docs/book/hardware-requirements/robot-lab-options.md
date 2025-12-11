---
title: Robot Lab Options
sidebar_position: 4
description: Options for the physical robot lab depending on budget and requirements
---

# The Robot Lab

Options for the physical robot, depending on budget.

## Option A: The "Proxy" Approach (Budget Recommended)

### Hardware
- **Unitree Go2 Edu** (~$1,800 - $3,000)

### Pros
- Durable, excellent ROS 2 support

### Cons
- Quadruped (dog), not a biped

## Option B: The "Miniature Humanoid" Approach

### Hardware
- **Unitree G1** (~$16k) or **Robotis OP3** (~$12k)

### Budget Alternative
- **Hiwonder TonyPi Pro** (~$600)

### Warning
Cheap kits often run on Raspberry Pi, which cannot run NVIDIA Isaac ROS efficiently.

## Option C: The "Premium" Lab (Sim-to-Real)

### Hardware
- **Unitree G1 Humanoid**

### Why
- Capable of dynamic walking with an open SDK for custom ROS 2 controllers

## Cloud Alternative (High OpEx)

For students without access to RTX Workstations:

### Instance
- **AWS g5.2xlarge** (A10G GPU, 24GB VRAM) or **g6e.xlarge**

### Software
- **NVIDIA Isaac Sim** on Omniverse Cloud

### Est. Cost
- ~$205 per quarter (based on ~120 hours of usage)

### Constraint
You still need the local Edge AI Kit and Physical Robot to deploy code, as controlling robots from the cloud introduces dangerous latency.