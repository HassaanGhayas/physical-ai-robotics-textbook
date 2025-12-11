---
title: Physical AI Edge Kit
sidebar_position: 3
description: Hardware requirements for the Physical AI Edge Kit for deployment
---

# The "Physical AI" Edge Kit

Required to learn "Physical AI" by deploying the nervous system on a desk before using a full robot.

## The Brain (Compute)

- **NVIDIA Jetson Orin Nano** (8GB) or **Orin NX** (16GB)
- **Role**: Industry standard for embodied AI; used to run ROS 2 nodes

## The Eyes (Vision)

- **Intel RealSense D435i** or **D455**
- **Role**: Provides RGB and Depth data for VSLAM and Perception

## The Inner Ear (Balance)

- **Generic USB IMU** (BNO055)
- **Note**: Often built into RealSense D435i, but a separate module is useful for calibration training

## Voice Interface

- **USB Microphone/Speaker array** (e.g., ReSpeaker)
- **Role**: For "Voice-to-Action" Whisper integration

## The Economy Jetson Student Kit (Breakdown)

| Component | Cost | Details | Notes |
|-----------|------|---------|--------|
| The Brain | ~$249 | Nano Super Dev Kit (8GB) - Capable of 40 TOPS | |
| The Eyes | ~$349 | Intel RealSense D435i - Must be the "i" version (includes IMU) | |
| The Ears | ~$69 | ReSpeaker USB Mic Array v2.0 - Far-field mic for voice commands | |
| Misc | ~$30 | 128GB High-Endurance SD Card + Jumper Wires - Required for OS and connections | |
| **TOTAL** | **~$700** | | |

## The Robot Lab

Options for the physical robot, depending on budget.

### Option A: The "Proxy" Approach (Budget Recommended)

- **Hardware**: Unitree Go2 Edu (~$1,800 - $3,000)
- **Pros**: Durable, excellent ROS 2 support
- **Cons**: Quadruped (dog), not a biped

### Option B: The "Miniature Humanoid" Approach

- **Hardware**: Unitree G1 (~$16k) or Robotis OP3 (~$12k)
- **Budget Alternative**: Hiwonder TonyPi Pro (~$600)
- **Warning**: Cheap kits often run on Raspberry Pi, which cannot run NVIDIA Isaac ROS efficiently

### Option C: The "Premium" Lab (Sim-to-Real)

- **Hardware**: Unitree G1 Humanoid
- **Why**: Capable of dynamic walking with an open SDK for custom ROS 2 controllers

## Cloud Alternative (High OpEx)

For students without access to RTX Workstations.

- **Instance**: AWS g5.2xlarge (A10G GPU, 24GB VRAM) or g6e.xlarge
- **Software**: NVIDIA Isaac Sim on Omniverse Cloud
- **Est. Cost**: ~$205 per quarter (based on ~120 hours of usage)
- **Constraint**: You still need the local Edge AI Kit and Physical Robot to deploy code, as controlling robots from the cloud introduces dangerous latency