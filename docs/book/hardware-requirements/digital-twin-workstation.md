---
title: Digital Twin Workstation
sidebar_position: 2
description: Hardware requirements for the Digital Twin workstation for simulation and training
---

# The "Digital Twin" Workstation

Required for every student for simulation and training.

## GPU (Critical Bottleneck)

### Minimum
- **NVIDIA RTX 4070 Ti** (12GB VRAM)

### Ideal
- **RTX 3090 or 4090** (24GB VRAM) for smoother Sim-to-Real training

### Reasoning
High VRAM is required to load Universal Scene Description (USD) assets and run Vision-Language-Action (VLA) models simultaneously.

## CPU Requirements

- **Intel Core i7** (13th Gen+) or **AMD Ryzen 9**

### Reasoning
Needed for CPU-intensive physics calculations (Rigid Body Dynamics).

## RAM Requirements

- **64 GB DDR5**

### Note
32 GB is the absolute minimum but may crash during complex scene rendering.

## Operating System

- **Ubuntu 22.04 LTS**

### Note
Dual-booting or dedicated Linux machines are mandatory; ROS 2 is native to Linux.

## Additional Considerations

- High-speed SSD storage (1TB+ recommended for asset storage)
- Sufficient cooling for sustained GPU workloads
- Multiple display outputs for efficient workflow