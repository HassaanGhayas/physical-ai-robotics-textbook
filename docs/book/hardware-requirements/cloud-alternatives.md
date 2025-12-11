---
title: Cloud Alternatives
sidebar_position: 5
description: Cloud-based alternatives for students without access to high-end hardware
---

# Cloud Alternative (High OpEx)

For students without access to RTX Workstations.

## Instance Options

### AWS
- **g5.2xlarge** (A10G GPU, 24GB VRAM)
- **g6e.xlarge**

### Software
- **NVIDIA Isaac Sim** on Omniverse Cloud

## Cost Considerations

### Est. Cost
- ~$205 per quarter (based on ~120 hours of usage)

## Important Constraint

You still need the local Edge AI Kit and Physical Robot to deploy code, as controlling robots from the cloud introduces dangerous latency.

## When to Consider Cloud Alternatives

Cloud alternatives are suitable for:
- Students without access to RTX Workstations
- Short-term intensive computation needs
- Collaborative projects requiring consistent environments
- Prototyping before investing in physical hardware

## Limitations

- Higher operational expenses over time
- Latency constraints for real-time control
- Limited customization of underlying hardware
- Internet connectivity dependency