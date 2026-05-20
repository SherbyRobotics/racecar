
# 🏎️ Architecture Documentation

This section describes the architecture of the racecar system.

## Overview

The system is built using a modular ROS2 architecture that supports both:

- Simulation (Gazebo)
- Real vehicle hardware (VESC)

The same control flow is reused for both environments.

---

## 📊 Architecture Diagrams

- [System Context](context.md)  
  High-level view of how the system interacts with users and external systems.

- [Component Map](components.md)  
  Internal structure of the system and relationships between modules.

- [Sequence Diagram](sequences.md)  
  Step-by-step command flow through the system.

---

## 🧠 Summary

The architecture follows a layered approach:
