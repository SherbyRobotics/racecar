---

# ✅ 📁 docs/architecture/components.md

```markdown
# Component Map

This diagram shows the internal structure of the system and relationships between key components.

```mermaid
flowchart TD
    Teleop[Teleop / Joy Input]
    Arbitration[cmd_vel_arbitration]
    Shim[cmd_vel_to_ackermann_drive]
    Controller[Ackermann Controllers]
    Bridge[ros_gz_bridge]
    Gazebo[Gazebo Ackermann Plugin]
    VESC[VESC Driver]

    Teleop --> Arbitration
    Arbitration --> Shim
    Shim --> Controller

    Controller --> Bridge
    Bridge --> Gazebo

    Controller --> VESC
