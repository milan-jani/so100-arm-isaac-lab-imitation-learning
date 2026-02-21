# SO-100 ARM INTEGRATION WITH ISAAC LAB

<div align="center">

![Status](https://img.shields.io/badge/Status-Working-success)
![Isaac%20Lab](https://img.shields.io/badge/Isaac%20Lab-1.0+-blue)
![Isaac%20Sim](https://img.shields.io/badge/Isaac%20Sim-5.0-green)
![Python](https://img.shields.io/badge/Python-3.10+-yellow)

**Complete step-by-step guide for integrating custom robotic arms into NVIDIA Isaac Lab**

[Features](#features) • [Demo](#demo) • [Installation](#prerequisites) • [Quick Start](#quick-start) • [Documentation](#table-of-contents)

</div>

---

## 🎯 What is This?

This repository provides a **complete, beginner-friendly guide** to integrate a custom 6-DOF robotic arm (SO-100) into NVIDIA Isaac Lab for **Imitation Learning** research. 

If you're working on:
- 🤖 Custom robot manipulation in simulation
- 🎓 Imitation Learning / Behavior Cloning research
- 📊 Robot learning data collection
- 🔬 Sim-to-real transfer experiments

**Then this guide is for you!**

---

## ✨ Features

- ✅ **Complete Integration Pipeline** - From zero to working robot in Isaac Lab
- ✅ **Two Task Environments** - Reach (position control) & Lift (object manipulation)
- ✅ **Keyboard Teleoperation** - WASD controls for easy demonstration collection
- ✅ **Physical Leader Arm Teleoperation** - Real SO-100 arm mirrors simulation in real time
- ✅ **Auto-Calibration** - Timed range + dedicated gripper 2-point calibration at startup
- ✅ **Zero-Diff Position Tracking** - Physics offset correction for exact 1:1 joint mapping
- ✅ **USB Serial Resilience** - Survives disconnects, holds last position, auto-recovers
- ✅ **Collision Detection Setup** - Step-by-step USD collision mesh configuration
- ✅ **Performance Tuning** - Optimized stiffness/damping for fast, stable control
- ✅ **IL-Ready** - Configured for Behavior Cloning and RL training
- ✅ **Troubleshooting Guide** - Common errors and solutions included
- ✅ **Beginner Friendly** - No prior Isaac Lab experience needed

---

## 🎬 Demo

### Reach Task
Robot moves end-effector to target positions using keyboard control.

```bash
./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py \
    --task Isaac-Reach-SO100-IK-Abs-v0 --teleop_device keyboard
```

### Lift Task (IK Control - Coordinated Movement)
Robot picks up and manipulates cube with IK control (multiple joints move together).

```bash
./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py \
    --task Isaac-Lift-Cube-SO100-IK-Abs-v0 --teleop_device keyboard
```

### Lift Task (Joint Control - Independent Movement) ⭐
Robot with independent joint control (each key controls ONE joint only).

```bash
./isaaclab.sh -p scripts/environments/teleoperation/teleop_joint_agent.py \
    --task Isaac-Lift-Cube-SO100-v0 --joint_delta 0.1
```

### Leader Arm Teleoperation ⭐ NEW (Feb 2026)
Physically move the real SO-100 leader arm — simulation arm mirrors every joint in real time.

```bash
# Quick run using alias (add to ~/.bash_aliases first):
teleop_so100

# Or directly:
cd ~/IsaacLab && ./isaaclab.sh -p scripts/environments/teleoperation/teleop_so100_leader.py \
    --task Isaac-Lift-Cube-SO100-v0 --port /dev/ttyACM0
```

**Startup sequence:**
1. Connect SO-100 leader arm via USB (`/dev/ttyACM0` default)
2. Environment loads and physics warms up (30 steps)
3. **8-second calibration** — move ALL arm joints through full range
4. **Gripper calibration skipped** — saved values used automatically (`GRIPPER_RAW_CLOSED=2027`, `GRIPPER_RAW_OPEN=3172`)
5. Sim arm snaps to leader arm position — teleoperation begins

**Real-time terminal display (updates every 10 steps):**
```
  Step   340  |    Raw |   Norm | Leader(rad)  |   Sim(rad) |     Diff
  ──────────────────────────────────────────────────────────────────────
  Shoulder Pan  |  1986 | -0.061 |     -0.1211  |    -0.1211 |  +0.0000
  Shoulder Lift |   948 | -1.000 |     +0.0000  |    +0.0000 |  +0.0000
  Elbow Flex    |  3021 | +0.950 |     -0.0782  |    -0.0782 |  +0.0000
  Wrist Flex    |  2245 | +0.192 |     -0.2941  |    -0.2941 |  +0.0000
  Wrist Roll    |  1025 | -0.999 |     -3.1385  |    -3.1385 |  +0.0000
  Gripper/Jaw   |  2027 | -0.021 |     +0.8774  |    +0.8774 |  +0.0000
  BACKSPACE = reset  |  Ctrl+C = quit
```

**Controls:**
- Move physical arm → sim arm mirrors exactly
- `BACKSPACE` in Isaac Sim window → reset environment + re-snap to leader position
- `Ctrl+C` in terminal → quit

**IK Control Keyboard (teleop_se3_agent.py):**
- `W/S/A/D/Q/E` - Move end-effector (position)
- `Z/X/T/G/C/V` - Rotate end-effector (orientation)
- `K` - Toggle gripper open/close
- `R` - Reset environment

**Joint Control Keyboard (teleop_joint_agent.py):**
- `Q/A` - Shoulder Pan (base rotation)
- `W/S` - Shoulder Lift (arm up/down)
- `E/D` - Elbow Flex (extend/retract)
- `C/V` - Wrist Flex (wrist up/down)
- `T/G` - Wrist Roll (wrist rotation)
- `Z/X` - Gripper (open/close) - **INCREMENTAL CONTROL!**
- `BACKSPACE` - Reset environment
- `F & SPACE` - Blocked (prevents camera reset & pause)

---

## 🚀 Quick Start

### 1. Prerequisites

**Software Requirements:**
- [NVIDIA Isaac Sim 5.0+](https://developer.nvidia.com/isaac-sim)
- [Isaac Lab 1.0+](https://github.com/isaac-sim/IsaacLab)
- Python 3.10+
- Ubuntu 22.04 (recommended)

**Hardware Requirements:**
- NVIDIA GPU (RTX 3060+ recommended)
- 16GB+ RAM
- 50GB+ free disk space

### 2. Clone This Repository

```bash
cd ~/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/reach/config/
git clone https://github.com/YOUR_USERNAME/so100-isaac-lab.git so100
```

### 3. Prepare Your Robot USD File

Place your robot USD file at:
```
/path/to/your/robot.usd
```

Update the path in `so100.py`:
```python
usd_path="/path/to/your/robot.usd"
```

### 4. Follow the Guide

Jump to [Step 1: Create Robot Asset Configuration](#step-1-create-so-100-robot-asset-configuration) below.

---

## 📖 Table of Contents

### Part 1: Reach Task Setup
1. [Create Robot Asset Configuration](#step-1-create-so-100-robot-asset-configuration)
2. [Export Robot in Package Init](#step-2-export-so-100-in-package-init)
3. [Create Reach Environment Configs](#step-3-create-reach-task-configurations)
4. [Register Reach Tasks](#step-4-register-reach-tasks-with-gymnasium)
5. [Update Parent Init File](#step-5-add-so-100-to-parent-init)
6. [Test Reach Task](#step-6-test-reach-task)

### Part 2: Lift Task Setup
7. [Create Lift Task Directory](#step-7-create-lift-task-directory)
8. [Create Joint Position Config](#step-8-create-joint-position-config-for-lift)
9. [Create IK Control Config](#step-9-create-ik-control-config-for-lift)
10. [Register Lift Tasks](#step-10-register-lift-tasks)
11. [Update Parent Init](#step-11-add-so-100-to-parent-init-file)
12. [Update Robot Asset](#step-12-update-so-100-robot-asset-for-lift)

### Part 3: Critical Setup
13. [Add Collision Meshes (REQUIRED)](#step-13-add-collision-meshes-in-isaac-sim-50)
14. [Clear Python Cache](#step-14-clear-python-cache)
15. [Test Lift Task](#step-15-test-lift-task-with-keyboard)

### Part 4: Independent Joint Control
16. [Create Joint Position Teleop Script](#step-16-create-independent-joint-control-script)
17. [Understanding Control Modes](#step-17-ik-vs-joint-position-control)
18. [Customizing Key Mappings](#step-18-customize-keyboard-mappings)

### Part 5: Physical Leader Arm Teleoperation ⭐ NEW (Feb 2026)
19. [Overview & Architecture](#step-19-leader-arm-teleoperation-overview)
20. [Hardware Setup](#step-20-hardware-setup)
21. [Calibration System](#step-21-calibration-system)
22. [Position Mapping & Offset Fix](#step-22-position-mapping--offset-correction)
23. [Response Time Tuning](#step-23-response-time-tuning)
24. [Terminal Display & Debugging](#step-24-real-time-terminal-display)
25. [Alias Setup](#step-25-shell-alias-setup)

### Additional Sections
- [Keyboard Controls](#keyboard-controls-same-as-reach)
- [Troubleshooting](#troubleshooting-common-issues)
- [Configuration Summary](#configuration-summary)
- [File Structure](#file-structure-overview)
- [Testing Checklist](#testing-checklist)
- [Future Improvements](#future-improvements--next-steps)
- [Resources](#recommended-resources)

---

## 🤔 Why This Guide?

**Problem:** NVIDIA Isaac Lab documentation focuses on pre-built robots (Franka, UR10). Integrating custom robots requires understanding multiple components, and errors can be confusing for beginners.

**Solution:** This guide provides a **real-world example** of custom robot integration with:
- ✅ Every file you need to create
- ✅ Exact code with explanations
- ✅ Common errors and how to fix them
- ✅ Performance tuning tips
- ✅ Ready for Imitation Learning workflows

---

## 📊 Project Status

| Component | Status | Notes |
|-----------|--------|-------|
| Robot Asset Config | ✅ Working | High stiffness for fast movement |
| Reach Task | ✅ Working | IK control with keyboard |
| Lift Task | ✅ Working | Collision detection enabled |
| Keyboard Teleoperation | ✅ Working | WASD + independent joint keys |
| **Leader Arm Teleoperation** | ✅ **Working** | **Real arm → sim mirror, ~0 diff** |
| **Gripper Calibration** | ✅ **Working** | **2-point saved calibration** |
| **Serial Resilience** | ✅ **Working** | **USB disconnect survivable** |
| IL Data Collection | 🔲 TODO | Recording pipeline in progress |
| Trained BC Model | 🔲 TODO | Training after data collection |

---

## 🙏 Acknowledgments

- **NVIDIA Isaac Lab Team** - For excellent robotics simulation framework
- **Isaac Sim Documentation** - Comprehensive physics and USD guides
- **Milan Jani** - Project author and maintainer

---

## 📄 License

This project is licensed under the BSD-3-Clause License - see Isaac Lab's license for details.

---

## 📧 Contact

- **Author:** Milan Jani
- **Last Updated:** February 19, 2026
- **GitHub:** [Your GitHub Profile]
- **YouTube:** [Your YouTube Channel]

For questions, issues, or contributions, please open an issue on GitHub!

---

## ⭐ Star This Repository

If this guide helped you integrate your robot into Isaac Lab, please **star this repository** and share it with others! 🌟

---

# DETAILED INTEGRATION GUIDE

Below is the complete step-by-step integration process. Each step includes file paths, exact code, and explanations.

---

## Author: Milan Jani
## Status: ✅ WORKING - Reach & Lift tasks with IK control and keyboard teleoperation

---

## OVERVIEW OF INTEGRATION PROCESS

This guide shows you how to integrate a custom SO-100 robotic arm into Isaac Lab step-by-step. You'll learn how to create reach and lift tasks, enable keyboard control, and fix collision issues for data collection in Imitation Learning (IL).

**What You'll Build:**
1. Robot asset configuration with physics properties
2. Reach task for position control practice
3. Lift task for object manipulation with gripper
4. Collision detection for realistic physics
5. Keyboard teleoperation for data collection

**Time Required:** 2-3 hours (first time), 30 minutes (if following exactly)

---

## PREREQUISITES

Before starting, ensure you have:

### Software Installed:
- ✅ Isaac Lab 1.0+ installed at: `~/IsaacLab` (or your custom path)
- ✅ Isaac Sim 5.0+ properly configured
- ✅ Python 3.10+ environment active

### Robot Requirements:
- ✅ Robot USD file available (e.g., `~/Downloads/SO100/so100/so100.usd` - replace with your path)
- ✅ Know your robot's joint names (e.g., shoulder_pan, elbow_flex, etc.)
- ✅ Know your end-effector body name (e.g., "jaw", "gripper_link", etc.)

### Knowledge Requirements:
- Basic Python programming
- Familiarity with terminal/bash commands
- Basic understanding of robotics (joints, end-effector, etc.)
- No prior Isaac Lab experience needed!

---

## ROBOT SPECIFICATIONS (SO-100 Example)

For reference, the SO-100 arm used in this guide has:
- **Joints:** 6 DOF (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper)
- **End-Effector:** "jaw" body
- **Gripper:** Single joint (0.0 = closed, 0.25 = open)
- **USD Path:** `~/Downloads/SO100/so100/so100.usd` (replace with your robot's USD path)

**Adapt these values for your robot!**

---

## PART 1: REACH TASK SETUP (FOUNDATION)

The reach task teaches the robot to move its end-effector to target positions. This is the foundation before adding object manipulation.

---

## STEP 1: CREATE SO-100 ROBOT ASSET CONFIGURATION

This is the **most important file** - it defines your robot's physical properties, actuators, and behavior in Isaac Lab.

### File Created:
```
~/IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/so100.py
# Replace ~ with your Isaac Lab installation path
```

### Content:
```python
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for SO-100 robotic arm from custom build."""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

##
# Configuration
##

SO100_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="~/Downloads/SO100/so100/so100.usd",  # REPLACE: Path to your robot USD file
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,  # Prevents arm drift
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,  # USD may not have proper collision meshes
            solver_position_iteration_count=12,
            solver_velocity_iteration_count=1,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 0.0,
        },
    ),
    actuators={
        "shoulder": ImplicitActuatorCfg(
            joint_names_expr=["shoulder_.*"],
            effort_limit_sim=87.0,
            velocity_limit_sim=2.175,
            stiffness=400.0,  # High stiffness for stable IK control
            damping=80.0,
        ),
        "elbow": ImplicitActuatorCfg(
            joint_names_expr=["elbow_.*"],
            effort_limit_sim=87.0,
            velocity_limit_sim=2.175,
            stiffness=400.0,
            damping=80.0,
        ),
        "wrist": ImplicitActuatorCfg(
            joint_names_expr=["wrist_.*"],
            effort_limit_sim=12.0,
            velocity_limit_sim=2.61,
            stiffness=400.0,
            damping=80.0,
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["gripper"],
            effort_limit_sim=200.0,
            velocity_limit_sim=0.2,
            stiffness=2e3,
            damping=1e2,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)

"""Configuration of SO-100 robotic arm."""
```

### Key Points:
- **Import pattern:** Use `isaaclab.sim` NOT `omni.isaac.lab.sim`
- **Joint names:** Must match exactly with USD file (check in Isaac Sim Stage panel)
- **Gravity disabled:** Prevents unwanted drift during teleoperation
- **High stiffness (400):** Required for stable IK control
- **High damping (80):** Reduces oscillations

---

## STEP 2: EXPORT SO-100 CONFIG IN ASSETS PACKAGE

### File Modified:
```
~/IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/__init__.py
```

### Change:
Added at the end of file:
```python
from .so100 import *
```

This makes `SO100_CFG` importable from `isaaclab_assets` package.

---

## STEP 3: CREATE SO-100 REACH ENVIRONMENT - JOINT POSITION CONTROL

### File Created:
```
~/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/reach/config/so100/joint_pos_env_cfg.py
```

### Content:
```python
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

import math

from isaaclab.utils import configclass

import isaaclab_tasks.manager_based.manipulation.reach.mdp as mdp
from isaaclab_tasks.manager_based.manipulation.reach.reach_env_cfg import ReachEnvCfg

##
# Pre-defined configs
##
from isaaclab_assets import SO100_CFG  # isort: skip


##
# Environment configuration
##

@configclass
class SO100ReachEnvCfg(ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # switch robot to SO-100
        self.scene.robot = SO100_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        
        # override rewards - use 'jaw' as end-effector
        self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = ["jaw"]
        self.rewards.end_effector_position_tracking_fine_grained.params["asset_cfg"].body_names = ["jaw"]
        self.rewards.end_effector_orientation_tracking.params["asset_cfg"].body_names = ["jaw"]
        
        # override actions - SO-100 joint pattern (exclude gripper for reach task)
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", 
            joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"], 
            scale=0.5, 
            use_default_offset=True
        )
        
        # override command generator body
        self.commands.ee_pose.body_name = "jaw"
        self.commands.ee_pose.ranges.pitch = (math.pi, math.pi)


@configclass
class SO100ReachEnvCfg_PLAY(SO100ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
```

### Key Points:
- Inherits from `ReachEnvCfg` base class
- End-effector body: `jaw` (not `gripper` - jaw is the actual tip)
- Joint names: Explicit list (5 joints, gripper excluded)
- This config is for **direct joint position control** (not used with keyboard)

---

## STEP 4: CREATE SO-100 REACH ENVIRONMENT - IK CONTROL (FOR KEYBOARD)

### File Created:
```
~/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/reach/config/so100/ik_abs_env_cfg.py
```

### Content:
```python
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from isaaclab.utils import configclass

from . import joint_pos_env_cfg

##
# Pre-defined configs
##
from isaaclab_assets import SO100_CFG  # isort: skip


@configclass
class SO100ReachEnvCfg(joint_pos_env_cfg.SO100ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        
        # Set actions for IK control - keyboard sends delta commands
        # Using relative mode for keyboard compatibility
        self.actions.arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
            body_name="jaw",  # End-effector body
            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=True,  # Relative mode for keyboard delta commands
                ik_method="dls"
            ),
            scale=0.5,  # Scale down the commands for smoother control
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, 0.05]),
        )


@configclass
class SO100ReachEnvCfg_PLAY(SO100ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
```

### Key Points:
- **IK (Inverse Kinematics):** Converts SE3 pose commands to joint angles
- **Relative mode:** Accepts incremental position/rotation deltas from keyboard
- **DLS method:** Damped Least Squares IK solver
- **Scale 0.5:** Smoother, slower movements
- **Body offset:** Small Z offset for gripper tip

---

## STEP 5: REGISTER SO-100 TASKS

### File Created:
```
~/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/reach/config/so100/__init__.py
```

### Content:
```python
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""
SO-100 reach environment configurations.
"""

import gymnasium as gym

from .joint_pos_env_cfg import *
from .ik_abs_env_cfg import SO100ReachEnvCfg as SO100ReachEnvCfg_IK

##
# Register Gym environments.
##

##
# Joint Position Control
##

gym.register(
    id="Isaac-Reach-SO100-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.joint_pos_env_cfg:SO100ReachEnvCfg",
    },
)

gym.register(
    id="Isaac-Reach-SO100-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.joint_pos_env_cfg:SO100ReachEnvCfg_PLAY",
    },
)

##
# Inverse Kinematics - Absolute Pose Control (FOR SE3 KEYBOARD)
##

gym.register(
    id="Isaac-Reach-SO100-IK-Abs-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.ik_abs_env_cfg:SO100ReachEnvCfg",
    },
)
```

### Key Points:
- **3 tasks registered:**
  - `Isaac-Reach-SO100-v0` - Joint position control
  - `Isaac-Reach-SO100-Play-v0` - Joint position play mode
  - `Isaac-Reach-SO100-IK-Abs-v0` - IK control (for keyboard) ✅ USE THIS
- Task IDs must be unique across Isaac Lab
- `f"{__name__}"` ensures correct module path resolution

---

## STEP 6: ADD SO-100 TO CONFIG PARENT INIT

### File Modified:
```
~/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/reach/config/__init__.py
```

### Change:
Added at the end:
```python
from . import franka, so100, ur_10
```

This ensures SO-100 config package is imported when reach module loads.

---

## RUNNING THE TELEOPERATION

### Command:
```bash
cd ~/IsaacLab
./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py --task Isaac-Reach-SO100-IK-Abs-v0 --teleop_device keyboard
```

### Keyboard Controls:
```
Move arm along x-axis: W/S
Move arm along y-axis: A/D
Move arm along z-axis: Q/E
Rotate arm along x-axis: Z/X
Rotate arm along y-axis: T/G
Rotate arm along z-axis: C/V
Reset environment: R
```

### What Happens:
1. Isaac Sim window opens
2. SO-100 arm spawns on ground plane
3. Blue/green target markers appear (goal pose)
4. Keyboard controls move the end-effector
5. IK solver automatically calculates joint angles
6. Multiple joints move together (this is normal for IK!)

---

## FILE STRUCTURE SUMMARY

```
IsaacLab/
├── source/
│   ├── isaaclab_assets/
│   │   └── isaaclab_assets/
│   │       └── robots/
│   │           ├── __init__.py              [MODIFIED] Added: from .so100 import *
│   │           └── so100.py                 [CREATED] SO-100 asset config
│   │
│   └── isaaclab_tasks/
│       └── isaaclab_tasks/
│           └── manager_based/
│               └── manipulation/
│                   └── reach/
│                       └── config/
│                           ├── __init__.py  [MODIFIED] Added: from . import so100
│                           └── so100/       [CREATED] SO-100 reach configs
│                               ├── __init__.py        [CREATED] Task registration
│                               ├── joint_pos_env_cfg.py [CREATED] Joint position control
│                               └── ik_abs_env_cfg.py    [CREATED] IK control
│
└── scripts/
    └── environments/
        └── teleoperation/
            └── teleop_se3_agent.py          [EXISTING] Teleoperation script
```

---

## TROUBLESHOOTING

### Issue 1: "No module named 'omni.isaac.lab'"
**Solution:** Use `import isaaclab.sim` NOT `import omni.isaac.lab.sim`

### Issue 2: "Invalid action shape, expected: 5, received: 6"
**Solution:** Use IK task (`Isaac-Reach-SO100-IK-Abs-v0`) instead of joint position task

### Issue 3: Arm drifts downward
**Solution:** Set `disable_gravity=True` in SO100_CFG

### Issue 4: Parts penetrating each other
**Solution:** Set `enabled_self_collisions=False` if USD lacks proper collision meshes

### Issue 5: Arm oscillates/unstable
**Solution:** Increase stiffness (400) and damping (80) in actuator configs

### Issue 6: Multiple joints move when pressing one key
**Solution:** This is NORMAL for IK control - it maintains end-effector pose

---

## IMPORTANT NOTES FOR YOUTUBE TUTORIAL

1. **Joint Names Matter:** Always check USD file in Isaac Sim Stage panel for exact names
2. **End-Effector Body:** Use the tip/gripper body, not parent links
3. **IK vs Joint Control:** IK is easier for teleoperation, joint control for precise movements
4. **Gravity:** Disable for reach tasks to prevent drift
5. **Stiffness Values:** Higher values (400+) needed for IK stability
6. **Task Naming:** Must be unique, follow pattern `Isaac-<Task>-<Robot>-<Control>-v0`

---

## NEXT STEPS FOR IMITATION LEARNING

### 1. Add Data Recording to Teleop Script:
- Record observations, actions, rewards during teleoperation
- Save to pickle/HDF5 file
- Implement episode segmentation

### 2. Create Lift Task (with table + cube):
- Copy reach config structure
- Use lift base environment
- Add gripper control

### 3. Train IL Model:
- Behavioral Cloning (BC)
- Use collected demonstrations
- Test trained policy in Isaac-Reach-SO100-Play-v0

---

## TESTED AND VERIFIED
- ✅ SO-100 asset loads correctly
---

## PART 2: LIFT TASK SETUP (CUBE MANIPULATION)

After successfully setting up the reach task, now we'll create a lift task where the robot can pick up and move a cube.

### STEP 7: CREATE LIFT TASK DIRECTORY

Create a new folder for SO-100 lift configs:

```bash
mkdir -p ~/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/lift/config/so100
```

---

### STEP 8: CREATE JOINT POSITION CONFIG FOR LIFT

**File:** `lift/config/so100/joint_pos_env_cfg.py`

This file sets up the lift environment with:
- SO-100 robot configuration
- Joint position control for arm (5 joints)
- Binary gripper control (open/close)
- Cube object spawning
- End-effector tracking

**Key differences from reach task:**
1. **Gripper action added** - Binary control to open/close gripper (0.25 = open, 0.0 = closed)
2. **Cube object** - DexCube spawns in front of robot for manipulation
3. **Frame transformer** - Tracks end-effector position for rewards

---

### STEP 9: CREATE IK CONTROL CONFIG FOR LIFT

**File:** `lift/config/so100/ik_abs_env_cfg.py`

This enables keyboard teleoperation for the lift task:
- Uses relative mode IK control (required for keyboard)
- Same control scheme as reach task (WASD/QE for position, ZXTGCV for rotation)
- K key toggles gripper open/close

**Important:** `use_relative_mode=True` is critical for keyboard control to work properly!

---

### STEP 10: REGISTER LIFT TASKS

**File:** `lift/config/so100/__init__.py`

Registers 3 tasks with Gymnasium:
1. **Isaac-Lift-Cube-SO100-v0** - Joint position control (for RL training)
2. **Isaac-Lift-Cube-SO100-Play-v0** - Play mode with 50 environments
3. **Isaac-Lift-Cube-SO100-IK-Abs-v0** - IK control with keyboard (for data collection)

---

### STEP 11: ADD SO-100 TO PARENT INIT FILE

**File:** `lift/config/__init__.py`

Add this import at the bottom:

```python
from .so100 import *
```

This makes Isaac Lab aware of SO-100 lift tasks.

---

### STEP 12: UPDATE SO-100 ROBOT ASSET FOR LIFT

**File:** `isaaclab_assets/robots/so100.py`

Update the robot configuration for lift task requirements:

**Key changes:**
1. **Faster movement** - Increased stiffness=800, damping=160 (doubled from 400/80)
2. **Wider gripper** - Initial position=0.25 (was 0.15) for better cube grip
3. **Better collision** - solver_position_iteration_count=16 for accurate physics
4. **Cube interaction** - activate_contact_sensors=True enabled

**Why these values?**
- High stiffness/damping = Faster response and stable lifting
- Wide gripper opening = Easily fits around cube
- More solver iterations = Better collision detection

---

## PART 3: COLLISION SETUP (CRITICAL FOR LIFT TASK!)

**Problem:** USD files from CAD software (Fusion 360, SolidWorks, etc.) only have visual meshes, NOT collision meshes. Without collisions, the robot arm will pass through the cube!

### WHY COLLISIONS ARE NEEDED:
- Robot must physically interact with cube
- Gripper must grip the cube (not pass through it)
- Data recording requires proper object manipulation
- Physics simulation needs collision geometry

---

### STEP 13: ADD COLLISION MESHES IN ISAAC SIM 5.0

Open Isaac Sim and follow these steps:

#### A. Load USD File
1. Open Isaac Sim 5.0
2. **File → Open** → Select `~/Downloads/SO100/so100/so100.usd`

#### B. Create Collision Group
1. In **Stage** panel (left side), select the root prim `so_arm100`
2. Right-click → **Create → Physics → Colliders Preset**
3. Popup will appear - select **"Convex Hull"** option
4. Click **Apply**

This automatically generates collision meshes for all robot links!

#### C. Configure Collision Settings
1. With root prim still selected, check **Property** panel (right side)
2. Find **Physics → Collision Group** section
3. Under **Colliders**, you should see:
   - **Includes:** `/so_arm100` (already added)
   - **Expansion Rule:** `expandPrims` (automatically set)
   - **Include Root:** Checkbox should be CHECKED ✅

#### D. Save USD File
1. **File → Save** (Ctrl+S)
2. Overwrite the original USD file

#### E. Verify Collisions (Optional)
1. Top menu → **Window → Physics → Debug**
2. Enable **"Show Colliders"**
3. You should see wireframe collision meshes overlaid on robot

**Note:** Yellow warnings about "Parse collision - triangle mesh" are NORMAL! Isaac Sim automatically converts meshes to convex hulls at runtime. These warnings don't affect functionality.

---

### STEP 14: CLEAR PYTHON CACHE

After updating USD and configs, clear cache:

```bash
find ~/IsaacLab/source/isaaclab_assets/isaaclab_assets/robots -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null
find ~/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/lift -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null
```

---

### STEP 15: TEST LIFT TASK WITH KEYBOARD

Run the lift task:

```bash
cd ~/IsaacLab
./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py --task Isaac-Lift-Cube-SO100-IK-Abs-v0 --teleop_device keyboard
```

**Expected behavior:**
- ✅ Robot spawns with cube in front
- ✅ Arm moves smoothly (fast response)
- ✅ Collision works (arm stops when touching cube)
- ✅ Gripper opens wide (0.25 opening)
- ✅ K key toggles gripper
- ✅ Can pick up and move cube

---

---

## PART 4: INDEPENDENT JOINT CONTROL ⭐ NEW

After setting up IK control, you may want **true independent joint control** where each key moves only ONE joint (no coordination).

### WHY USE JOINT CONTROL INSTEAD OF IK?

**IK Control (teleop_se3_agent.py):**
- ✅ Intuitive Cartesian movement (forward, up, rotate)
- ✅ End-effector stays stable
- ❌ Multiple joints move together (coordinated)
- ❌ Action shape issues with 5-DOF arms

**Joint Position Control (teleop_joint_agent.py):**
- ✅ **Independent joint movement** (one key = one joint)
- ✅ No action shape mismatch errors
- ✅ Precise control over individual joints
- ✅ Easier for understanding robot kinematics
- ❌ Less intuitive than Cartesian control
- ❌ End-effector may drift during movement

---

### STEP 16: CREATE INDEPENDENT JOINT CONTROL SCRIPT

**File:** `scripts/environments/teleoperation/teleop_joint_agent.py`

This script was created to provide true independent joint control with keyboard.

**Key Features:**
1. **Direct Joint Position Commands** - No IK solver involved
2. **Keyboard Event Handling** - Blocks conflicting keys (F, SPACE)
3. **Position Accumulation** - Joints maintain position after key press
4. **Device Compatibility** - Automatically uses CUDA/CPU matching environment
5. **Detailed Documentation** - Comments explain every joint and key mapping

**File Location:**
```
~/IsaacLab/scripts/environments/teleoperation/teleop_joint_agent.py
```

**File already created during conversation** - See the actual file for complete code.

---

### STEP 17: IK VS JOINT POSITION CONTROL

**Understanding the Difference:**

| Aspect | IK Control | Joint Position Control |
|--------|-----------|------------------------|
| **Script** | `teleop_se3_agent.py` | `teleop_joint_agent.py` |
| **Task** | `Isaac-Lift-Cube-SO100-IK-Abs-v0` | `Isaac-Lift-Cube-SO100-v0` |
| **Action Input** | SE3 pose (7 values: x,y,z,roll,pitch,yaw,gripper) | Joint angles (6 values: 5 joints + gripper) |
| **Movement Style** | Cartesian space (move forward, rotate) | Joint space (bend elbow, rotate shoulder) |
| **Joint Coordination** | Multiple joints move together | Each joint moves independently |
| **Best For** | Intuitive teleoperation, data collection | Understanding kinematics, precise control |

**Example:**
- **IK:** Press W → End-effector moves forward (shoulder, elbow, wrist all adjust)
- **Joint:** Press W → Only shoulder_lift moves up (other joints stay still)

---

### STEP 18: CUSTOMIZE KEYBOARD MAPPINGS

**All key mappings are in one location:**

**File:** `scripts/environments/teleoperation/teleop_joint_agent.py`
**Lines:** ~95-180 (in `advance()` method)

**Current Mappings:**
```python
# JOINT 0: Shoulder Pan (base rotation)
Q = positive rotation, A = negative rotation

# JOINT 1: Shoulder Lift (arm up/down)
W = move up, S = move down

# JOINT 2: Elbow Flex (extend/retract)
E = extend, D = retract

# JOINT 3: Wrist Flex (wrist up/down)
R = up, V = down  # F changed to V (camera conflict)

# JOINT 4: Wrist Roll (rotation)
T = rotate positive, G = rotate negative

# JOINT 5: Gripper (open/close)
Z = open, X = close

# SPECIAL KEYS:
BACKSPACE = Reset environment
F = Blocked (prevents camera reset)
SPACE = Blocked (prevents simulation pause)
```

**To Change Keys:**
1. Open `teleop_joint_agent.py`
2. Find the `advance()` method (~line 95)
3. Change the key names in `self._key_state.get("KEY_NAME", False)`
4. Update the printed instructions in `__init__()` method (~line 100)

**Example - Change Wrist Flex from R/V to U/I:**
```python
# Line ~156-160
# JOINT 3: Wrist Flex
if self._key_state.get("U", False):  # Changed from R
    delta[3] = self.joint_delta
elif self._key_state.get("I", False):  # Changed from V
    delta[3] = -self.joint_delta
```

---

### RUNNING JOINT CONTROL

**Command:**
```bash
cd ~/IsaacLab
./isaaclab.sh -p scripts/environments/teleoperation/teleop_joint_agent.py \
    --task Isaac-Lift-Cube-SO100-v0 \
    --joint_delta 0.1
```

**Arguments:**
- `--task` - Must use joint position task (NOT IK task)
- `--joint_delta` - Radians to move per key press (default: 0.1)
  - `0.05` = Slow, precise (2.9° per press)
  - `0.1` = Balanced (5.7° per press) ✅ Recommended
  - `0.2` = Fast, less precise (11.4° per press)

**Expected Behavior:**
- ✅ One key press = One joint moves
- ✅ Other joints stay still (no coordination)
- ✅ Joint position holds after releasing key
- ✅ No action shape errors
- ✅ No camera reset on F press
- ✅ No pause on SPACE press
- ✅ Reset works with BACKSPACE

---

### TROUBLESHOOTING JOINT CONTROL

#### Issue 1: "Expected 6, received 7" Error
**Cause:** Using IK task with joint control script

**Solution:** Use joint position task:
```bash
# WRONG (IK task):
--task Isaac-Lift-Cube-SO100-IK-Abs-v0

# CORRECT (Joint task):
--task Isaac-Lift-Cube-SO100-v0
```

---

#### Issue 2: "Device mismatch: cuda:0 and cpu" Error
**Cause:** Already fixed in v1.2!

**Solution:** Update script if using old version - controller now auto-detects device.

---

#### Issue 3: Keys Don't Update Position
**Cause:** Already fixed in v1.2!

**Solution:** Update script - positions now accumulate properly using `current_joint_targets += delta`.

---

#### Issue 4: Reset (BACKSPACE) Not Working After Press
**Cause:** Already fixed in v1.2!

**Solution:** Update script - reset now properly reinitializes joint targets from environment.

---

#### Issue 5: Camera Resets When Pressing Keys
**Cause:** F key triggers Isaac Sim camera reset

**Solution:** Already blocked in v1.2 - F key events don't propagate to simulation.

---

#### Issue 6: Simulation Pauses Unexpectedly
**Cause:** SPACE key triggers Isaac Sim pause/play

**Solution:** Already blocked in v1.2 - SPACE key events don't propagate to simulation.

---

## KEYBOARD CONTROLS (SAME AS REACH)

### Position Control:
- **W/S** - Move forward/backward (X-axis)
- **A/D** - Move left/right (Y-axis)
- **Q/E** - Move up/down (Z-axis)

### Rotation Control:
- **Z/X** - Rotate around X-axis
- **T/G** - Rotate around Y-axis
- **C/V** - Rotate around Z-axis

### Gripper Control:
- **K** - Toggle gripper open/close

### Other:
- **R** - Reset environment
- **Esc** - Quit

---

## TROUBLESHOOTING COMMON ISSUES

### Issue 1: "Joint limits out of range" Error

**Error:** `shoulder_lift: -0.500 not in [0.000, 3.500]`

**Solution:** Check joint limits in USD file and update `init_state` in `so100.py`:
```python
"shoulder_lift": 0.5,  # Must be positive (range: 0 to 3.5)
"elbow_flex": -0.5,    # Must be negative (range: -3.14 to 0)
```

---

### Issue 2: Robot Moving Too Slow

**Problem:** Arm feels heavy and moves slowly

**Solution:** Increase stiffness and damping values in `so100.py`:
```python
stiffness=800.0,  # Higher = faster response
damping=160.0,    # Higher = more stable
```

Start with Franka values (400/80) and double if needed (800/160).

---

### Issue 3: No Collision Detection

**Problem:** Robot arm passes through cube

**Solution:** Follow STEP 13 carefully to add collision meshes in Isaac Sim. Key points:
- Use **Colliders Preset** with **Convex Hull**
- Ensure **Include Root** checkbox is checked
- Save USD file after adding collisions
- Clear Python cache before testing

**Verification:** In Isaac Sim, enable Physics Debug → Show Colliders to see collision wireframes.

---

### Issue 4: Gripper Not Opening Wide Enough

**Problem:** Gripper fingers too close together to grip cube

**Solution:** Increase gripper opening values:

In `so100.py`:
```python
"gripper": 0.25,  # Initial position (increase from 0.15)
```

In `joint_pos_env_cfg.py`:
```python
open_command_expr={"gripper": 0.25},  # Open position
```

Test with values: 0.15 → 0.25 → 0.35 until cube fits comfortably.

---

### Issue 5: Arm Drifting or Unstable

**Problem:** Arm slowly drifts down or shakes

**Solution:** 
1. Disable gravity: `disable_gravity=True` in `so100.py`
2. Increase damping: `damping=160.0` (higher damping = less oscillation)
3. Check if stiffness is too low: `stiffness=800.0`

---

### Issue 6: Cube Spawns Too Far Away

**Problem:** Cube spawns outside robot's reach

**Solution:** Adjust cube spawn position in `joint_pos_env_cfg.py`:
```python
init_state=RigidObjectCfg.InitialStateCfg(
    pos=[0.4, 0.0, 0.055],  # X=0.4 (forward), Y=0 (center), Z=0.055 (table height)
    rot=[1, 0, 0, 0]
)
```

Decrease X value (e.g., 0.4 → 0.3) to bring cube closer.

---

### Issue 7: ModuleNotFoundError for "omni.isaac.lab"

**Error:** `No module named 'omni.isaac.lab'`

**Solution:** Isaac Lab 1.0+ changed module name. Use:
```python
import isaaclab.sim as sim_utils  # Correct
# NOT: import omni.isaac.lab.sim  # Old, will fail
```

Update all imports to use `isaaclab` instead of `omni.isaac.lab`.

---

## CONFIGURATION SUMMARY

### 🔥 LATEST WORKING CONFIG (January 31, 2026)

This config successfully lifts the cube with incremental gripper control!

---

### Robot Asset Settings (`so100.py`):

**File:** `~/IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/so100.py`

```python
SO100_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/robotics-mu/Downloads/SO100/so100/so100.usd",
        activate_contact_sensors=False,  # CRITICAL: Franka uses False - prevents penetration
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,   # Same as Franka
            solver_velocity_iteration_count=0,   # Same as Franka
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        rot=(0.7071, 0, 0, 0.7071),  # 90° rotation - robot faces cube
        joint_pos={
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.5,
            "elbow_flex": -0.5,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 0.04,  # Same initial as Franka
        },
    ),
    actuators={
        "shoulder": ImplicitActuatorCfg(
            joint_names_expr=["shoulder_.*"],
            effort_limit_sim=87.0,
            velocity_limit_sim=2.175,
            stiffness=80.0,   # Same as Franka
            damping=4.0,      # Same as Franka
        ),
        "elbow": ImplicitActuatorCfg(
            joint_names_expr=["elbow_.*"],
            effort_limit_sim=87.0,
            velocity_limit_sim=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "wrist": ImplicitActuatorCfg(
            joint_names_expr=["wrist_.*"],
            effort_limit_sim=12.0,
            velocity_limit_sim=2.61,
            stiffness=80.0,
            damping=4.0,
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["gripper"],
            effort_limit_sim=200.0,  # Same as Franka
            velocity_limit_sim=0.2,
            stiffness=2e3,
            damping=1e2,
        ),
    },
)
```

---

### Lift Task Config (`joint_pos_env_cfg.py`):

**File:** `~/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/lift/config/so100/joint_pos_env_cfg.py`

```python
@configclass
class SO100CubeLiftEnvCfg(LiftEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # Set SO-100 as robot
        self.scene.robot = SO100_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # INCREMENTAL GRIPPER CONTROL - All joints in one action
        # This enables Z/X keys to move gripper incrementally (not binary open/close)
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"],
            scale=1.0,  # Direct position control
            use_default_offset=True
        )
        self.actions.gripper_action = None  # Disabled - gripper is part of arm_action

        # Cube object - Franka-like config for stability
        self.scene.object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.35, 0.0, 0.055], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                scale=(0.5, 0.5, 0.5),  # Smaller cube for SO-100's gripper
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
            ),
        )
```

---

### Key Config Values Comparison:

| Parameter | Franka | SO-100 (Working) | Notes |
|-----------|--------|------------------|-------|
| `activate_contact_sensors` | `False` | **`False`** | Prevents penetration |
| `solver_position_iteration_count` | 8 | **8** | Contact resolution |
| `solver_velocity_iteration_count` | 0 | **0** | Same as Franka |
| Actuator `stiffness` | 80 | **80** | Matched to Franka |
| Actuator `damping` | 4 | **4** | Matched to Franka |
| Gripper `stiffness` | 2000 | **2000** | Strong grip |
| Gripper `damping` | 100 | **100** | Stable grip |
| Cube `scale` | 0.8 | **0.5** | Smaller for SO-100 |
| Gripper control | Binary | **Incremental** | Z/X keys for gradual control |

---

### Quick Run Command:

```bash
cd ~/IsaacLab
./isaaclab.sh -p scripts/environments/teleoperation/teleop_joint_agent.py \
    --task Isaac-Lift-Cube-SO100-v0 \
    --joint_delta 0.01
```

### Keyboard Controls:
| Key | Action |
|-----|--------|
| Q/A | Shoulder Pan (left/right) |
| W/S | Shoulder Lift (up/down) |
| E/D | Elbow Flex (extend/retract) |
| C/V | Wrist Flex (up/down) |
| T/G | Wrist Roll (rotate) |
| **Z/X** | **Gripper (open/close) - INCREMENTAL!** |
| BACKSPACE | Reset environment |

---

### Lift Task Differences:
| Feature | Reach Task | Lift Task |
|---------|-----------|-----------|
| Object | None (just target marker) | Cube (DexCube USD) |
| Gripper | Not used | Incremental control (Z/X keys) |
| Frame Transformer | Not needed | Tracks end-effector |
| Collision | Optional | REQUIRED |
| Use Case | Position reaching | Object manipulation |

---

## FILE STRUCTURE OVERVIEW

```
IsaacLab/
├── source/
│   ├── isaaclab_assets/
│   │   └── isaaclab_assets/
│   │       └── robots/
│   │           ├── __init__.py          # Export SO100_CFG
│   │           └── so100.py             # Robot asset config
│   │
│   └── isaaclab_tasks/
│       └── isaaclab_tasks/
│           └── manager_based/
│               └── manipulation/
│                   ├── reach/
│                   │   └── config/
│                   │       ├── __init__.py  # Import so100
│                   │       └── so100/
│                   │           ├── __init__.py        # Task registration
│                   │           ├── joint_pos_env_cfg.py
│                   │           └── ik_abs_env_cfg.py
│                   │
│                   └── lift/
│                       └── config/
│                           ├── __init__.py  # Import so100
│                           └── so100/
│                               ├── __init__.py        # Task registration
│                               ├── joint_pos_env_cfg.py
│                               └── ik_abs_env_cfg.py
│
└── scripts/
    └── environments/
        └── teleoperation/
            ├── teleop_se3_agent.py      # IK control (existing)
            ├── teleop_joint_agent.py    # ⭐ NEW: Joint control
            └── SO100_INTEGRATION_GUIDE.md
```

---

## TESTING CHECKLIST

### Reach Task (Isaac-Reach-SO100-IK-Abs-v0):
- ✅ Robot spawns correctly
- ✅ Target marker visible
- ✅ Keyboard controls work (WASD/QE/ZXTGCV)
- ✅ Arm reaches target smoothly
- ✅ No drift or instability
- ✅ Environment resets properly (R key)

### Lift Task - IK Control (Isaac-Lift-Cube-SO100-IK-Abs-v0):
- ✅ Robot spawns with cube in front
- ✅ Cube has correct size and position
- ✅ Collision detection works (arm doesn't pass through cube)
- ✅ Gripper opens wide enough (K key)
- ✅ Can pick up cube successfully
- ✅ Can move cube to different positions
- ✅ Cube responds to physics (gravity, momentum)
- ✅ Fast arm movement (not slow/heavy)

### Lift Task - Joint Control (Isaac-Lift-Cube-SO100-v0) ⭐ NEW:
- ✅ Robot spawns with cube
- ✅ Each key moves only ONE joint (independent control)
- ✅ Joints maintain position after key release
- ✅ No action shape errors (6 values accepted)
- ✅ BACKSPACE resets environment properly
- ✅ F key doesn't reset camera
- ✅ SPACE key doesn't pause simulation
- ✅ No device mismatch errors (CUDA/CPU)
- ✅ Gripper opens/closes with Z/X keys
- ✅ All 5 arm joints + gripper controllable

---

## NEXT STEPS: DATA COLLECTION FOR IMITATION LEARNING

Now that both tasks work with keyboard control, you can:

1. **Record Demonstrations:**
   - Use keyboard to perform pick-and-place tasks
   - Isaac Lab can record state-action pairs
   - Save trajectories for training

2. **Train IL Model:**
   - Use collected data with Behavior Cloning (BC)
   - Or other imitation learning algorithms
   - Fine-tune on SO-100 specific motions

3. **Create Custom Tasks:**
   - Follow same pattern (copy Franka configs)
   - Replace robot with SO100_CFG
   - Update joint names and end-effector
   - Add collision meshes to USD

---

## IMPORTANT NOTES

### About USD Collision Meshes:
- **CAD exports (Fusion 360, SolidWorks) only include visual geometry**
- You MUST add collision meshes manually in Isaac Sim
- Without collisions, objects pass through each other
- Collision setup is ONE-TIME process per USD file

### About Performance:
- Higher stiffness/damping = Faster but less smooth
- Lower values = Smoother but slower response
- Balance based on your robot's real-world behavior
- SO-100 uses 800/160 for fast demonstrations

### About Gripper Values:
- Opening values depend on gripper joint limits in USD
- Test different values: 0.15, 0.25, 0.35
- Too wide = unstable, too narrow = can't grip cube
- 0.25 works well for standard cube size

---

## CONTACT & CREDITS

**Author:** Milan Jani
**Integration Date:** January 24, 2026
**Isaac Lab Version:** 1.0+ (Isaac Sim 5.0+)

This guide was created for educational purposes and YouTube tutorial content.

For more information:
- Isaac Lab Documentation: https://isaac-sim.github.io/IsaacLab/
- Isaac Sim Physics: https://docs.omniverse.nvidia.com/extensions/latest/ext_physics.html

---

## FINAL SUCCESS CRITERIA

✅ **Reach Task Working** - Robot reaches target positions smoothly
✅ **Lift Task Working** - Robot picks up and moves cube successfully  
✅ **Collisions Enabled** - Arm interacts with cube properly
✅ **Keyboard Control** - All keys respond correctly (WASD/QE/ZXTGCV/K)
✅ **Fast Movement** - Arm responds quickly without lag
✅ **Wide Gripper** - Gripper opens enough to grip cube
✅ **Ready for Data Collection** - Can demonstrate pick-and-place tasks

**Congratulations! Your SO-100 arm is now fully integrated with Isaac Lab! 🎉**

---

## FUTURE IMPROVEMENTS & NEXT STEPS

### 🔧 PERFORMANCE OPTIMIZATION (TODO)

#### 1. Speed Improvements
**Current Status:** SO-100 moves slower than Franka robot

**Possible Solutions:**
- **Test higher stiffness values:** Try 1000, 1200, or even 1600 (current: 800)
- **Increase velocity limits:** Check `velocity_limit_sim` in actuator config
- **Reduce IK scale:** Lower `scale` value in IK action (try 0.3 or 0.2 instead of 0.5)
- **Check USD joint properties:** Ensure no artificial damping in USD file itself
- **Profile real SO-100:** Measure actual robot speed and match in simulation

**Testing Command:**
```bash
# Test different speeds by editing so100.py and clearing cache
find ~/IsaacLab/source/isaaclab_assets -name "__pycache__" -exec rm -rf {} + 2>/dev/null
```

---

#### 2. Arm Initial Pose & Cube Spawn Position
**Current Issue:** Sometimes cube spawns outside arm's reach

**Root Cause:** SO-100 USD has different default pose than Franka (which points forward)

**Solution A: Fix in USD File (Recommended)**
1. Open SO-100 USD in Isaac Sim
2. Adjust initial joint positions to point arm forward
3. Save USD with new default pose
4. Update `so100.py` init_state to match

**Solution B: Adjust Cube Spawn Position**
In `lift/config/so100/joint_pos_env_cfg.py`:
```python
# Experiment with different positions
init_state=RigidObjectCfg.InitialStateCfg(
    pos=[0.35, 0.1, 0.055],  # Try: X=0.3-0.4, Y=-0.2 to 0.2
    rot=[1, 0, 0, 0]
)
```

**Solution C: Randomize Cube Spawn**
Add spawn randomization within arm's workspace:
```python
# In events configuration
cube_spawn_range = {"x": (0.25, 0.45), "y": (-0.15, 0.15), "z": (0.055, 0.055)}
```

---

### 🎓 IMITATION LEARNING DATA COLLECTION (TODO)

#### Step 1: Record Demonstrations
**Goal:** Collect expert demonstrations using keyboard control

**Method A: Using Isaac Lab's Built-in Recorder**
```bash
# Record demonstrations (if available in Isaac Lab)
./isaaclab.sh -p scripts/tools/record_demonstrations.py \
    --task Isaac-Lift-Cube-SO100-IK-Abs-v0 \
    --num_demos 100 \
    --save_path ~/so100_demos/
```

**Method B: Manual Recording Script**
Create custom script to save state-action pairs:
```python
# Save: robot_state, action, cube_position, gripper_state
# Format: HDF5 or pickle for efficient storage
```

**What to Record:**
- Joint positions (6 values)
- Joint velocities (6 values)
- End-effector pose (position + orientation)
- Gripper state (open/close)
- Cube position & orientation
- Action taken (IK command)

**Recommended Dataset Size:**
- Training: 500-1000 demonstrations
- Validation: 100-200 demonstrations
- Each demo = 100-300 timesteps (pick → place → success)

---

#### Step 2: Data Annotation & Quality Check
**Tasks:**
1. **Label Success/Failure:** Mark which demos successfully picked and placed cube
2. **Filter Bad Demos:** Remove demonstrations where:
   - Cube fell off table
   - Gripper missed the cube
   - Robot collided violently
   - Task timeout occurred

3. **Add Task Labels:** Annotate different behaviors:
   - `pick_from_left`
   - `pick_from_right`
   - `place_high`
   - `place_low`

**Tools:**
- Use Isaac Lab's visualization tools
- Create simple GUI for demo playback and labeling
- Save metadata JSON alongside trajectories

---

#### Step 3: Train Imitation Learning Model
**Recommended Algorithm:** Behavior Cloning (BC) - simplest to start

**Framework Options:**
1. **Stable Baselines3** (with BC extension)
2. **Imitation library** (built on SB3)
3. **RoboMimic** (specifically for robot manipulation)

**Basic Training Pipeline:**
```bash
# 1. Preprocess data
python scripts/il/preprocess_demos.py --input ~/so100_demos/ --output ~/so100_processed/

# 2. Train BC model
python scripts/il/train_bc.py \
    --data ~/so100_processed/ \
    --task Isaac-Lift-Cube-SO100-v0 \
    --epochs 100 \
    --batch_size 256

# 3. Evaluate trained policy
python scripts/il/eval_policy.py \
    --checkpoint ~/models/bc_so100_best.pth \
    --task Isaac-Lift-Cube-SO100-v0 \
    --num_episodes 50
```

**Training Considerations:**
- Use data augmentation (add noise to states)
- Try different network architectures (MLP vs Transformer)
- Monitor overfitting (train vs validation loss)
- Experiment with learning rate (1e-4 to 1e-3)

---

#### Step 4: Fine-Tuning with RL (Optional)
After BC pre-training, improve with Reinforcement Learning:

**Algorithms to Try:**
- PPO (Proximal Policy Optimization)
- SAC (Soft Actor-Critic)
- TD3 (Twin Delayed DDPG)

**Why Fine-tune?**
- BC learns average behavior (may fail on novel situations)
- RL can explore better strategies
- Combining BC + RL often gives best results

---

### 📊 EXPERIMENT TRACKING (TODO)

**Use Weights & Biases or TensorBoard:**
```bash
# Track metrics:
- Success rate over time
- Average episode length
- Grasp stability
- Task completion time
- Collision frequency
```

---

### 🔬 ADVANCED FEATURES (TODO)

#### 1. Multi-Object Manipulation
- Add multiple cubes with different colors/shapes
- Task: Sort objects by color
- Requires object detection and classification

#### 2. Dynamic Obstacles
- Add moving obstacles in workspace
- Teach arm to avoid collisions while manipulating

#### 3. Sim-to-Real Transfer
- Domain randomization (vary lighting, friction, mass)
- Train in Isaac Lab, deploy on real SO-100
- Collect real-world data for fine-tuning

#### 4. Vision-Based Control
- Add camera sensor to robot
- Use RGB/depth images as input instead of state
- Train end-to-end visuomotor policies

---

### 📝 KNOWN LIMITATIONS & WORKAROUNDS

#### Limitation 1: Speed Not Matching Franka
**Status:** Under investigation
**Workaround:** Use current speed for data collection, optimize later

#### Limitation 2: Cube Spawn Position
**Status:** Needs USD pose adjustment
**Workaround:** Manually adjust spawn coordinates per environment reset

#### Limitation 3: Collision Approximation Warnings
**Status:** Normal Isaac Sim behavior
**Impact:** None - collisions work correctly despite warnings

---

### 🎯 PRIORITY TASK LIST

**High Priority (Do Next):**
1. ✅ ~~Integrate reach task~~ - DONE
2. ✅ ~~Integrate lift task~~ - DONE
3. ✅ ~~Fix collision detection~~ - DONE
4. 🔲 Record 100+ demonstrations using keyboard
5. 🔲 Annotate and filter quality demonstrations
6. 🔲 Train initial BC model

**Medium Priority:**
1. 🔲 Optimize arm movement speed (test higher stiffness)
2. 🔲 Fix cube spawn position (adjust USD or config)
3. 🔲 Add spawn randomization for robustness
4. 🔲 Create data visualization tools

**Low Priority (Future Work):**
1. 🔲 Add camera-based observations
2. 🔲 Multi-object tasks
3. 🔲 Sim-to-real transfer experiments
4. 🔲 Deploy trained policy on real SO-100

---

### 📚 RECOMMENDED RESOURCES

**Isaac Lab Documentation:**
- Main Docs: https://isaac-sim.github.io/IsaacLab/
- Manipulation Tasks: https://isaac-sim.github.io/IsaacLab/source/features/environments.html
- Teleoperation: https://isaac-sim.github.io/IsaacLab/source/features/teleoperation.html

**Imitation Learning:**
- RoboMimic Paper: https://robomimic.github.io/
- Behavior Cloning Tutorial: https://imitation.readthedocs.io/
- Isaac Lab RL Examples: Check `source/standalone/workflows/` in Isaac Lab

**Community:**
- Isaac Sim Forums: https://forums.developer.nvidia.com/c/isaac-sim/
- Isaac Lab GitHub: https://github.com/isaac-sim/IsaacLab/

---

## 🎬 YOUTUBE TUTORIAL OUTLINE (SUGGESTED)

**Video 1: Introduction & Setup (10-15 min)**
- What is Isaac Lab and why use it
- SO-100 arm overview
- Installation and prerequisites
- File structure explanation

**Video 2: Reach Task Integration (15-20 min)**
- Creating robot asset config
- Setting up reach environment
- Registering tasks
- Testing with keyboard control

**Video 3: Lift Task & Collision Setup (20-25 min)**
- Lift task configuration
- Adding collision meshes in Isaac Sim
- Troubleshooting common issues
- Performance tuning (speed, gripper)

**Video 4: Data Collection for IL (15-20 min)**
- Recording demonstrations
- Quality checking and annotation
- Data preprocessing
- Best practices

**Video 5: Training Imitation Learning Model (20-25 min)**
- Setting up training pipeline
- Behavior Cloning basics
- Training and evaluation
- Deploying trained policy

---

## 🔧 QUICK CONFIGURATION REFERENCE

### Key Parameters (v1.1)

**Robot Orientation:**
```python
# In: isaaclab_assets/robots/so100.py
init_state=ArticulationCfg.InitialStateCfg(
    rot=(0.7071, 0, 0, 0.7071),  # Robot faces LEFT (towards cube)
```

**Keyboard Speed:**
```python
# In: lift/config/so100/ik_abs_env_cfg.py
scale=2,  # 3x faster than default (was 0.5)
```

**Cube Spawn Range:**
```python
# In: lift/config/so100/joint_pos_env_cfg.py
self.events.reset_object_position.params["pose_range"] = {
    "x": (-0.05, 0.05),   # Forward/backward: ±5cm
    "y": (-0.10, 0.10),   # Left/right: ±10cm
    "z": (0.0, 0.0),      # Table height
}
```

**Gripper Settings:**
```python
# In: lift/config/so100/joint_pos_env_cfg.py
open_command_expr={"gripper": 0.08},   # Fully open
close_command_expr={"gripper": 0.0},   # Fully closed

# In: isaaclab_assets/robots/so100.py
"gripper": 0.08,  # Initial state: fully open
```

**Actuator Speeds:**
```python
# In: isaaclab_assets/robots/so100.py
stiffness=800.0,  # High for fast response
damping=160.0,    # High for stability
```

---

## 📁 IMPORTANT FILE LOCATIONS

**⚠️ Common Mistake:** Two so100.py files exist, but only ONE is used!

```
❌ NOT USED:
/source/extensions/isaaclab.assets/isaaclab_assets/so100.py

✅ ACTUAL FILE (Edit this one):
/source/isaaclab_assets/isaaclab_assets/robots/so100.py
```

**Always check which file is loading:**
```python
import isaaclab_assets
print(isaaclab_assets.__file__)  # Shows actual import path
```

---

## VERSION HISTORY

**v1.0 - January 24, 2026 (Milan Jani)**
- Initial integration of SO-100 reach task
- Added lift task with cube manipulation
- Fixed collision detection in USD
- Documented complete setup process
- Added keyboard teleoperation support

**v1.1 - January 27, 2026 (Milan Jani)**
- ✅ Fixed robot rotation to face cube (LEFT orientation)
- ✅ Increased keyboard control speed 3x (scale: 0.5 → 1.5)
- ✅ Reduced cube spawn range for reliable reaching
- ✅ Updated gripper opening range (0.08 max)
- ✅ Fixed file structure confusion (2 so100.py files issue)

**v1.3 - February 21, 2026 (Milan Jani)**
- ✅ Gripper jaw delay fixed: `velocity_limit_sim` raised from `0.2` → `7.5` in `so100.py`
- ✅ Wrist roll hardcoded calibration: `WRIST_ROLL_RAW_CENTER=2046`, `WRIST_ROLL_RAW_HALF=2046` — no longer depends on 8-sec sweep
- ✅ Sim joint limits printed at startup for easy debugging

**v1.2 - February 19, 2026 (Milan Jani)**
- ✅ Added `teleop_so100_leader.py` — physical leader arm mirrors simulation
- ✅ Raw pyserial protocol (no scservo_sdk dependency)
- ✅ Auto-range calibration: 8-second timed sweep per joint
- ✅ Dedicated 2-point gripper calibration with saved values
- ✅ Fixed position offset: subtracted `default_joint_pos` to cancel `use_default_offset=True`
- ✅ Per-joint deadband `[4,4,4,4,4,1]` — gripper near-instant
- ✅ EMA smoothing replaced with deadband-only (zero lag)
- ✅ Serial disconnect resilience: holds last position, warns, keeps running
- ✅ Real-time side-by-side terminal table (leader vs sim, no scroll)
- ✅ Backspace keyboard handler via `carb`/`omni.appwindow`
- ✅ Shell alias `teleop_so100` added to `~/.bash_aliases`
- ✅ SERIAL_TIMEOUT reduced to 1ms, INTER_READ_SLEEP to 0.5ms

---

## PART 5: PHYSICAL LEADER ARM TELEOPERATION

---

## STEP 19: LEADER ARM TELEOPERATION OVERVIEW

### What it does
`teleop_so100_leader.py` reads joint positions from a physical SO-100 leader arm via USB serial and sends them directly to the Isaac Lab simulation. Every joint in the simulation mirrors the physical arm in real time with near-zero difference.

### Architecture
```
Physical SO-100 Leader Arm
        │
        │  USB (Feetech STS3215 servo bus @ 1Mbaud)
        │
  SO100LeaderController
    ├─ _read_raw_positions()   ← raw 0-4095 encoder counts
    ├─ deadband filter          ← ignore noise < DEADBAND_COUNTS per joint
    ├─ normalize [-1, +1]       ← using calibrated raw_center + raw_half
    └─ map to sim range          ← sim_center + sign × norm × sim_half
        │
   action = leader_pos - default_joint_pos   ← offset correction
        │
  Isaac Lab env.step(action)
        │
  sim_joint_pos ≈ leader_pos   (Diff ≈ 0.000)
```

### Key insight — why `action ≠ leader_pos`
The SO-100 lift task uses `use_default_offset=True` in `JointPositionActionCfg`.
Isaac Lab internally computes:
```
sim_pos = default_joint_pos + action
```
So sending `action = leader_pos` gives `sim_pos = default + leader` — wrong.
Fix: `action = leader_pos - default_joint_pos` ⇒ `sim_pos = leader_pos` exactly.

---

## STEP 20: HARDWARE SETUP

### Requirements
- SO-100 leader arm connected via USB
- `pyserial` installed: `pip install pyserial`
- Port: `/dev/ttyACM0` (default) — check with `ls /dev/ttyACM*`

### Check port & permissions
```bash
ls -la /dev/ttyACM*
sudo chmod 666 /dev/ttyACM0   # if permission denied
```

### If USB disconnects mid-session
```bash
fuser /dev/ttyACM0     # shows which process holds the port
fuser -k /dev/ttyACM0  # kills stale process
```

### Run command
```bash
cd ~/IsaacLab && ./isaaclab.sh -p scripts/environments/teleoperation/teleop_so100_leader.py \
    --task Isaac-Lift-Cube-SO100-v0 --port /dev/ttyACM0
```

### Shell alias (run from anywhere)
Add to `~/.bash_aliases`:
```bash
alias teleop_so100='cd ~/IsaacLab && ./isaaclab.sh -p scripts/environments/teleoperation/teleop_so100_leader.py --task Isaac-Lift-Cube-SO100-v0'
```
Activate: `source ~/.bashrc`

---

## STEP 21: CALIBRATION SYSTEM

### Phase 1 — Full arm range (8 seconds, automatic)
At startup, after physics warmup:
```
CALIBRATION — Move ALL joints through their FULL range
You have 8 seconds. Move each joint fully in both directions.
  6.2s left | observed range (counts): [823, 756, 901, 612, 788, 445]
```
For each joint, the script records the min and max raw encoder values seen.
These determine `raw_center` and `raw_half` for the linear mapping.

**Guard:** If a joint barely moved (<50 counts), a default ±1024-count range is used and a warning is printed.

### Phase 2 — Gripper (saved values, no prompt)
Gripper calibration is saved in code:
```python
GRIPPER_RAW_CLOSED = 2027   # raw when jaw fully closed
GRIPPER_RAW_OPEN   = 3172   # raw when jaw fully open
```
To re-calibrate: set both to `None` — interactive prompts will appear.

The sign direction is auto-detected:
- `raw_open > raw_closed` → `JOINT_SIGNS[5] = +1`
- `raw_open < raw_closed` → `JOINT_SIGNS[5] = -1`

### Position mapping formula
```
normalized = clip((raw - raw_center) / raw_half, -1, +1)
sim_pos    = sim_center + JOINT_SIGNS[i] × normalized × sim_half
action     = sim_pos - default_joint_pos
```

---

## STEP 22: POSITION MAPPING & OFFSET CORRECTION

### Sim joint limits are read from the environment
```python
joint_limits  = env.unwrapped.scene["robot"].data.joint_limits[0, :NUM_JOINTS, :]
sim_joint_min = joint_limits[:, 0].cpu().numpy()
sim_joint_max = joint_limits[:, 1].cpu().numpy()
leader.set_sim_limits(sim_joint_min, sim_joint_max)
```
This ensures the leader arm's full physical range maps **exactly** to the sim's full joint range — no hardcoded constants.

### Action offset correction
```python
default_joint_pos = env.unwrapped.scene["robot"].data.default_joint_pos[0, :NUM_JOINTS]
action = leader_joints - default_joint_pos
```

---

## STEP 23: RESPONSE TIME TUNING

All timing constants are in the configuration block at the top of `teleop_so100_leader.py`:

```python
# Minimum encoder count change required to pass through to sim
# Lower = faster, Higher = smoother
DEADBAND_COUNTS = [4, 4, 4, 4, 4, 1]  # index 5 = gripper = near-instant

# Max wait time per servo serial response
# Lower = faster loop, but more missed reads
SERIAL_TIMEOUT = 0.001  # 1ms

# Gap between reading consecutive motors (prevents bus collision)
INTER_READ_SLEEP = 0.0005  # 0.5ms
```

**Total per-loop serial overhead:**
- 6 motors × 1ms timeout + 5 gaps × 0.5ms = ~8.5ms maximum

**If arm still feels laggy:** lower `SERIAL_TIMEOUT` to `0.0005` and `INTER_READ_SLEEP` to `0.0002`.
**If arm is jittery:** raise `DEADBAND_COUNTS` arm joints from `4` to `6` or `8`.

---

## STEP 24: REAL-TIME TERMINAL DISPLAY

Every 10 simulation steps, the terminal overwrites in-place (no scroll):

```
  Step    340  |    Raw |   Norm | Leader(rad)  |   Sim(rad) |     Diff
  ------------------------------------------------------------------
  Shoulder Pan  |  1986 | -0.061 |     -0.1211  |    -0.1211 |  +0.0000
  Shoulder Lift |   948 | -1.000 |     +0.0000  |    +0.0000 |  +0.0000
  Elbow Flex    |  3021 | +0.950 |     -0.0782  |    -0.0782 |  +0.0000
  Wrist Flex    |  2245 | +0.192 |     -0.2941  |    -0.2941 |  +0.0000
  Wrist Roll    |  1025 | -0.999 |     -3.1385  |    -3.1385 |  +0.0000
  Gripper/Jaw   |  2027 | -0.021 |     +0.8774  |    +0.8774 |  +0.0000
  ------------------------------------------------------------------
  BACKSPACE = reset  |  Ctrl+C = quit
```

**Columns:**
- `Raw` — encoder count from servo (0–4095)
- `Norm` — normalized position (-1 to +1)
- `Leader(rad)` — what was sent to the sim as target
- `Sim(rad)` — actual sim joint position read back
- `Diff` — error; ideally 0.000 on all joints

---

## STEP 25: SHELL ALIAS SETUP

The alias is stored in `~/.bash_aliases` (sourced by `~/.bashrc`):

```bash
# Add manually:
nano ~/.bash_aliases
# Paste:
alias teleop_so100='cd ~/IsaacLab && ./isaaclab.sh -p scripts/environments/teleoperation/teleop_so100_leader.py --task Isaac-Lift-Cube-SO100-v0'
# Save: Ctrl+O → Enter → Ctrl+X
source ~/.bashrc
```

Usage:
```bash
teleop_so100                      # default port /dev/ttyACM0
teleop_so100 --port /dev/ttyUSB0  # custom port
```
- ✅ Added collision mesh setup in Isaac Sim 5.0
- 🔲 Speed optimization ongoing (target: match Franka)
- 🔲 IL data collection pipeline (next priority)

**v1.2 - January 28, 2026 (Milan Jani)**
- ✅ **Created Independent Joint Position Control Script** (`teleop_joint_agent.py`)
  - True independent joint control (each key controls ONE joint)
  - No IK coordination - pure joint-by-joint movement
  - Fixes "Expected 6, received 7" action shape errors
  - Customizable key mappings with detailed documentation
- ✅ **Fixed Device Mismatch Issues**
  - Resolved CPU vs CUDA tensor errors
  - Controller now uses environment's device automatically
- ✅ **Keyboard Event Handling Improvements**
  - Blocked F key to prevent camera reset conflicts
  - Blocked SPACE key to prevent simulation pause
  - Fixed reset functionality (BACKSPACE key)
  - Proper event attribute safety checks
- ✅ **Joint Position Accumulation**
  - Fixed "move and return" issue
  - Joints now maintain position after key press
  - Targets accumulate properly across frames
- ✅ **Complete Documentation in Script**
  - Line-by-line key mapping reference
  - Joint index to body part mapping (delta[0-5])
  - Easy customization guide for key changes

---

**End of Guide - Happy Robot Learning! 🤖🚀**

---
