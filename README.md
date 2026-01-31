# SO-100 Robot Integration for IsaacLab

Complete configuration files for integrating SO-100 robot with IsaacLab for cube manipulation tasks.

## Quick Start

```bash
cd ~/IsaacLab
./isaaclab.sh -p scripts/environments/teleoperation/teleop_joint_agent.py \
    --task Isaac-Lift-Cube-SO100-v0 \
    --joint_delta 0.01
```

## Keyboard Controls

| Key | Action |
|-----|--------|
| Q/A | Shoulder Pan +/- |
| W/S | Shoulder Lift +/- |
| E/D | Elbow Flex +/- |
| R/F | Wrist Flex +/- |
| T/G | Wrist Roll +/- |
| Z/X | Gripper Close/Open |
| BACKSPACE | Reset Environment |

## Folder Structure

```
IL_MJ/
├── README.md                           # This file
├── SO100_INTEGRATION_GUIDE.md          # Detailed integration guide
├── isaaclab_assets/
│   └── robots/
│       └── so100.py                    # Robot asset config
├── lift/
│   └── config/
│       ├── __init__.py                 # Lift task registration
│       └── so100/
│           ├── __init__.py
│           ├── ik_abs_env_cfg.py       # IK control config
│           └── joint_pos_env_cfg.py    # Joint position config
├── reach/
│   └── config/
│       ├── __init__.py                 # Reach task registration
│       └── so100/
│           ├── __init__.py
│           ├── ik_abs_env_cfg.py
│           └── joint_pos_env_cfg.py
└── teleoperation/
    └── teleop_joint_agent.py           # Keyboard teleop script
```

## Key Configuration

- **SO-100 Robot**: 6-DOF (5 arm + 1 gripper)
- **Physics**: Franka-like settings for stable manipulation
- **Gripper**: Incremental control (NOT binary)

See [SO100_INTEGRATION_GUIDE.md](SO100_INTEGRATION_GUIDE.md) for detailed setup instructions.
