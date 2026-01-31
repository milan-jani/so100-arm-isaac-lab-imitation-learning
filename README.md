# SO-100 Robot Integration for IsaacLab

Complete configuration files for integrating SO-100 robot with IsaacLab for cube manipulation tasks.

## 🚀 Quick Start

```bash
cd ~/IsaacLab
./isaaclab.sh -p scripts/environments/teleoperation/teleop_joint_agent.py \
    --task Isaac-Lift-Cube-SO100-v0 \
    --joint_delta 0.01
```

**Note**: Camera is automatically enabled (no `--enable_cameras` flag needed)

## 🎮 Keyboard Controls

| Key | Action |
|-----|--------|
| Q/A | Shoulder Pan +/- |
| W/S | Shoulder Lift +/- |
| E/D | Elbow Flex +/- |
| R/F | Wrist Flex +/- |
| T/G | Wrist Roll +/- |
| **Z/X** | **Gripper Close/Open (Incremental)** |
| BACKSPACE | Reset Environment |

## 📁 Folder Structure

```
IL_MJ/
├── README.md                           # This file
├── CHANGELOG.md                        # Update history
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
│           └── joint_pos_env_cfg.py    # Joint position config + Camera
├── reach/
│   └── config/
│       ├── __init__.py                 # Reach task registration
│       └── so100/
│           ├── __init__.py
│           ├── ik_abs_env_cfg.py
│           └── joint_pos_env_cfg.py
└── teleoperation/
    └── teleop_joint_agent.py           # Keyboard teleop (Camera auto-enabled)
```

## ✨ Key Features

### �� Vision-Based Learning
- **Wrist Camera**: RGB + Depth, 640x480
- **Position**: Mounted on gripper at `(0.1349, -0.0068, -0.03398)`
- **Auto-Enabled**: No command-line flag needed

### 🎲 Domain Randomization
- **Lighting**: Intensity randomization (500-2000)
- **Physics**: Cube friction randomization (0.5-1.5)
- **Purpose**: Improved sim-to-real transfer

### 🔧 Incremental Gripper
- **Z/X Keys**: Smooth open/close control
- **Proportional**: Movement matches key press duration
- **6-DOF Control**: All joints in single action space

### 🏎️ Stable Physics
- **Franka-Like Config**: `activate_contact_sensors=False`
- **Solver Iterations**: 8 position, 0 velocity
- **Stiffness/Damping**: 80.0 / 4.0 (arm), 2e3 / 1e2 (gripper)

## 📚 Documentation

See [SO100_INTEGRATION_GUIDE.md](SO100_INTEGRATION_GUIDE.md) for:
- Detailed setup instructions
- File-by-file explanation
- Troubleshooting guide
- Training workflow

## 📝 Latest Updates

See [CHANGELOG.md](CHANGELOG.md) for complete update history.

**Latest (2026-01-31)**:
- ✅ Added wrist camera integration
- ✅ Added domain randomization
- ✅ Camera auto-enabled in teleop
- ✅ Incremental gripper control

## 🔗 Quick Links

- [IsaacLab Documentation](https://isaac-sim.github.io/IsaacLab)
- [Isaac Sim Download](https://developer.nvidia.com/isaac-sim)
