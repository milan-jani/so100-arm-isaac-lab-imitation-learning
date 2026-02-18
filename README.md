# SO-100 Robot Integration for IsaacLab

Complete configuration files for integrating SO-100 robot with IsaacLab for cube manipulation tasks.

## 🚀 Quick Start

### Keyboard Teleoperation
```bash
cd ~/IsaacLab
./isaaclab.sh -p scripts/environments/teleoperation/teleop_joint_agent.py \
    --task Isaac-Lift-Cube-SO100-v0 \
    --joint_delta 0.01
```

### Leader Arm Teleoperation (Physical SO-100) ⭐ NEW
```bash
cd ~/IsaacLab
./isaaclab.sh -p scripts/environments/teleoperation/teleop_so100_leader.py \
    --task Isaac-Lift-Cube-SO100-v0
```

**Note**: Camera is automatically enabled (no `--enable_cameras` flag needed)

## 🎮 Keyboard Controls

| Key | Action |
|-----|--------|
| Q/A | Shoulder Pan +/- |
| W/S | Shoulder Lift +/- |
| E/D | Elbow Flex +/- |
| C/V | Wrist Flex +/- |
| T/G | Wrist Roll +/- |
| **Z/X** | **Gripper Open/Close (Incremental)** |
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
│           └── joint_pos_env_cfg.py    # Joint position config (camera in USD)
├── reach/
│   └── config/
│       ├── __init__.py                 # Reach task registration
│       └── so100/
│           ├── __init__.py
│           ├── ik_abs_env_cfg.py
│           └── joint_pos_env_cfg.py
└── teleoperation/
    ├── teleop_joint_agent.py           # Keyboard teleop (Camera auto-enabled)
    └── teleop_so100_leader.py          # Physical SO-100 leader arm teleop ⭐ NEW
```

## ✨ Key Features

### �� Vision-Based Learning
- **Wrist Camera**: RGB + Depth, defined directly in USD file
- **Position**: Mounted on gripper at `(-0.18599, -0.00463, -0.05317)`, Euler `(-88°, -72°, 91°)`
- **Focal Length**: 14.0 — wide-angle view showing cube from above and gripper jaws
- **Auto-Enabled**: No command-line flag needed

### 🦾 Leader Arm Teleoperation ⭐ NEW
- **Physical SO-100**: Mirror real leader arm joints directly into sim
- **Serial Protocol**: Pure pyserial, Feetech STS raw packets, `/dev/ttyACM0`, 1 Mbaud
- **6-DOF**: Reads all 6 motor positions (IDs 1–6), maps to sim joint space
- **No SDK Required**: Works with standard pyserial (no scservo_sdk needed)

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
- **Solver Iterations**: 16 position, 1 velocity
- **Stiffness/Damping**: 80.0 / 4.0 (arm), 2e3 / 1e2 (gripper)

## 📚 Documentation

See [SO100_INTEGRATION_GUIDE.md](SO100_INTEGRATION_GUIDE.md) for:
- Detailed setup instructions
- File-by-file explanation
- Troubleshooting guide
- Training workflow

## 📝 Latest Updates

See [CHANGELOG.md](CHANGELOG.md) for complete update history.

**Latest (2026-02-18)**:
- ✅ Added physical SO-100 leader arm teleoperation (`teleop_so100_leader.py`)
- ✅ Moved camera config to USD file (cleaner Python code)
- ✅ Pure pyserial Feetech STS protocol — no external SDK needed
- ✅ Confirmed working on `/dev/ttyACM0` (QinHeng CH343 USB chip)

## 🔗 Quick Links

- [IsaacLab Documentation](https://isaac-sim.github.io/IsaacLab)
- [Isaac Sim Download](https://developer.nvidia.com/isaac-sim)
