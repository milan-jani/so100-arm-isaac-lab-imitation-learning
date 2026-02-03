# Changelog - SO-100 IsaacLab Integration

## [Latest Update] - 2026-02-03

### Changed
- **Camera Mount Location** 📷
  - Moved from `jaw/Camera` to `wrist/Camera` for stability
  - Camera no longer rotates with gripper (T/G keys)
  - New position: `(0.0, 0.1, 0.2)` - 10cm sideways, 20cm forward
  
- **Camera Orientation Optimized** 🎯
  - Target angles: X=-53°, Y=5°, Z=185°
  - Quaternion: `(-0.019555, 0.058444, 0.443646, 0.894081)`
  - Provides optimal workspace view for manipulation

### Files Updated
- `lift/config/so100/joint_pos_env_cfg.py` - Wrist camera with optimized orientation

---

## [Update] - 2026-01-31

### Added
- **Wrist Camera Integration** 📷
  - RGB + Depth camera on gripper
  - Resolution: 640x480
  - Auto-enabled in teleop script (no `--enable_cameras` flag needed)
  
- **Domain Randomization** 🎲
  - Lighting randomization: intensity range 500-2000
  - Cube material randomization: friction 0.5-1.5
  - Purpose: Improved sim-to-real transfer

- **Incremental Gripper Control** 🔧
  - Z/X keys for continuous gripper open/close
  - Movement proportional to key press duration
  - 6 joints in single `JointPositionActionCfg`

### Changed
- Camera permanently enabled in `teleop_joint_agent.py`
- Removed need for `--enable_cameras` command-line flag

### Files Updated
- `lift/config/so100/joint_pos_env_cfg.py` - Camera + randomization
- `teleoperation/teleop_joint_agent.py` - Auto camera enable

---

## [Initial Release] - 2026-01-30

### Added
- Initial SO-100 integration with Isaac Lab
- Franka-like physics configuration for stability
- Keyboard teleoperation support
- Lift and Reach task environments

### Fixed
- Cube slip/grip issues
- Physics solver iterations tuning
- Gripper control from binary to incremental
