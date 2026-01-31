# Changelog - SO-100 IsaacLab Integration

## [Latest Update] - 2026-01-31

### Added
- **Wrist Camera Integration** 📷
  - RGB + Depth camera on gripper (`jaw/Camera`)
  - Resolution: 640x480
  - Position: `(0.1349, -0.0068, -0.03398)`
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

## [Previous] - 2026-01-30

### Added
- Initial SO-100 integration with Isaac Lab
- Franka-like physics configuration for stability
- Keyboard teleoperation support
- Lift and Reach task environments

### Fixed
- Cube slip/grip issues
- Physics solver iterations tuning
- Gripper control from binary to incremental
