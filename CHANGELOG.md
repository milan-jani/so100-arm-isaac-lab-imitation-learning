# Changelog - SO-100 IsaacLab Integration

## [v1.3] - 2026-02-21

### Fixed
- **Gripper jaw delay** — `velocity_limit_sim` raised from `0.2` → `7.5` in `so100.py`; physics engine was capping jaw speed at Franka's linear finger rate
- **Wrist Roll one-direction block** — hardcoded `WRIST_ROLL_RAW_CENTER=2046`, `WRIST_ROLL_RAW_HALF=2046` from confirmed good sweep; 8-sec calibration no longer overwrites wrist_roll values

### Files Updated
- `isaaclab_assets/robots/so100.py` — gripper velocity limit fix
- `teleoperation/teleop_so100_leader.py` — wrist roll hardcoded calibration, sim joint limits printed at startup

---

## [v1.2] - 2026-02-19

### Added
- **Physical Leader Arm Teleoperation** 🦾
  - `teleop_so100_leader.py` — mirrors physical SO-100 leader arm to sim in real time
  - Runs via: `./isaaclab.sh -p scripts/environments/teleoperation/teleop_so100_leader.py --task Isaac-Lift-Cube-SO100-v0`
  - Servo protocol: Feetech STS3215 raw pyserial at 1Mbaud on `/dev/ttyACM0`

- **Auto-Calibration System**
  - `calibrate_range()`: 8-second timed sweep — move all joints through full range, records real min/max per joint
  - `calibrate_gripper()`: 2-point closed/open measurement with auto-sign detection; values saved as constants (`GRIPPER_RAW_CLOSED=2027`, `GRIPPER_RAW_OPEN=3172`)
  - `set_sim_limits()`: reads actual `joint_limits` from env, computes `sim_center + sim_half` for linear mapping
  - Linear mapping formula: `sim_pos = sim_center + sign × ((raw - raw_center) / raw_half) × sim_half`

- **Real-Time Joint Display**
  - Terminal table showing Raw / Norm / Leader(rad) / Sim(rad) / Diff per joint, updated every 10 steps via ANSI cursor-up overwrite

### Fixed
- **Action offset bug** — `use_default_offset=True` adds `default_joint_pos` internally; fixed with `action = leader_joints - default_joint_pos`
- **Serial read resilience** — failed reads fall back to last known value (no more snap-to-center jitter)

### Performance
- Per-joint deadband: `DEADBAND_COUNTS=[4,4,4,4,4,1]` (gripper near-instant)
- Serial timeout: `1ms`, inter-read sleep: `0.5ms`

### Files Updated
- `teleoperation/teleop_so100_leader.py` — full rewrite with calibration, offset fix, display, speed tuning
- `teleoperation/teleop_joint_agent.py` — Wrist Flex key: R/V → C/V

---

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
