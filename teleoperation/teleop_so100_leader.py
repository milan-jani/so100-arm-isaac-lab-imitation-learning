# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to run SO-100 Leader Arm teleoperation with direct joint position control.

VERSION: v1.2  —  February 19, 2026
================================================================================

LEADER ARM TELEOPERATION:
=========================
Physically move the SO-100 LEADER arm — the simulation robot in Isaac Lab
mirrors its joint positions in real time with ~0.000 rad difference.

JOINT MAPPING (Leader → Simulation):
--------------------------------------
  Motor ID 1  →  Joint 0: Shoulder Pan
  Motor ID 2  →  Joint 1: Shoulder Lift
  Motor ID 3  →  Joint 2: Elbow Flex
  Motor ID 4  →  Joint 3: Wrist Flex
  Motor ID 5  →  Joint 4: Wrist Roll
  Motor ID 6  →  Joint 5: Gripper / Jaw

HARDWARE REQUIREMENTS:
-----------------------
  - SO-100 Leader Arm connected via USB cable
  - Feetech STS3215 servos are built into the arm — no extra SDK needed
  - pyserial: pip install pyserial

STARTUP SEQUENCE:
-----------------
  1. Physics warms up (30 steps)
  2. Sim joint limits read from environment
  3. 8-second calibration — move ALL joints through full range
  4. Gripper uses saved calibration (GRIPPER_RAW_CLOSED / GRIPPER_RAW_OPEN)
  5. Sim arm snaps to leader arm starting position
  6. Real-time teleoperation begins

POSITION MAPPING:
-----------------
  normalized = clip((raw - raw_center) / raw_half, -1, +1)
  sim_pos    = sim_center + JOINT_SIGNS[i] × normalized × sim_half
  action     = sim_pos - default_joint_pos   ← offset cancels use_default_offset=True

RESPONSE TIME TUNING:
---------------------
  DEADBAND_COUNTS  — per-joint noise gate (lower = faster, higher = smoother)
  SERIAL_TIMEOUT   — max wait per servo read (currently 1ms)
  INTER_READ_SLEEP — gap between motor reads (currently 0.5ms)

CONTROLS:
---------
  Move leader arm  → sim arm mirrors exactly
  BACKSPACE        → reset environment (Isaac Sim window focused)
  Ctrl+C           → quit

USAGE:
------
  ./isaaclab.sh -p scripts/environments/teleoperation/teleop_so100_leader.py \\
      --task Isaac-Lift-Cube-SO100-v0 --port /dev/ttyACM0

  Or via alias (add to ~/.bash_aliases):
    alias teleop_so100='cd ~/IsaacLab && ./isaaclab.sh -p scripts/environments/teleoperation/teleop_so100_leader.py --task Isaac-Lift-Cube-SO100-v0'

"""

"""Launch Isaac Sim Simulator first."""

import argparse
from isaaclab.app import AppLauncher

# ---------------------------------------------------------------------------
# Argument parsing — must happen before AppLauncher
# ---------------------------------------------------------------------------
parser = argparse.ArgumentParser(description="SO-100 Leader Arm teleoperation for Isaac Lab.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task",     type=str, default=None, help="Name of the task.")
parser.add_argument("--port",     type=str, default="/dev/ttyACM0",
                    help="Serial port of the SO-100 leader arm (e.g. /dev/ttyACM0).")
parser.add_argument("--baudrate", type=int, default=1_000_000,
                    help="Baudrate for the leader arm serial connection.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Force cameras on
args_cli.enable_cameras = True

app_launcher    = AppLauncher(args_cli)
simulation_app  = app_launcher.app

"""Rest everything follows."""

import math
import struct
import time
import numpy as np
import torch
import gymnasium as gym
import carb
import omni.appwindow

from isaaclab.managers import TerminationTermCfg as DoneTerm

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.manager_based.manipulation.lift import mdp
from isaaclab_tasks.utils import parse_env_cfg

# ---------------------------------------------------------------------------
# Leader-Arm Configuration  — EDIT THESE IF NEEDED
# ---------------------------------------------------------------------------
MOTOR_IDS       = [1, 2, 3, 4, 5, 6]   # Feetech servo IDs on the bus
NUM_JOINTS      = 6

# Feetech STS3215 encoder: 0–4095 counts over 0°–360°
RAW_MIN         = 0
RAW_MAX         = 4095

# Per-joint sign correction: flip -1 for any joint that moves wrong way in sim
JOINT_SIGNS = [1, 1, 1, 1, 1, 1]

# ---------------------------------------------------------------------------
# WRIST ROLL HARDCODED CALIBRATION — from your confirmed good sweep
#   raw_center=2046, raw_half=2046 captured when both directions worked.
#   Set to None to let calibrate_range() compute it from sweep instead.
# ---------------------------------------------------------------------------
WRIST_ROLL_RAW_CENTER = 2046
WRIST_ROLL_RAW_HALF   = 2046
WRIST_ROLL_JOINT_IDX  = 4

# Gripper is joint index 5 — needs dedicated 2-point calibration
GRIPPER_JOINT_IDX = 5

# ---------------------------------------------------------------------------
# GRIPPER HARDCODED CALIBRATION  — set from your one-time calibration run
# ---------------------------------------------------------------------------
# Set both to None to re-run interactive calibration at startup.
# These are YOUR measured values — do not change unless you re-calibrate.
GRIPPER_RAW_CLOSED = 2027   # raw encoder value when jaw is fully closed
GRIPPER_RAW_OPEN   = 3172   # raw encoder value when jaw is fully open
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# RESPONSE TIME CONTROLS — tune these if arm feels laggy or jittery
# ---------------------------------------------------------------------------
# DEADBAND_COUNTS: minimum encoder count change to accept a new reading.
#   Lower  = more responsive but more jittery noise.
#   Higher = smoother but feels sluggish.
#   Gripper (index 5) is 1 = near-instant. Arm joints are 4.
DEADBAND_COUNTS = [4, 4, 4, 4, 4, 1]

# SERIAL_TIMEOUT: seconds to wait for each servo response.
#   Lower = faster reads but more missed packets → fallback to last value.
#   0.005 = 5ms is aggressive but works well at 1Mbit with USB direct connection.
SERIAL_TIMEOUT = 0.001

# INTER_READ_SLEEP: seconds between reading each motor on the bus.
#   Prevents serial collision between consecutive motor reads.
#   0.0005 = 0.5ms is enough headroom at 1Mbit.
INTER_READ_SLEEP = 0.0005
# ---------------------------------------------------------------------------

# Calibration time: how many seconds to move arm through full range at startup
CALIBRATION_SECONDS = 8


class SO100LeaderController:
    """
    Reads joint positions from a physical SO-100 leader arm using raw pyserial.
    Implements the Feetech STS/SMS protocol directly — no scservo_sdk needed.
    """

    # ------------------------------------------------------------------ #
    # Feetech STS protocol constants
    # ------------------------------------------------------------------ #
    _HEADER           = 0xFF
    _INST_READ        = 0x02
    _ADDR_PRESENT_POS = 56   # Present Position register (2 bytes)

    def __init__(self, port: str, baudrate: int, motor_ids: list, device: str = "cuda:0"):
        self.motor_ids  = motor_ids
        self.num_joints = len(motor_ids)
        self.device     = device

        # Last known good raw values (fallback on read failure)
        self._last_raw  = np.full(self.num_joints, 2048, dtype=np.float32)

        # Calibrated range per joint (set by calibrate_range)
        self._raw_center    = np.full(self.num_joints, 2048.0, dtype=np.float32)
        self._raw_half      = np.full(self.num_joints, 1024.0, dtype=np.float32)  # half-range

        # Sim joint limits (set by set_sim_limits before first use)
        self._sim_center    = np.zeros(self.num_joints, dtype=np.float32)
        self._sim_half      = np.ones(self.num_joints,  dtype=np.float32)

        # Deadband: last raw that was accepted
        self._accepted_raw  = np.full(self.num_joints, 2048.0, dtype=np.float32)

        import serial
        self._ser = serial.Serial(port, baudrate=baudrate, timeout=SERIAL_TIMEOUT)
        time.sleep(0.1)

        print("\n" + "=" * 60)
        print("SO-100 LEADER ARM TELEOPERATION")
        print("=" * 60)
        print(f"  Port     : {port}  (pyserial, no extra SDK needed)")
        print(f"  Baudrate : {baudrate}")
        print(f"  Motors   : {motor_ids}")
        print()
        labels = ["Shoulder Pan", "Shoulder Lift", "Elbow Flex",
                  "Wrist Flex",   "Wrist Roll",    "Gripper"]
        for i, (mid, label) in enumerate(zip(motor_ids, labels)):
            print(f"  Motor ID {mid}  →  Joint {i}: {label}")
        print()
        print("  Move the leader arm physically to control the simulation.")
        print("  Press BACKSPACE in Isaac Sim window to reset environment.")
        print("=" * 60 + "\n")

        self._verify_motors()

    # ------------------------------------------------------------------ #
    # Low-level STS protocol helpers
    # ------------------------------------------------------------------ #

    def _checksum(self, data: list) -> int:
        return (~sum(data)) & 0xFF

    def _read_register(self, motor_id: int, address: int, length: int) -> int | None:
        """Send a READ instruction and return the integer value, or None on failure."""
        try:
            params   = [address, length]
            data     = [motor_id, len(params) + 2, self._INST_READ] + params
            packet   = bytes([self._HEADER, self._HEADER] + data + [self._checksum(data)])

            self._ser.reset_input_buffer()
            self._ser.write(packet)

            # Response: FF FF ID LEN ERR [DATA...] CHECKSUM
            header = self._ser.read(4)
            if len(header) < 4 or header[0] != 0xFF or header[1] != 0xFF:
                return None

            resp_id  = header[2]
            resp_len = header[3]           # number of remaining bytes incl. checksum
            rest     = self._ser.read(resp_len)
            if len(rest) < resp_len:
                return None

            # rest = [ERR, data..., checksum]
            data_bytes = rest[1:-1]        # strip ERR and checksum
            if len(data_bytes) == 2:
                # Little-endian 16-bit
                return data_bytes[0] | (data_bytes[1] << 8)
            elif len(data_bytes) == 1:
                return data_bytes[0]
            return None
        except Exception:
            # Covers SerialException (device disconnected / no data) and OSError
            return None

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #

    def _verify_motors(self):
        for mid in self.motor_ids:
            val = self._read_register(mid, self._ADDR_PRESENT_POS, 2)
            if val is None:
                print(f"[LeaderArm] WARNING: Motor ID {mid} not responding — check wiring/power!")
            else:
                print(f"[LeaderArm] Motor ID {mid} ✓  raw={val}")
        print()

    def _read_raw_positions(self) -> np.ndarray:
        raw = np.zeros(self.num_joints, dtype=np.float32)
        for i, mid in enumerate(self.motor_ids):
            val = self._read_register(mid, self._ADDR_PRESENT_POS, 2)
            if val is not None:
                raw[i] = float(val)
                self._last_raw[i] = raw[i]
            else:
                raw[i] = self._last_raw[i]
            # Skip inter-read delay for the last motor (gripper) — nothing follows it on the bus
            if i < self.num_joints - 1:
                time.sleep(INTER_READ_SLEEP)  # prevents serial bus collision between motors
        return raw

    # ------------------------------------------------------------------ #
    # Calibration & limit setup
    # ------------------------------------------------------------------ #

    def set_sim_limits(self, sim_min: np.ndarray, sim_max: np.ndarray):
        """Pass the sim's actual joint limits so mapping is exact."""
        self._sim_center = ((sim_min + sim_max) / 2.0).astype(np.float32)
        self._sim_half   = ((sim_max - sim_min) / 2.0).astype(np.float32)
        print("[LeaderArm] Sim joint limits loaded.")
        print(f"  sim_min : {np.round(sim_min, 3)}")
        print(f"  sim_max : {np.round(sim_max, 3)}")

    def calibrate_range(self, seconds: int = CALIBRATION_SECONDS):
        """
        Timed calibration: user moves every joint through its FULL physical range.
        Records observed raw min/max per joint → used for proper linear mapping.
        """
        print("\n" + "=" * 60)
        print("CALIBRATION — Move ALL joints through their FULL range")
        print(f"You have {seconds} seconds.")
        print("  • Move EACH joint fully to BOTH ends (left AND right / up AND down)")
        print("  • Especially WRIST ROLL — sweep both directions from center")
        print("  • Gripper: calibrated separately after this step")
        print("=" * 60)

        # Prime the serial bus: flush stale data and warm up all servos
        # (especially gripper/jaw which can lag on cold start)
        print("  Priming servo bus...", end="", flush=True)
        self._ser.reset_input_buffer()
        for _ in range(5):
            self._read_raw_positions()
            time.sleep(0.05)
        print(" ready.\n")

        raw_lo = np.full(self.num_joints, RAW_MAX, dtype=np.float32)
        raw_hi = np.full(self.num_joints, RAW_MIN, dtype=np.float32)

        deadline = time.time() + seconds
        while time.time() < deadline:
            raw     = self._read_raw_positions()
            raw_lo  = np.minimum(raw_lo, raw)
            raw_hi  = np.maximum(raw_hi, raw)
            remain  = deadline - time.time()
            ranges  = (raw_hi - raw_lo).astype(int)
            print(f"\r  {remain:4.1f}s left | observed range (counts): {ranges}", end="", flush=True)

        print("\n")

        # Guard: if a joint was never moved, use a sensible default range
        for i in range(self.num_joints):
            if (raw_hi[i] - raw_lo[i]) < 50:
                print(f"  [!] Joint {i} barely moved — using default ±half range. Move it next time!")
                raw_lo[i] = 2048 - 1024
                raw_hi[i] = 2048 + 1024

        self._raw_center = ((raw_hi + raw_lo) / 2.0).astype(np.float32)
        self._raw_half   = ((raw_hi - raw_lo) / 2.0).astype(np.float32)

        print("  [Calibration] Center = midpoint of swept range per joint.")
        self._accepted_raw[:] = self._read_raw_positions()

        # Wrist roll hardcoded override — ignore sweep result, use known-good values
        if WRIST_ROLL_RAW_CENTER is not None and WRIST_ROLL_RAW_HALF is not None:
            self._raw_center[WRIST_ROLL_JOINT_IDX] = float(WRIST_ROLL_RAW_CENTER)
            self._raw_half[WRIST_ROLL_JOINT_IDX]   = float(WRIST_ROLL_RAW_HALF)
            print(f"  [WristRoll] Hardcoded: center={WRIST_ROLL_RAW_CENTER}, half={WRIST_ROLL_RAW_HALF}")

        print("[LeaderArm] Calibration complete!")
        print(f"  raw_center : {self._raw_center.astype(int)}")
        print(f"  raw_range  : {(self._raw_half * 2).astype(int)}")
        print()

    def calibrate_gripper(self):
        """
        2-point gripper calibration.
        Uses GRIPPER_RAW_CLOSED / GRIPPER_RAW_OPEN if set — skips interactive prompt.
        """
        idx = GRIPPER_JOINT_IDX

        if GRIPPER_RAW_CLOSED is not None and GRIPPER_RAW_OPEN is not None:
            # Use hardcoded values — no user interaction needed
            raw_closed = float(GRIPPER_RAW_CLOSED)
            raw_open   = float(GRIPPER_RAW_OPEN)
            print(f"[LeaderArm] Gripper: using saved calibration "
                  f"(closed={int(raw_closed)}, open={int(raw_open)})")
        else:
            # Interactive calibration
            print("\n" + "=" * 60)
            print("GRIPPER CALIBRATION (2-point)")
            print("=" * 60)
            input("  1. Fully CLOSE the leader arm jaw/gripper, then press Enter...")
            raws = []
            for _ in range(5):
                r = self._read_raw_positions()
                raws.append(r[idx])
                time.sleep(0.02)
            raw_closed = float(np.median(raws))
            print(f"     Recorded CLOSED raw = {int(raw_closed)}")

            input("  2. Fully OPEN  the leader arm jaw/gripper, then press Enter...")
            raws = []
            for _ in range(5):
                r = self._read_raw_positions()
                raws.append(r[idx])
                time.sleep(0.02)
            raw_open = float(np.median(raws))
            print(f"     Recorded OPEN  raw = {int(raw_open)}")
            print(f"\n  → To skip this next time, set in code:")
            print(f"      GRIPPER_RAW_CLOSED = {int(raw_closed)}")
            print(f"      GRIPPER_RAW_OPEN   = {int(raw_open)}")

        # Override calibration for gripper index only
        # Map: raw_closed → sim_joint_min[idx],  raw_open → sim_joint_max[idx]
        # raw_center = midpoint, raw_half = half-range
        self._raw_center[idx] = (raw_closed + raw_open) / 2.0
        self._raw_half[idx]   = abs(raw_open - raw_closed) / 2.0

        # Determine sign: if physically open = higher raw → positive norm = open → sign=+1
        # If physically open = lower raw → positive norm = closed → need sign=-1
        # sim convention: sim_max = open, sim_min = closed
        # norm = (raw - center) / half → +1 when raw=raw_open, -1 when raw=raw_closed
        # sim_pos = sim_center + sign * norm * sim_half
        # We want norm=+1 → sim_max (open), norm=-1 → sim_min (closed)
        # If raw_open > raw_closed: norm at open = +1 → sign=+1 maps +1 to sim_max ✓
        # If raw_open < raw_closed: norm at open = -1 → sign=-1 maps -1*(-1)=+1 to sim_max ✓
        if raw_open < raw_closed:
            JOINT_SIGNS[idx] = -1
            print(f"     Gripper sign auto-set to -1 (open = lower raw value)")
        else:
            JOINT_SIGNS[idx] = 1
            print(f"     Gripper sign auto-set to +1 (open = higher raw value)")

        self._accepted_raw[idx] = self._read_raw_positions()[idx]
        print(f"  Gripper calibration done: center={int(self._raw_center[idx])}, "
              f"half={int(self._raw_half[idx])}, sign={JOINT_SIGNS[idx]}")
        print()

    def get_joint_positions(self) -> torch.Tensor:
        """
        Read leader arm and return sim joint positions directly.

        Mapping (per joint):
          1. Apply deadband — ignore tiny noise without adding any lag
          2. Normalize raw → [-1, 1] using calibrated physical range
          3. Apply sign correction
          4. Scale to sim joint range:  sim_center + normalized * sim_half
        """
        raw = self._read_raw_positions()

        # Deadband: only accept a reading if it moved more than per-joint threshold
        deadband = np.array(DEADBAND_COUNTS, dtype=np.float32)
        diff = np.abs(raw - self._accepted_raw)
        self._accepted_raw = np.where(diff >= deadband, raw, self._accepted_raw)

        # Normalize to [-1, +1] based on calibrated physical range
        normalized = (self._accepted_raw - self._raw_center) / np.maximum(self._raw_half, 1.0)
        normalized = np.clip(normalized, -1.0, 1.0)

        # Apply sign and map to sim range
        signs    = np.array(JOINT_SIGNS, dtype=np.float32)
        sim_pos  = self._sim_center + signs * normalized * self._sim_half

        # Store for external display
        self._last_normalized = normalized.copy()
        self._last_sim_pos    = sim_pos.copy()
        self._last_raw_used   = self._accepted_raw.copy()

        return torch.tensor(sim_pos, dtype=torch.float32, device=self.device)

    def reset(self):
        """Re-seed accepted_raw from current physical position."""
        self._accepted_raw[:] = self._read_raw_positions()
        print("[LeaderArm] Reset — mirroring current leader arm position.")

    def close(self):
        if hasattr(self, "_ser") and self._ser.is_open:
            self._ser.close()
            print("[LeaderArm] Serial port closed.")

    def __del__(self):
        self.close()


# ---------------------------------------------------------------------------
# Keyboard handler — BACKSPACE to reset (same pattern as teleop_joint_agent)
# ---------------------------------------------------------------------------
class KeyboardHandler:
    def __init__(self):
        self._key_state = {}
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input     = carb.input.acquire_input_interface()
        self._keyboard  = self._appwindow.get_keyboard()
        self._sub       = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_event)

    def _on_event(self, event, *args, **kwargs):
        if not hasattr(event, "input") or not hasattr(event.input, "name"):
            return True
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            self._key_state[event.input.name] = True
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self._key_state[event.input.name] = False
        return True

    def consume_backspace(self) -> bool:
        """Returns True once if BACKSPACE was pressed, then clears the flag."""
        if self._key_state.get("BACKSPACE", False):
            self._key_state["BACKSPACE"] = False
            return True
        return False

    def __del__(self):
        if hasattr(self, "_sub") and hasattr(self, "_input"):
            try:
                self._input.unsubscribe_to_keyboard_events(self._keyboard, self._sub)
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Real-time joint display
# ---------------------------------------------------------------------------
JOINT_LABELS = ["Shoulder Pan ", "Shoulder Lift", "Elbow Flex   ",
                "Wrist Flex   ", "Wrist Roll   ", "Gripper/Jaw  "]
TABLE_LINES = 10  # total lines printed by print_joint_table (for cursor-up)

def print_joint_table(leader: "SO100LeaderController",
                      sim_joints: np.ndarray,
                      step: int,
                      first_print: bool = False):
    """
    Print (or overwrite) a side-by-side table of leader vs sim joint values.
    Uses ANSI cursor-up so it stays in place instead of scrolling.
    """
    raw    = leader._last_raw_used
    norm   = leader._last_normalized
    leader_sim = leader._last_sim_pos   # what was sent to sim

    if not first_print:
        # Move cursor up TABLE_LINES lines and clear to end of screen
        print(f"\033[{TABLE_LINES}A\033[J", end="")
    else:
        # Reserve blank lines so cursor-up on next print lands correctly
        print("\n" * TABLE_LINES, end="")

    print(f"  Step {step:>6d}  | {'Raw':>6} | {'Norm':>6} | {'Leader(rad)':>11} | {'Sim(rad)':>10} | {'Diff':>8}")
    print("  " + "-" * 66)
    for i, label in enumerate(JOINT_LABELS):
        diff = leader_sim[i] - sim_joints[i]
        print(f"  {label} | {int(raw[i]):>6} | {norm[i]:>+6.3f} | {leader_sim[i]:>+11.4f} | {sim_joints[i]:>+10.4f} | {diff:>+8.4f}")
    print("  " + "-" * 66)
    print("  BACKSPACE = reset  |  Ctrl+C = quit")


def main():
    """Run SO-100 leader arm teleoperation."""

    # Parse environment config
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs)
    env_cfg.terminations.time_out = None
    if "Lift" in args_cli.task:
        env_cfg.commands.object_pose.resampling_time_range = (1.0e9, 1.0e9)
        env_cfg.terminations.object_reached_goal = DoneTerm(func=mdp.object_reached_goal)

    # Create Isaac Lab environment
    env    = gym.make(args_cli.task, cfg=env_cfg).unwrapped
    device = env.device

    # Create leader arm controller
    leader = SO100LeaderController(
        port      = args_cli.port,
        baudrate  = args_cli.baudrate,
        motor_ids = MOTOR_IDS,
        device    = device,
    )

    # Set up keyboard handler for BACKSPACE reset
    kbd = KeyboardHandler()

    # Reset environment
    obs, _ = env.reset()

    # Warmup — let physics settle
    _warmup_action = torch.zeros(env.num_envs, NUM_JOINTS, device=device)
    for _ in range(30):
        env.step(_warmup_action)

    # Read sim joint limits from the environment and pass to controller
    joint_limits = env.unwrapped.scene["robot"].data.joint_limits[0, :NUM_JOINTS, :]
    sim_joint_min = joint_limits[:, 0].cpu().numpy()
    sim_joint_max = joint_limits[:, 1].cpu().numpy()
    joint_names = ["ShoulderPan", "ShoulderLft", "ElbowFlex  ", "WristFlex  ", "WristRoll  ", "Gripper    "]
    print("\n[Info] Sim joint limits (from USD):")
    for i, name in enumerate(joint_names):
        print(f"  {name}: [{sim_joint_min[i]:.3f}, {sim_joint_max[i]:.3f}]  range={sim_joint_max[i]-sim_joint_min[i]:.3f} rad")
    print()
    leader.set_sim_limits(sim_joint_min, sim_joint_max)

    # Auto-calibrate: user moves arm through full physical range
    leader.calibrate_range(seconds=CALIBRATION_SECONDS)

    # Dedicated gripper calibration — 2-point close/open to get exact jaw mapping
    leader.calibrate_gripper()

    # Init display fields so print_joint_table works before first get_joint_positions call
    leader._last_raw_used   = leader._accepted_raw.copy()
    leader._last_normalized = np.zeros(NUM_JOINTS, dtype=np.float32)
    leader._last_sim_pos    = np.zeros(NUM_JOINTS, dtype=np.float32)

    # Step sim to leader arm's current starting position immediately
    # (so sim arm matches leader from frame 1, not from sim default)
    #
    # IMPORTANT: action space uses use_default_offset=True, scale=1.0
    # so:  sim_pos = default_joint_pos + action
    # To get sim_pos = leader_joints  →  action = leader_joints - default_joint_pos
    default_joint_pos = env.unwrapped.scene["robot"].data.default_joint_pos[0, :NUM_JOINTS].clone()
    print(f"[Info] Robot default joint pos: {np.round(default_joint_pos.cpu().numpy(), 4)}")

    start_pos = leader.get_joint_positions()
    start_action = (start_pos - default_joint_pos).unsqueeze(0).repeat(env.num_envs, 1)
    for _ in range(10):
        env.step(start_action)

    print("\nSO-100 Leader Arm teleoperation started!")
    print("Leader arm is now directly mirrored to simulation — 1:1 position match.")
    print("Move the leader arm to control the simulation robot.")
    print("Press BACKSPACE in Isaac Sim window to reset environment.\n")

    should_reset = False
    step_count   = 0
    first_print  = True

    while simulation_app.is_running():
        try:
            with torch.inference_mode():

                # -----------------------------------------------------------
                # Read leader arm joint positions
                # -----------------------------------------------------------
                leader_joints = leader.get_joint_positions()  # absolute radians

                # OFFSET CORRECTION: use_default_offset=True means sim applies:
                #   sim_pos = default_joint_pos + action
                # So to get sim_pos == leader_joints exactly:
                #   action = leader_joints - default_joint_pos
                actions = (leader_joints - default_joint_pos).unsqueeze(0).repeat(env.num_envs, 1)

                # Step simulation
                env.step(actions)
                step_count += 1

                # Read actual sim joint positions for comparison display
                sim_joints = env.unwrapped.scene["robot"].data.joint_pos[0, :NUM_JOINTS].cpu().numpy()

                # Print side-by-side table every 10 steps (no scroll spam)
                if step_count % 10 == 0 or first_print:
                    print_joint_table(leader, sim_joints, step_count, first_print)
                    first_print = False

                # -----------------------------------------------------------
                # Reset on BACKSPACE
                # -----------------------------------------------------------
                if kbd.consume_backspace():
                    should_reset = True

                if should_reset:
                    obs, _ = env.reset()
                    leader.reset()
                    # Snap sim to current leader position after reset
                    snap_pos    = leader.get_joint_positions()
                    snap_action = (snap_pos - default_joint_pos).unsqueeze(0).repeat(env.num_envs, 1)
                    for _ in range(10):
                        env.step(snap_action)
                    should_reset = False
                    first_print  = True
                    step_count   = 0
                    print("[Reset] Done.")

        except KeyboardInterrupt:
            print("\n[Info] KeyboardInterrupt — exiting.")
            break
        except Exception as e:
            err_str = str(e)
            if "device disconnected" in err_str or "readiness to read" in err_str or "returned no data" in err_str:
                # Serial port disconnect — last-known positions will be held automatically
                # Just warn and continue; the arm will freeze in place rather than crash
                print(f"[SerialWarning] USB connection issue: {e}")
                print("[SerialWarning] Check cable / run: fuser -k /dev/ttyACM0")
                print("[SerialWarning] Holding last joint positions. Reconnect and press BACKSPACE to recalibrate.")
                time.sleep(0.5)  # brief pause to avoid log spam
            else:
                print(f"[Error] {e}")
                break

    env.close()
    leader.close()
    del kbd


if __name__ == "__main__":
    main()
    simulation_app.close()
