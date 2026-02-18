# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to run SO-100 Leader Arm teleoperation with direct joint position control.

LEADER ARM TELEOPERATION:
=========================
Physically move the SO-100 LEADER arm — the follower robot in Isaac Lab
mirrors its joint positions in real time.

JOINT MAPPING (Leader → Simulation):
--------------------------------------
  Motor ID 1  →  Joint 0: Shoulder Pan
  Motor ID 2  →  Joint 1: Shoulder Lift
  Motor ID 3  →  Joint 2: Elbow Flex
  Motor ID 4  →  Joint 3: Wrist Flex
  Motor ID 5  →  Joint 4: Wrist Roll
  Motor ID 6  →  Joint 5: Gripper

HARDWARE REQUIREMENTS:
-----------------------
  - SO-100 Leader Arm connected via USB cable (the arm you already have)
  - NOTE: Feetech STS3215 are the servos BUILT INTO the SO-100 arm itself
          — no external hardware needed, just the arm + USB cable
  - pyserial installed:  pip install pyserial  (likely already installed)

CONFIGURATION (edit the constants below):
------------------------------------------
  LEADER_PORT      : USB serial port  (e.g. /dev/ttyUSB0 or /dev/ttyACM0)
  LEADER_BAUDRATE  : Serial baudrate  (default: 1000000)
  MOTOR_IDS        : Servo IDs on the bus (default: [1, 2, 3, 4, 5, 6])

USAGE:
------
  ./isaaclab.sh -p scripts/environments/teleoperation/teleop_so100_leader.py \\
      --task Isaac-Lift-Cube-SO100-v0 \\
      --port /dev/ttyUSB0

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
RAW_CENTER      = 2048   # neutral = 0 rad
COUNTS_PER_RAD  = (RAW_MAX - RAW_MIN) / (2 * math.pi)  # ≈ 651.9 counts/rad

# Per-joint sign correction: flip -1 for any joint that moves wrong way in sim
JOINT_SIGNS = [1, 1, 1, 1, 1, 1]


class SO100LeaderController:
    """
    Reads joint positions from a physical SO-100 leader arm using raw pyserial.
    Implements the Feetech STS/SMS protocol directly — no scservo_sdk needed.
    """

    # ------------------------------------------------------------------ #
    # Feetech STS protocol constants
    # ------------------------------------------------------------------ #
    _HEADER          = 0xFF
    _INST_READ       = 0x02
    _ADDR_PRESENT_POS = 56   # Present Position register (2 bytes)

    def __init__(self, port: str, baudrate: int, motor_ids: list, device: str = "cuda:0"):
        self.motor_ids   = motor_ids
        self.num_joints  = len(motor_ids)
        self.device      = device
        self._zero_offsets = np.zeros(self.num_joints, dtype=np.float32)

        import serial
        self._ser = serial.Serial(port, baudrate=baudrate, timeout=0.05)
        time.sleep(0.1)  # let port settle

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
            raw[i] = float(val) if val is not None else RAW_CENTER
        return raw

    def _raw_to_radians(self, raw: np.ndarray) -> np.ndarray:
        return (raw - RAW_CENTER) / COUNTS_PER_RAD

    def calibrate_zero(self):
        raw = self._read_raw_positions()
        self._zero_offsets = self._raw_to_radians(raw)
        print(f"[LeaderArm] Zero calibrated: {np.round(self._zero_offsets, 3)} rad")

    def get_joint_positions(self) -> torch.Tensor:
        raw       = self._read_raw_positions()
        rad       = self._raw_to_radians(raw)
        rel_rad   = rad - self._zero_offsets
        corrected = np.array(JOINT_SIGNS, dtype=np.float32) * rel_rad
        return torch.tensor(corrected, dtype=torch.float32, device=self.device)

    def reset(self):
        self.calibrate_zero()

    def close(self):
        if hasattr(self, "_ser") and self._ser.is_open:
            self._ser.close()
            print("[LeaderArm] Serial port closed.")

    def __del__(self):
        self.close()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

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

    # Reset environment and calibrate leader arm zero position
    obs, _ = env.reset()
    leader.calibrate_zero()

    # Read initial sim joint positions as base target
    robot_joint_pos     = env.unwrapped.scene["robot"].data.joint_pos[:, :NUM_JOINTS].clone()
    sim_home_positions  = robot_joint_pos[0].clone()

    print("SO-100 Leader Arm teleoperation started!")
    print(f"Sim home positions: {sim_home_positions}\n")
    print("Move the leader arm to control the simulation robot.")
    print("Press BACKSPACE in Isaac Sim window to reset.\n")

    should_reset = False

    while simulation_app.is_running():
        try:
            with torch.inference_mode():

                # -----------------------------------------------------------
                # Read leader arm joint positions
                # -----------------------------------------------------------
                leader_joints = leader.get_joint_positions()

                # Add leader arm delta on top of sim home positions
                # (so simulation robot starts at its default pose, not 0,0,0)
                current_joint_targets = sim_home_positions + leader_joints

                # Broadcast to all parallel environments
                actions = current_joint_targets.unsqueeze(0).repeat(env.num_envs, 1)

                # Step simulation
                env.step(actions)

                # -----------------------------------------------------------
                # Reset on BACKSPACE (Isaac Sim keyboard still works)
                # -----------------------------------------------------------
                if should_reset:
                    obs, _ = env.reset()
                    leader.reset()
                    robot_joint_pos    = env.unwrapped.scene["robot"].data.joint_pos[:, :NUM_JOINTS].clone()
                    sim_home_positions = robot_joint_pos[0].clone()
                    should_reset       = False
                    print(f"[Reset] New sim home: {sim_home_positions}")

        except KeyboardInterrupt:
            print("\n[Info] KeyboardInterrupt — exiting.")
            break
        except Exception as e:
            print(f"[Error] {e}")
            break

    env.close()
    leader.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
