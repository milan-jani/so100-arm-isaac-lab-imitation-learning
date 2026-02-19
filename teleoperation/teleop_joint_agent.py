# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to run keyboard teleoperation with direct joint position control.

KEYBOARD CONTROL CONFIGURATION:
================================

JOINT MAPPINGS (Change in advance() method, ~Line 95-180):
-----------------------------------------------------------
delta[0] = Shoulder Pan   → Keys: Q (left) / A (right)
delta[1] = Shoulder Lift  → Keys: W (up) / S (down)
delta[2] = Elbow Flex     → Keys: E (extend) / D (retract)
delta[3] = Wrist Flex     → Keys: C (up) / V (down)
delta[4] = Wrist Roll     → Keys: T (rotate+) / G (rotate-)
delta[5] = Gripper        → Keys: Z (open) / X (close)

SPECIAL KEYS:
-------------
BACKSPACE → Reset environment (Line ~277)
F         → Blocked to prevent camera reset (Line ~118)
SPACE     → Blocked to prevent simulation pause (Line ~121)
ESC       → Quit application (Isaac Sim default)

TO CUSTOMIZE:
-------------
1. Change key mappings: Edit the advance() method (~Line 95-180)
2. Change reset key: Edit line ~245 (currently BACKSPACE)
3. Change camera block: Edit _on_keyboard_event() method (~Line 81)
4. Change joint delta: Use --joint_delta command line argument

USAGE:
------
./isaaclab.sh -p scripts/environments/teleoperation/teleop_joint_agent.py \\
    --task Isaac-Lift-Cube-SO100-v0 \\
    --joint_delta 0.1

"""

"""Launch Isaac Sim Simulator first."""

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Joint position keyboard teleoperation for Isaac Lab environments.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--joint_delta", type=float, default=0.1, help="Joint movement delta in radians per key press.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# Force enable cameras (permanent)
args_cli.enable_cameras = True

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import torch
import carb
import omni.appwindow

from isaaclab.managers import TerminationTermCfg as DoneTerm

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.manager_based.manipulation.lift import mdp
from isaaclab_tasks.utils import parse_env_cfg


class JointKeyboardController:
    """Simple keyboard controller for independent joint position control."""
    
    def __init__(self, num_joints: int = 6, joint_delta: float = 0.1, device: str = "cuda:0"):
        """
        Initialize joint keyboard controller.
        
        Args:
            num_joints: Number of joints to control (5 arm + 1 gripper)
            joint_delta: Default movement amount per key press in radians
            device: Device to create tensors on (cuda:0 or cpu)
        """
        self.num_joints = num_joints
        self.joint_delta = joint_delta
        self.device = device
        self.joint_positions = torch.zeros(num_joints, device=device)
        
        # Per-joint sensitivity multipliers (modify these for different joint speeds)
        # Lower values = slower, more precise control
        # For continuous movement (joint moves only while key held):
        #   Use values between 0.1 - 0.5 for smooth control
        self.joint_sensitivity = {
            0: 0.25,  # Shoulder Pan - slower for precision
            1: 0.25,  # Shoulder Lift - slower for precision
            2: 0.25,  # Elbow Flex - slower for precision
            3: 0.25,  # Wrist Flex - even slower (fine control)
            4: 0.25,  # Wrist Roll - even slower (fine control)
            5: 0.25,  # Gripper - faster (gripper needs quick open/close)
        }
        
        # Keyboard interface
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_keyboard_event)
        
        # Key state tracking
        self._key_state = {}
        
        print("\n" + "="*60)
        print("JOINT POSITION KEYBOARD CONTROL")
        print("="*60)
        print("Independent Joint Control:")
        print("  Q/A - Shoulder Pan (base rotation)")
        print("  W/S - Shoulder Lift (arm up/down)")
        print("  E/D - Elbow Flex (extend/retract)")
        print("  C/V - Wrist Flex (wrist up/down)")  # Changed from R/V to C/V
        print("  T/G - Wrist Roll (wrist rotation)")
        print("  Z/X - Gripper (open/close)")
        print("\nOther:")
        print("  BACKSPACE - Reset environment")
        print("  F & SPACE - Blocked (camera reset & pause prevention)")
        print("  ESC - Quit")
        print("="*60 + "\n")
    
    def _on_keyboard_event(self, event, *args, **kwargs):
        """Handle keyboard events."""
        # Check if event has input attribute (safety check)
        if not hasattr(event, 'input') or not hasattr(event.input, 'name'):
            return True
        
        # Define all joint control keys that should be blocked from Isaac Lab's camera/environment controls
        joint_control_keys = {"Q", "A", "W", "S", "E", "D", "C", "V", "T", "G", "Z", "X"}
        
        # Block joint control keys from propagating to camera/environment controls
        if event.input.name in joint_control_keys:
            # Update our key state
            if event.type == carb.input.KeyboardEventType.KEY_PRESS:
                self._key_state[event.input.name] = True
            elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
                self._key_state[event.input.name] = False
            return False  # Block propagation to prevent camera/environment shortcuts
            
        # Block F key to prevent camera reset
        if event.input.name == "F":
            return False  # Don't propagate F key to other handlers
        
        # Block SPACE key to prevent simulation pause
        if event.input.name == "SPACE":
            return False  # Don't propagate SPACE key (prevents pause/play toggle)
            
        # For other keys, track state but allow propagation
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            self._key_state[event.input.name] = True
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self._key_state[event.input.name] = False
        return True
    
    def advance(self) -> torch.Tensor:
        """
        Get joint position deltas based on keyboard input.
        
        Returns:
            Joint position delta tensor [6] for (5 arm joints + gripper)
            
        Joint Index Mapping:
            delta[0] = Shoulder Pan (base rotation left/right)
            delta[1] = Shoulder Lift (arm up/down)
            delta[2] = Elbow Flex (forearm extend/retract)
            delta[3] = Wrist Flex (wrist up/down)
            delta[4] = Wrist Roll (wrist rotation)
            delta[5] = Gripper (open/close)
        
        Key Mappings (Change keys here if needed):
            Q/A = Shoulder Pan (delta[0])
            W/S = Shoulder Lift (delta[1])
            E/D = Elbow Flex (delta[2])
            C/V = Wrist Flex (delta[3])
            T/G = Wrist Roll (delta[4])
            Z/X = Gripper (delta[5])
        
        Special Keys:
            BACKSPACE = Reset environment (see line ~202)
            F = Blocked to prevent camera reset (see _on_keyboard_event)
            ESC = Quit (handled by Isaac Sim)
        """
        delta = torch.zeros(self.num_joints, device=self.device)
        
        # ============================================================
        # JOINT 0: Shoulder Pan (base rotation)
        # Keys: Q (positive) / A (negative)
        # ============================================================
        if self._key_state.get("Q", False):
            delta[0] = self.joint_delta * self.joint_sensitivity[0]
        elif self._key_state.get("A", False):
            delta[0] = -self.joint_delta * self.joint_sensitivity[0]
        
        # ============================================================
        # JOINT 1: Shoulder Lift (arm up/down)
        # Keys: W (positive/up) / S (negative/down)
        # ============================================================
        if self._key_state.get("W", False):
            delta[1] = self.joint_delta * self.joint_sensitivity[1]
        elif self._key_state.get("S", False):
            delta[1] = -self.joint_delta * self.joint_sensitivity[1]
        
        # ============================================================
        # JOINT 2: Elbow Flex (forearm extend/retract)
        # Keys: E (positive/extend) / D (negative/retract)
        # ============================================================
        if self._key_state.get("E", False):
            delta[2] = self.joint_delta * self.joint_sensitivity[2]
        elif self._key_state.get("D", False):
            delta[2] = -self.joint_delta * self.joint_sensitivity[2]
        
        # ============================================================
        # JOINT 3: Wrist Flex (wrist up/down)
        # Keys: C (positive) / V (negative)
        # Changed from R/V to C/V for user preference
        # ============================================================
        if self._key_state.get("C", False):
            delta[3] = self.joint_delta * self.joint_sensitivity[3]
        elif self._key_state.get("V", False):
            delta[3] = -self.joint_delta * self.joint_sensitivity[3]
        
        # ============================================================
        # JOINT 4: Wrist Roll (wrist rotation)
        # Keys: T (positive) / G (negative)
        # ============================================================
        if self._key_state.get("T", False):
            delta[4] = self.joint_delta * self.joint_sensitivity[4]
        elif self._key_state.get("G", False):
            delta[4] = -self.joint_delta * self.joint_sensitivity[4]
        
        # ============================================================
        # JOINT 5: Gripper (open/close)
        # Keys: Z (positive/open) / X (negative/close)
        # ============================================================
        if self._key_state.get("Z", False):
            delta[5] = self.joint_delta * self.joint_sensitivity[5]  # Open
        elif self._key_state.get("X", False):
            delta[5] = -self.joint_delta * self.joint_sensitivity[5]  # Close
        
        return delta
    
    def reset(self):
        """Reset joint positions."""
        self.joint_positions.zero_()
        self._key_state.clear()
    
    def __del__(self):
        """Clean up keyboard subscription."""
        if hasattr(self, '_sub_keyboard') and hasattr(self, '_input'):
            try:
                self._input.unsubscribe_to_keyboard_events(self._keyboard, self._sub_keyboard)
            except:
                pass  # Ignore cleanup errors


def main():
    """Run joint position teleoperation."""
    
    # parse configuration
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs)
    
    # modify configuration
    env_cfg.terminations.time_out = None
    if "Lift" in args_cli.task:
        # set the resampling time range to large number to avoid resampling
        env_cfg.commands.object_pose.resampling_time_range = (1.0e9, 1.0e9)
        # add termination condition for reaching the goal
        env_cfg.terminations.object_reached_goal = DoneTerm(func=mdp.object_reached_goal)
    
    # create environment
    env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
    
    # Get device from environment
    device = env.device
    
    # create joint controller with matching device
    controller = JointKeyboardController(num_joints=6, joint_delta=args_cli.joint_delta, device=device)
    
    # reset environment
    obs, _ = env.reset()
    controller.reset()
    
    # Initialize joint positions from robot's current state
    # Get initial joint positions from the environment
    robot_joint_pos = env.unwrapped.scene["robot"].data.joint_pos[:, :6].clone()
    current_joint_targets = robot_joint_pos[0].clone()  # Use first env's positions
    
    # track reset request
    should_reset = False
    
    print("Joint position teleoperation started!")
    print(f"Joint delta: {args_cli.joint_delta} radians per key press")
    print(f"Initial joint positions: {current_joint_targets}\n")
    
    # simulate environment
    while simulation_app.is_running():
        try:
            with torch.inference_mode():
                # Check for reset (BACKSPACE key - changed from SPACE)
                if controller._key_state.get("BACKSPACE", False):
                    should_reset = True
                    controller._key_state["BACKSPACE"] = False
                
                # get joint deltas from keyboard
                joint_delta = controller.advance()
                
                # Accumulate deltas into target positions
                current_joint_targets = current_joint_targets + joint_delta
                
                # create actions for all environments (use accumulated targets)
                actions = current_joint_targets.unsqueeze(0).repeat(env.num_envs, 1)
                
                # apply actions
                env.step(actions)
                
                # reset if requested
                if should_reset:
                    obs, _ = env.reset()
                    controller.reset()
                    # Re-initialize joint targets from reset positions
                    robot_joint_pos = env.unwrapped.scene["robot"].data.joint_pos[:, :6].clone()
                    current_joint_targets = robot_joint_pos[0].clone()
                    should_reset = False
                    print(f"Environment reset! New joint positions: {current_joint_targets}")
                    
        except Exception as e:
            print(f"Error during simulation: {e}")
            break
    
    # close environment
    env.close()
    # No need to explicitly call __del__, Python will handle it


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
