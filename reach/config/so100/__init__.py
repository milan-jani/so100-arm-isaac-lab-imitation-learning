# ============================================================================
# PATH: isaaclab/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/reach/config/so100/__init__.py
# ============================================================================

# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
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
