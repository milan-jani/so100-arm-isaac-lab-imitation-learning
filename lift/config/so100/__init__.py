# ============================================================================
# PATH: isaaclab/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/lift/config/so100/__init__.py
# ============================================================================

# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

##
# Register Gym environments.
##

##
# Joint Position Control
##

gym.register(
    id="Isaac-Lift-Cube-SO100-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.joint_pos_env_cfg:SO100CubeLiftEnvCfg",
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Lift-Cube-SO100-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.joint_pos_env_cfg:SO100CubeLiftEnvCfg_PLAY",
    },
    disable_env_checker=True,
)

##
# Inverse Kinematics - Absolute Pose Control (with Relative Mode for Keyboard)
##

gym.register(
    id="Isaac-Lift-Cube-SO100-IK-Abs-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.ik_abs_env_cfg:SO100CubeLiftEnvCfg",
    },
    disable_env_checker=True,
)
