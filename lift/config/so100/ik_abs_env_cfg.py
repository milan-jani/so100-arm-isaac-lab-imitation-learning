# ============================================================================
# PATH: isaaclab/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/lift/config/so100/ik_abs_env_cfg.py
# ============================================================================

# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.envs.mdp.actions.actions_cfg import JointPositionActionCfg
from isaaclab.utils import configclass

from . import joint_pos_env_cfg

##
# Pre-defined configs
##
from isaaclab_assets import SO100_CFG  # isort: skip


@configclass
class SO100CubeLiftEnvCfg(joint_pos_env_cfg.SO100CubeLiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set actions for the specific robot type (SO-100)
        # Using JOINT POSITION control for independent joint movements via keyboard
        # This gives direct control of each joint without IK solver issues
        self.actions.arm_action = JointPositionActionCfg(
            asset_name="robot",
            joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
            scale=0.5,  # Movement speed per key press (0.5 radians per step)
            use_default_offset=True,  # Use robot's current position as starting point
        )


@configclass
class SO100CubeLiftEnvCfg_PLAY(SO100CubeLiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
