# ============================================================================
# PATH: isaaclab/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/reach/config/so100/joint_pos_env_cfg.py
# ============================================================================

# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

from isaaclab.utils import configclass

import isaaclab_tasks.manager_based.manipulation.reach.mdp as mdp
from isaaclab_tasks.manager_based.manipulation.reach.reach_env_cfg import ReachEnvCfg

##
# Pre-defined configs
##
from isaaclab_assets import SO100_CFG  # isort: skip


##
# Environment configuration
##


@configclass
class SO100ReachEnvCfg(ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # switch robot to SO-100
        self.scene.robot = SO100_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        
        # override rewards - use 'jaw' as end-effector
        self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = ["jaw"]
        self.rewards.end_effector_position_tracking_fine_grained.params["asset_cfg"].body_names = ["jaw"]
        self.rewards.end_effector_orientation_tracking.params["asset_cfg"].body_names = ["jaw"]
        
        # override actions - SO-100 joint pattern (exclude gripper for reach task)
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", 
            joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"], 
            scale=0.5, 
            use_default_offset=True
        )
        # override command generator body
        # end-effector is along z-direction
        self.commands.ee_pose.body_name = "jaw"
        self.commands.ee_pose.ranges.pitch = (math.pi, math.pi)


@configclass
class SO100ReachEnvCfg_PLAY(SO100ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
