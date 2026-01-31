# ============================================================================
# PATH: isaaclab/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/reach/config/so100/ik_abs_env_cfg.py
# ============================================================================

# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from isaaclab.utils import configclass

from . import joint_pos_env_cfg

##
# Pre-defined configs
##
from isaaclab_assets import SO100_CFG  # isort: skip


@configclass
class SO100ReachEnvCfg(joint_pos_env_cfg.SO100ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Switch to SO-100 robot (already set in parent, but can use high PD if needed)
        # For now, keeping the same SO100_CFG from parent
        
        # Set actions for IK control - keyboard sends delta commands
        # Using relative mode for keyboard compatibility
        self.actions.arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
            body_name="jaw",  # End-effector body
            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=True,  # Relative mode required for 5-DOF arm (SO-100)
                ik_method="dls"
            ),
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, 0.05]),  # Approximate gripper offset
            # Note: scale removed - not compatible with absolute mode
        )


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
