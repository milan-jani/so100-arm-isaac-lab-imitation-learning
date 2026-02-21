# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for SO-100 robotic arm from custom build."""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

##
# Configuration
##

SO100_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/robotics-mu/Downloads/SO100/so100/so100.usd",
        activate_contact_sensors=False,  # CRITICAL: Franka uses False - prevents penetration issues
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,  # Keep disabled for stability
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,  # Disable self collisions
            solver_position_iteration_count=8,  # Same as Franka
            solver_velocity_iteration_count=0,  # Same as Franka
        ),
        # collision_props removed - let defaults handle it like Franka
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        rot=(0.7071, 0, 0, 0.7071),  # 90° rotation - robot faces LEFT (towards cube)
        joint_pos={
            # SO-100 starting pose - safe neutral position within limits
            "shoulder_pan": 0.0,      # Forward direction
            "shoulder_lift": 0.5,     # CHANGED: Positive value (was -0.5, out of [0, 3.5])
            "elbow_flex": -0.5,       # CHANGED: Negative value (was 0.8, out of [-3.14, 0])
            "wrist_flex": 0.0,        # Neutral wrist
            "wrist_roll": 0.0,        # Neutral roll
            "gripper": 0.04,          # Same initial as Franka
        },
    ),
    actuators={
        "shoulder": ImplicitActuatorCfg(
            joint_names_expr=["shoulder_.*"],
            effort_limit_sim=87.0,
            velocity_limit_sim=2.175,
            stiffness=80.0,   # Same as Franka
            damping=4.0,      # Same as Franka
        ),
        "elbow": ImplicitActuatorCfg(
            joint_names_expr=["elbow_.*"],
            effort_limit_sim=87.0,
            velocity_limit_sim=2.175,
            stiffness=80.0,   # Same as Franka
            damping=4.0,      # Same as Franka
        ),
        "wrist": ImplicitActuatorCfg(
            joint_names_expr=["wrist_.*"],
            effort_limit_sim=12.0,
            velocity_limit_sim=2.61,
            stiffness=80.0,   # Same as Franka
            damping=4.0,      # Same as Franka
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["gripper"],
            effort_limit_sim=200.0,  # Same as Franka
            velocity_limit_sim=7.5,  # Fast open/close (was 0.2 Franka default — too slow for SO100 jaw)
            stiffness=2e3,    # Same as Franka
            damping=1e2,      # Same as Franka
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)

"""Configuration of SO-100 robotic arm."""
