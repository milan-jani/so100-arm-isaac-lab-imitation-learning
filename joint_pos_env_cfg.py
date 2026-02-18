# ============================================================================
# PATH: isaaclab/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/lift/config/so100/joint_pos_env_cfg.py
# ============================================================================

# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import isaaclab.sim as sim_utils  # Added for collision_props
from isaaclab.assets import RigidObjectCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.sim.spawners.shapes import CuboidCfg  # Cuboid for plate/tray
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from isaaclab_tasks.manager_based.manipulation.lift import mdp
from isaaclab_tasks.manager_based.manipulation.lift.lift_env_cfg import LiftEnvCfg

##
# Pre-defined configs
##
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip
from isaaclab_assets import SO100_CFG  # isort: skip


@configclass
class SO100CubeLiftEnvCfg(LiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set SO-100 as robot
        self.scene.robot = SO100_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Set actions for SO-100 - ALL JOINTS INCLUDING GRIPPER in one action
        # This enables incremental gripper control (jitna press utna move)
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"],
            scale=1.0,  # Direct position control
            use_default_offset=True
        )
        # Remove binary gripper action - now gripper is part of arm_action
        self.actions.gripper_action = None
        
        # Set the body name for the end effector
        self.commands.object_pose.body_name = "jaw"

        # Set Cube as object - spawn in front of SO-100 arm
        # USING FRANKA-LIKE CONFIG FOR STABILITY
        self.scene.object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.35, 0.0, 0.055], rot=[1, 0, 0, 0]),  # Closer and centered
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                scale=(0.5, 0.5, 0.5),
                # CRITICAL: Franka uses activate_contact_sensors=False!
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,  # Same as Franka
                    solver_velocity_iteration_count=1,   # Same as Franka
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
            ),
        )

        # Add Bucket - positioned to the side for cube placement
        # Add Tray/Plate - thin flat surface on floor for cube placement
        # Using thin cuboid as tray/plate that sits on ground
        self.scene.bucket = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Bucket",
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=[0.35, 0.25, 0.005],  # Right side, on floor (z=height/2 = 0.01/2)
                rot=[1, 0, 0, 0]
            ),
            spawn=CuboidCfg(
                size=(0.25, 0.25, 0.01),  # 25cm x 25cm x 1cm thin plate/tray
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(0.9, 0.7, 0.2),  # Golden/yellow plate
                    metallic=0.3,
                ),
                rigid_props=RigidBodyPropertiesCfg(
                    kinematic_enabled=True,  # Fixed plate on floor
                    disable_gravity=True,
                ),
                collision_props=sim_utils.CollisionPropertiesCfg(
                    collision_enabled=True,
                ),
            ),
        )

        # Listens to the required transforms for end-effector tracking
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base",  # SO-100 base link
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/jaw",  # SO-100 end-effector
                    name="end_effector",
                    offset=OffsetCfg(
                        pos=[0.0, 0.0, 0.05],  # Approximate gripper tip offset
                    ),
                ),
            ],
        )
        # =====================================================================
        # CAMERA IS DEFINED DIRECTLY IN THE USD FILE — not configured here.
        # =====================================================================
        # Camera transform & lens values (set inside USD):
        #   Position   : X=-0.18599, Y=-0.00463, Z=-0.05317
        #   Orientation: X=-88.002°, Y=-72.0°, Z=90.998°
        #   Focal Length: 14.0  |  Focus Distance: 400.0
        #
        # self.scene.wrist_camera = CameraCfg(
        #     prim_path="{ENV_REGEX_NS}/Robot/gripper/Camera",
        #     update_period=0.1,
        #     height=480,
        #     width=640,
        #     data_types=["rgb", "distance_to_image_plane"],
        #     spawn=sim_utils.PinholeCameraCfg(
        #         focal_length=14.0,
        #         focus_distance=400.0,
        #         horizontal_aperture=20.955,
        #         clipping_range=(0.1, 1.0e5),
        #     ),
        #     offset=CameraCfg.OffsetCfg(
        #         pos=(-0.18599, -0.00463, -0.05317),
        #         rot=...,  # X=-88.002°, Y=-72.0°, Z=90.998°
        #         convention="world",
        #     ),
        # )

        # Reduce cube spawn randomization range for SO-100's smaller reach
        # These values are OFFSETS added to initial position [0.4, 0.0, 0.055]
        # Final spawn range: X=[0.37-0.43], Y=[-0.05, 0.05], Z=[0.055]
        self.events.reset_object_position.params["pose_range"] = {
            "x": (-0.02, 0.02),   # ±3cm forward/backward → Final: 0.37 to 0.43m
            "y": (-0.05, 0.05),   # ±5cm left/right → Final: -0.05 to 0.05m (TIGHTER!)
            "z": (0.0, 0.0),      # No vertical variation (table height fixed)
        }


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
