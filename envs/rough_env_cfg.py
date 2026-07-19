# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.managers import TerminationTermCfg as DoneTerm

import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg, RewardsCfg

##
# Pre-defined configs
##
from isaaclab_assets.robots.robinion_cfg import ROBINION_CONFIG


import torch
from isaaclab.envs import ManagerBasedRLEnv

def joint_deviation_l2(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    asset = env.scene[asset_cfg.name]

    deviation = asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids]

    return torch.sum(torch.square(deviation), dim=-1)
    
def custom_backwards_hip_pitch_penalty(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:

    robot = env.scene[asset_cfg.name]
    
    joint_names = robot.data.joint_names
    
    r_pitch_idx = [i for i, name in enumerate(joint_names) if "r_hip_pitch" in name][0]
    l_pitch_idx = [i for i, name in enumerate(joint_names) if "l_hip_pitch" in name][0]
    
    r_pitch_pos = robot.data.joint_pos[:, r_pitch_idx]
    l_pitch_pos = robot.data.joint_pos[:, l_pitch_idx]
    
    r_penalty = torch.abs(torch.clamp(r_pitch_pos, max=0.0))
    
    l_penalty = torch.clamp(l_pitch_pos, min=0.0)
    
    total_penalty = r_penalty + l_penalty
    
    return total_penalty

@configclass
class H1Rewards(RewardsCfg):
    """Reward terms for the MDP."""

    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    #lin_vel_z_l2 = None
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0)
    
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_yaw_frame_exp,
        weight=1.5,
        params={"command_name": "base_velocity", "std": 0.45}, 
    )
    
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_world_exp, weight=1.0, params={"command_name": "base_velocity", "std": 0.5}
    )
    
    feet_air_time = RewTerm(
        func=mdp.feet_air_time_positive_biped,
        weight=0.7,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*foot.*"),
            "threshold": 0.32,
        },
    )
    
    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=-0.4,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*foot.*"),
            "asset_cfg": SceneEntityCfg("robot", body_names=".*foot.*"),
        },
    )
    
    dof_pos_limits = RewTerm(
        func=mdp.joint_pos_limits, 
        weight=-10.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*")} 
    )
    
    joint_deviation_hip = RewTerm(
        func=joint_deviation_l2,
        #func=mdp.joint_deviation_l1,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_yaw.*", ".*_hip_roll.*", ".*_ankle_roll.*"])},
    )
    
#    joint_deviation_ankles = RewTerm(
#        func=mdp.joint_deviation_l1,
#        weight=-0.5,
#        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_ankle_roll.*", ".*_ankle_pitch.*"])},
#    )
    
    joint_deviation_arms = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.2,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*shoulder.*", ".*elbow.*"])},
    )
    
    joint_deviation_torso = RewTerm(
        func=mdp.joint_deviation_l1, weight=-0.1, params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*torso.*")}
    )

    penalty_hip_pitch_backwards = RewTerm(
        func=custom_backwards_hip_pitch_penalty,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )

@configclass
class H1RoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    rewards: H1Rewards = H1Rewards()

    def __post_init__(self):
        super().__post_init__()
        
        if self.scene.contact_forces is not None:
            self.scene.contact_forces.prim_path = "{ENV_REGEX_NS}/Robot/robinion2s/.*"
            self.scene.contact_forces.body_names = [".*foot.*"]

        self.scene.robot = ROBINION_CONFIG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        
        if self.scene.height_scanner is not None:
            self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/robinion2s/upper_body_link"

        self.events.push_robot = None
        self.events.add_base_mass = None
        #self.events.reset_robot_joints.params["position_range"] = (0.0, 0.0)
        self.events.reset_robot_joints = None
        
        self.events.base_external_force_torque.params["asset_cfg"].body_names = [".*upper_body_link.*"]
        self.events.reset_base.params = {
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0), "y": (0.0, 0.0), "z": (0.0, 0.0),
                "roll": (0.0, 0.0), "pitch": (0.0, 0.0), "yaw": (0.0, 0.0),
            },
        }
        self.events.base_com = None

        self.rewards.undesired_contacts = None
        self.rewards.flat_orientation_l2.weight = -1.0
        self.rewards.dof_torques_l2.weight = -0.0002	#0.0
        self.rewards.action_rate_l2.weight = -0.05	#-0.005
        self.rewards.dof_acc_l2.weight = -2.5e-7	#-1.25e-7

        self.commands.base_velocity.ranges.lin_vel_x = (0.1, 1.5)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)
        
        self.terminations.base_contact = DoneTerm(
            func=mdp.illegal_contact,
            params={
                "sensor_cfg": SceneEntityCfg("contact_forces", body_names=[".*_body.*", ".*head.*", ".*arm.*"]),
                "threshold": 1.0,
            },
        )

        self.sim.physx.collision_stack_size = 256 * 1024 * 1024  
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 2 * 1024 * 1024  
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 2 * 1024 * 1024      


@configclass
class H1RoughEnvCfg_PLAY(H1RoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        
        self.sim.physx.collision_stack_size = 100 * 1024 * 1024  
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 1024 * 1024

        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.episode_length_s = 40.0
        self.scene.terrain.max_init_terrain_level = None
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False

        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 3.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)
        self.commands.base_velocity.ranges.heading = (0.0, 0.0)
        
        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None
