# Copyright (c) 2022-2025, Fan Yang and Per Frivik, ETH Zurich.
# All rights reserved.
#
# SPDX-License-Identifier: MIT

"""Configuration for the Go2W (Unitree Go2W) robot.

The following configuration parameters are available:

* :obj:`GO2W_CFG`: The Go2W robot with wheels.

Reference:
    The Go2W is a quadruped wheeled robot (4 legs + 4 wheels).
"""

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

# Local assets directory for this extension
_ASSETS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")

__all__ = ["GO2W_CFG"]


GO2W_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{_ASSETS_DIR}/Robots/Go2W/go2w_rsl.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=None,
            max_angular_velocity=None,
            max_depenetration_velocity=1.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=4, solver_velocity_iteration_count=0
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.38),
        joint_pos={
            ".*hip_joint": 0.0,
            "F.*thigh_joint": 0.5,
            "R.*thigh_joint": 0.8,
            ".*calf_joint": -1.5,
            ".*foot_joint": 0.0,
        },
    ),
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*hip_joint", ".*thigh_joint"],
            effort_limit_sim=23.5,
            velocity_limit_sim=30.1,
            stiffness={".*": 20.0},
            damping={".*": 0.5},
        ),
        "legs_calf": ImplicitActuatorCfg(
            joint_names_expr=[".*calf_joint"],
            effort_limit_sim=23.5,
            velocity_limit_sim=30.1,
            stiffness={".*": 20.0},
            damping={".*": 0.5},
        ),
        "wheels": ImplicitActuatorCfg(
            joint_names_expr=[".*foot_joint"],
            effort_limit_sim=23.5,
            velocity_limit_sim=50.0,
            stiffness={".*": 0.0},
            damping={".*": 0.5},
        ),
    },
    soft_joint_pos_limit_factor=0.95,
)
"""Configuration of Go2W robot using ImplicitActuatorCfg."""
