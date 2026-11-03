# Copyright (c) 2022-2026, Fan Yang and Per Frivik, ETH Zurich.
# All rights reserved.
#
# SPDX-License-Identifier: MIT

"""RSL-RL agent configurations for Go2W navigation tasks (SRU-Odin 5-Phase Curriculum)."""

from isaaclab.utils import configclass

from isaaclab_nav_task.navigation.config.rl_cfg import (
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticCfg,
    RslRlPpoAlgorithmCfg,
)


# =============================================================================
# Base Runner Definitions
# =============================================================================

@configclass
class Go2WNavBaseRunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24  # 4.8s 轨迹窗口，确保 MemorySRU 时序充分展开
    save_interval = 200
    logger = "tensorboard"
    seed = 42
    wandb_project = "isaaclab_nav_go2w"
    empirical_normalization = False
    reward_shifting_value = 0.05
    policy = RslRlPpoActorCriticCfg(
        class_name="ActorCriticSRU",
        init_noise_std=1.0,
        actor_hidden_dims=[512, 256, 128],
        critic_hidden_dims=[512, 256, 128],
        activation="elu",
        rnn_hidden_size=512,
        rnn_type="lstm_sru",
        rnn_num_layers=1,
        dropout=0.2,
        num_cameras=1,
        image_input_dims=(64, 5, 8),  # 64 channels * 5 * 8 = 2560
        height_input_dims=(64, 7, 7),  # 64 channels * 7 * 7 = 3136
    )


# =============================================================================
# MDPO 5-Phase Runners (Primary Algorithm, strictly aligned with SRU-Odin B2W MDPO)
# =============================================================================

@configclass
class Go2WNavMDPORunnerPhase1Cfg(Go2WNavBaseRunnerCfg):
    """Phase 1: Cold start exploration (0 -> 500 iters)."""
    max_iterations = 500
    save_interval = 250
    experiment_name = "go2w_navigation_mdpo_phase1"
    algorithm = RslRlPpoAlgorithmCfg(
        class_name="MDPO",
        value_loss_coef=0.02,
        use_clipped_value_loss=True,
        clip_param=0.2,
        value_clip_param=0.2,
        entropy_coef=0.005,       # 维持充分的冷启动探索度
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="exponential",   # 标准指数平滑衰减
        gamma=0.995,              # 贴合 60s (300 步) 单回合的高效信用分配
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


@configclass
class Go2WNavMDPORunnerPhase2Cfg(Go2WNavMDPORunnerPhase1Cfg):
    """Phase 2: Stabilization & precision stopping (500 -> 1,500 iters, +1000 iters; P1+P2=1500 total)."""
    max_iterations = 1000
    save_interval = 250
    experiment_name = "go2w_navigation_mdpo_phase2"
    def __post_init__(self):
        super().__post_init__()
        self.algorithm.entropy_coef = 0.003


@configclass
class Go2WNavMDPORunnerPhase3Cfg(Go2WNavMDPORunnerPhase1Cfg):
    """Phase 3: Stairs (10-15cm) & moderate mazes (1,500 -> 3,000 iters, +1500 iters)."""
    max_iterations = 1500
    save_interval = 250
    experiment_name = "go2w_navigation_mdpo_phase3"
    def __post_init__(self):
        super().__post_init__()
        self.algorithm.entropy_coef = 0.003


@configclass
class Go2WNavMDPORunnerPhase4Cfg(Go2WNavMDPORunnerPhase1Cfg):
    """Phase 4: Full 20cm stairs, pits, and dense mazes (3,000 -> 4,500 iters, +1500 iters)."""
    max_iterations = 1500
    save_interval = 250
    experiment_name = "go2w_navigation_mdpo_phase4"
    def __post_init__(self):
        super().__post_init__()
        # 1. 学习率平滑余弦衰减：从 1.0e-3 降至 1.0e-4
        self.algorithm.schedule = "cosine"
        self.algorithm.learning_rate = 1.0e-3
        self.algorithm.min_learning_rate = 1.0e-4

        # 2. 策略熵平滑衰减：前 200 轮预热保持 0.003 探索深坑/20cm台阶，随后余弦退火至 0.001 紧致收敛
        self.algorithm.entropy_coef = 0.003
        self.algorithm.min_entropy_coef = 0.001
        self.algorithm.entropy_hold_iters = 200
        self.algorithm.entropy_decay_span = 1300
        self.algorithm.entropy_schedule = "cosine"

        # 3. PPO Clip 参数平滑余弦衰减：从 0.20 降至 0.15，后期收紧信任域消除抖动
        self.algorithm.clip_param = 0.20
        self.algorithm.value_clip_param = 0.20
        self.algorithm.min_clip_param = 0.15
        self.algorithm.clip_schedule = "cosine"


@configclass
class Go2WNavMDPORunnerPhase5Cfg(Go2WNavMDPORunnerPhase1Cfg):
    """Phase 5: Trajectory smoothing & fine-grained refinement (4,500 -> 6,000 iters, +1500 iters; P4+P5=3000 total)."""
    max_iterations = 1500
    save_interval = 250
    experiment_name = "go2w_navigation_mdpo_phase5"
    def __post_init__(self):
        super().__post_init__()
        self.algorithm.entropy_coef = 0.002  # 收紧探索，压制指令抖动
        self.algorithm.desired_kl = 0.01
        self.algorithm.clip_param = 0.15
        self.algorithm.value_clip_param = 0.15
        self.algorithm.learning_rate = 5.0e-4


@configclass
class Go2WNavMDPORunnerPureMazeCfg(Go2WNavMDPORunnerPhase5Cfg):
    """Pure-maze corridor refinement."""
    experiment_name = "go2w_navigation_mdpo_puremaze"


@configclass
class Go2WNavMDPORunnerDevCfg(Go2WNavMDPORunnerPhase1Cfg):
    """Development / smoke testing."""
    max_iterations = 300
    experiment_name = "go2w_navigation_mdpo_dev"


# =============================================================================
# PPO 5-Phase Runners (Baseline Algorithm)
# =============================================================================

@configclass
class Go2WNavPPORunnerPhase1Cfg(Go2WNavBaseRunnerCfg):
    """Phase 1: Cold start exploration (0 -> 500 iters)."""
    max_iterations = 500
    save_interval = 250
    experiment_name = "go2w_navigation_ppo_phase1"
    algorithm = RslRlPpoAlgorithmCfg(
        class_name="PPO",
        value_loss_coef=0.02,
        use_clipped_value_loss=True,
        clip_param=0.15,
        value_clip_param=0.15,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.995,
        lam=0.95,
        desired_kl=0.02,
        max_grad_norm=0.8,
    )


@configclass
class Go2WNavPPORunnerPhase2Cfg(Go2WNavPPORunnerPhase1Cfg):
    """Phase 2: Stabilization & precision stopping (500 -> 1,500 iters, +1000 iters; P1+P2=1500 total)."""
    max_iterations = 1000
    save_interval = 250
    experiment_name = "go2w_navigation_ppo_phase2"
    def __post_init__(self):
        super().__post_init__()
        self.algorithm.entropy_coef = 0.003


@configclass
class Go2WNavPPORunnerPhase3Cfg(Go2WNavPPORunnerPhase1Cfg):
    """Phase 3: Stairs (10-15cm) & moderate mazes (1,500 -> 3,000 iters, +1500 iters)."""
    max_iterations = 1500
    save_interval = 250
    experiment_name = "go2w_navigation_ppo_phase3"
    def __post_init__(self):
        super().__post_init__()
        self.algorithm.entropy_coef = 0.003


@configclass
class Go2WNavPPORunnerPhase4Cfg(Go2WNavPPORunnerPhase1Cfg):
    """Phase 4: Full 20cm stairs, pits, and dense mazes (3,000 -> 4,500 iters, +1500 iters)."""
    max_iterations = 1500
    save_interval = 250
    experiment_name = "go2w_navigation_ppo_phase4"
    def __post_init__(self):
        super().__post_init__()
        self.algorithm.entropy_coef = 0.003


@configclass
class Go2WNavPPORunnerPhase5Cfg(Go2WNavPPORunnerPhase1Cfg):
    """Phase 5: Trajectory smoothing & fine-grained refinement (4,500 -> 6,000 iters, +1500 iters; P4+P5=3000 total)."""
    max_iterations = 1500
    save_interval = 250
    experiment_name = "go2w_navigation_ppo_phase5"
    def __post_init__(self):
        super().__post_init__()
        self.algorithm.entropy_coef = 0.002
        self.algorithm.desired_kl = 0.015
        self.algorithm.clip_param = 0.12
        self.algorithm.value_clip_param = 0.12
        self.algorithm.learning_rate = 5.0e-4


@configclass
class Go2WNavPPORunnerPureMazeCfg(Go2WNavPPORunnerPhase5Cfg):
    """Pure-maze corridor refinement."""
    experiment_name = "go2w_navigation_ppo_puremaze"


@configclass
class Go2WNavPPORunnerDevCfg(Go2WNavPPORunnerPhase1Cfg):
    """Development / smoke testing."""
    max_iterations = 300
    experiment_name = "go2w_navigation_ppo_dev"


# =============================================================================
# Warm-Start Fine-Tuning Runner (Strictly aligned with SRU-Odin Go2NavPPORunnerCfg)
# =============================================================================

@configclass
class Go2WNavPPOFineTuneRunnerCfg(Go2WNavBaseRunnerCfg):
    """PPO runner configuration for Go2W navigation, tuned for fine-tuning from a B2W checkpoint."""

    num_steps_per_env = 16
    max_iterations = 2500
    save_interval = 250
    logger = "tensorboard"
    seed = 42
    wandb_project = "isaaclab_nav_go2w"
    experiment_name = "go2w_navigation_ppo_ft_from_b2w"
    empirical_normalization = False
    reward_shifting_value = 0.05

    policy = RslRlPpoActorCriticCfg(
        class_name="ActorCriticSRU",
        init_noise_std=0.5,
        actor_hidden_dims=[512, 256, 128],
        critic_hidden_dims=[512, 256, 128],
        activation="elu",
        rnn_hidden_size=512,
        rnn_type="lstm_sru",
        rnn_num_layers=1,
        dropout=0.2,
        num_cameras=1,
        image_input_dims=(64, 5, 8),
        height_input_dims=(64, 7, 7),
    )

    algorithm = RslRlPpoAlgorithmCfg(
        class_name="PPO",
        value_loss_coef=0.05,
        use_clipped_value_loss=True,
        clip_param=0.1,
        value_clip_param=0.1,
        entropy_coef=0.001,
        num_learning_epochs=3,
        num_mini_batches=4,
        learning_rate=3.0e-4,
        schedule="adaptive",
        gamma=0.995,
        lam=0.95,
        desired_kl=0.005,
        max_grad_norm=0.5,
    )


@configclass
class Go2WNavPPOLowObsRunnerCfg(Go2WNavPPOFineTuneRunnerCfg):
    """PPO runner for Go2W navigation low obstacle (40-80cm) fine-tuning from nav_policy.onnx."""
    max_iterations = 2000
    save_interval = 250
    experiment_name = "go2w_navigation_ppo_low_obs"

    def __post_init__(self):
        super().__post_init__()
        # Conservative fine-tuning parameters to preserve pre-trained navigation capabilities
        self.algorithm.learning_rate = 1.5e-4
        self.algorithm.clip_param = 0.10
        self.algorithm.value_clip_param = 0.10
        self.algorithm.desired_kl = 0.005
        self.policy.init_noise_std = 0.35


# =============================================================================
# Arrival Heading Control Fine-Tuning Runners (strictly aligned with SRU-Odin)
# =============================================================================

@configclass
class Go2WNavMDPOHeadingRunnerCfg(Go2WNavBaseRunnerCfg):
    """Conservative MDPO fine-tuning runner for arrival heading control from warmstart checkpoint."""
    max_iterations = 800
    save_interval = 100
    experiment_name = "go2w_navigation_mdpo_heading"

    def __post_init__(self):
        super().__post_init__()
        self.policy.init_noise_std = 0.35
        self.algorithm = RslRlPpoAlgorithmCfg(
            class_name="MDPO",
            value_loss_coef=0.02,
            use_clipped_value_loss=True,
            clip_param=0.10,
            value_clip_param=0.10,
            entropy_coef=0.001,
            num_learning_epochs=3,
            num_mini_batches=4,
            learning_rate=1.5e-4,
            schedule="adaptive",
            gamma=0.995,
            lam=0.95,
            desired_kl=0.005,
            max_grad_norm=0.5,
        )


@configclass
class Go2WNavPPOHeadingRunnerCfg(Go2WNavBaseRunnerCfg):
    """Conservative PPO fine-tuning runner for arrival heading control from warmstart checkpoint."""
    max_iterations = 800
    save_interval = 100
    experiment_name = "go2w_navigation_ppo_heading"

    def __post_init__(self):
        super().__post_init__()
        self.policy.init_noise_std = 0.35
        self.algorithm = RslRlPpoAlgorithmCfg(
            class_name="PPO",
            value_loss_coef=0.05,
            use_clipped_value_loss=True,
            clip_param=0.10,
            value_clip_param=0.10,
            entropy_coef=0.001,
            num_learning_epochs=3,
            num_mini_batches=4,
            learning_rate=1.5e-4,
            schedule="adaptive",
            gamma=0.995,
            lam=0.95,
            desired_kl=0.005,
            max_grad_norm=0.5,
        )


# =============================================================================
# Backward Compatibility Aliases & Default Runners
# =============================================================================

Go2WNavMDPORunnerCfg = Go2WNavMDPORunnerPhase1Cfg
Go2WNavPPORunnerCfg = Go2WNavPPORunnerPhase1Cfg


