import argparse
import os
import pickle
import shutil

from hexapod_env import HexEnv
from rsl_rl.runners import OnPolicyRunner

import genesis as gs


def get_train_cfg(exp_name, max_iterations):
    train_cfg_dict = {
        "algorithm": {
            "clip_param": 0.2,
            "desired_kl": 0.01,
            "entropy_coef": 0.01,
            "gamma": 0.99,
            "lam": 0.95,
            "learning_rate": 1e-7,
            "max_grad_norm": 1.0,
            "num_learning_epochs": 5,
            "num_mini_batches": 4,
            "schedule": "adaptive",
            "use_clipped_value_loss": True,
            "value_loss_coef": 1.0,
        },
        "init_member_classes": {},
        "policy": {
            "activation": "elu",
            "actor_hidden_dims": [512, 256, 128],
            "critic_hidden_dims": [512, 256, 128],
            "init_noise_std": 0.8,
        },
        "runner": {
            "algorithm_class_name": "PPO",
            "checkpoint": -1,
            "experiment_name": exp_name,
            "load_run": -1,
            "log_interval": 1,
            "max_iterations": max_iterations,
            "num_steps_per_env": 24,
            "policy_class_name": "ActorCritic",
            "record_interval": -1,
            "resume": False,
            "resume_path": None,
            "run_name": "",
            "runner_class_name": "runner_class_name",
            "save_interval": 50,
        },
        "runner_class_name": "OnPolicyRunner",
        "seed": 1,
    }

    return train_cfg_dict


def get_cfgs():
    env_cfg = {
        "num_actions": 18,
        # joint/link names
        "default_joint_angles": {
            'leg1_coxa': 0.0,
            'leg1_femur': 1.39626,
            'leg1_tibia': -1.5708,
            'leg2_coxa': 0.0,
            'leg2_femur': 1.39626,
            'leg2_tibia': -1.5708,
            'leg3_coxa': 0.0,
            'leg3_femur': 1.39626,
            'leg3_tibia': -1.5708,
            'leg4_coxa': 0.0,
            'leg4_femur': 1.39626,
            'leg4_tibia': -1.5708,
            'leg5_coxa': 0.0,
            'leg5_femur': 1.39626,
            'leg5_tibia': -1.5708,
            'leg6_coxa': 0.0,
            'leg6_femur': 1.39626,
            'leg6_tibia': -1.5708,
        },
        "dof_names": [
            'leg1_coxa', 'leg1_femur', 'leg1_tibia',
            'leg2_coxa', 'leg2_femur', 'leg2_tibia',
            'leg3_coxa', 'leg3_femur', 'leg3_tibia',
            'leg4_coxa', 'leg4_femur', 'leg4_tibia',
            'leg5_coxa', 'leg5_femur', 'leg5_tibia',
            'leg6_coxa', 'leg6_femur', 'leg6_tibia',
        ],
        # PD
        "kp": 120.0,
        "kd": 3.0,
        # termination
        "terminate_after_contacts_on": ["base_link"],
        "termination_if_roll_greater_than": 10,  # degree
        "termination_if_pitch_greater_than": 10,
        "termination_if_base_z_less_than": 0.02,
        # base pose
        "base_init_pos": [0.0, 0.0, 0.1],
        "base_init_quat": [1.0, 0.0, 0.0, 0.0],
        "episode_length_s": 20.0,
        "resampling_time_s": 4.0,
        "action_scale": 1.0,
        "simulate_action_latency": True,
        "clip_actions": 10.0,
        "clip_observations": 10.0,
    }
    obs_cfg = {
        "num_obs": 65,
        "obs_scales": {
            "lin_vel": 1.0,
            "ang_vel": 1.0,
            "dof_pos": 1.0,
            "dof_vel": 1.0,
        },
        "add_noise": False,
        "noise_level": 1.0,
        "noise_scale": {
            "dof_pos": 0.01,
            "dof_vel": 1.5,
            "lin_vel": 0.1,
            "ang_vel": 0.2,
            "gravity": 0.05,
        },
    }
    reward_cfg = {
        "tracking_sigma": 0.25,
        "base_height_target": 0.04,
        "feet_height_target": 0.125,
        "reward_scales": {
            "tracking_lin_vel": 1.0,
            "tracking_ang_vel": 0.5,
            "alive": 0.15,
            "gait_contact": 0.18,
            "gait_swing": -0.18,
            "lin_vel_z": -2.0,
            "ang_vel_xy": -0.05,
            "base_height": -10.0,
            "action_rate": -0.01,
            "contact_no_vel": -0.2,
            "feet_swing_height": -20.0,
            "orientation": -10.0,
            "dof_vel": -0.001,
            "similar_to_default": -0.1
        },
    }
    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0.0, 0.0],
        "lin_vel_y_range": [0.25, 0.25],
        "ang_vel_range": [0, 0],
    }
    domain_rand_cfg = {
        'randomize_friction': False,
        'friction_range': [0.01, 1.25],
        'randomize_mass': False,
        'added_mass_range': [-1.0, 3.0],
        'rand_interval_s': 3.5, # seconds
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg, domain_rand_cfg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="hex-walking")
    parser.add_argument("-B", "--num_envs", type=int, default=128)
    parser.add_argument("--max_iterations", type=int, default=100)
    args = parser.parse_args()

    gs.init(logging_level="warning")

    log_dir = f"logs/{args.exp_name}"
    env_cfg, obs_cfg, reward_cfg, command_cfg, domain_rand_cfg = get_cfgs()
    train_cfg = get_train_cfg(args.exp_name, args.max_iterations)

    if os.path.exists(log_dir):
        shutil.rmtree(log_dir)
    os.makedirs(log_dir, exist_ok=True)

    env = HexEnv(
        num_envs=args.num_envs, env_cfg=env_cfg, obs_cfg=obs_cfg,
        reward_cfg=reward_cfg, command_cfg=command_cfg,
        domain_rand_cfg=domain_rand_cfg
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir, device="cuda:0")

    pickle.dump(
        [env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg, domain_rand_cfg],
        open(f"{log_dir}/cfgs.pkl", "wb"),
    )

    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)


if __name__ == "__main__":
    main()

"""
# training
python examples/locomotion/hex_train.py
"""