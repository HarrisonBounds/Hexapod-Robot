import torch
import math
import numpy as np
import genesis as gs
from genesis.utils.geom import quat_to_xyz, xyz_to_quat, transform_by_quat, inv_quat, transform_quat_by_quat
import random

def gs_rand_float(lower, upper, shape, device):
    return (upper - lower) * torch.rand(size=shape, device=device) + lower

class HexEnv:
    def __init__(self, num_envs, env_cfg, obs_cfg, reward_cfg, command_cfg, domain_rand_cfg, show_viewer=False, device="cuda"):
        self.device = torch.device(device)

        self.num_envs = num_envs
        self.num_obs = obs_cfg["num_obs"]
        self.num_privileged_obs = None
        self.num_actions = env_cfg["num_actions"]
        self.num_commands = command_cfg["num_commands"]

        self.simulate_action_latency = False  # there is a 1 step latency on unitree real robot
        self.dt = 0.02 # Control Frequency is 50Hz on Unitree Go2 Real Robot
        self.max_episode_length = math.ceil(env_cfg["episode_length_s"] / self.dt)

        self.env_cfg = env_cfg
        self.obs_cfg = obs_cfg
        self.reward_cfg = reward_cfg
        self.command_cfg = command_cfg
        self.domain_rand_cfg = domain_rand_cfg

        self.obs_scales = obs_cfg["obs_scales"]
        self.noise_scales = obs_cfg["noise_scale"]
        self.noise_level = obs_cfg["noise_level"]
        self.reward_scales = reward_cfg["reward_scales"]

        # create scene
        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(dt=self.dt, substeps=2),
            viewer_options=gs.options.ViewerOptions(
                max_FPS=int(0.5 / self.dt),
                camera_pos=(2.0, 0.0, 2.5),
                camera_lookat=(0.0, 0.0, 0.5),
                camera_fov=40,
            ),
            vis_options=gs.options.VisOptions(n_rendered_envs=1),
            rigid_options=gs.options.RigidOptions(
                dt=self.dt,
                constraint_solver=gs.constraint_solver.Newton,
                enable_collision=True,
                enable_self_collision=True,
                enable_joint_limit=True,
            ),
            show_viewer=show_viewer,
        )

        # add plain
        self.plane = self.scene.add_entity(gs.morphs.URDF(file="urdf/plane/plane.urdf", fixed=True))

        # add robot
        self.base_init_pos = torch.tensor(self.env_cfg["base_init_pos"], device=self.device)
        self.base_init_quat = torch.tensor(self.env_cfg["base_init_quat"], device=self.device)
        self.inv_base_init_quat = inv_quat(self.base_init_quat)
        self.robot = self.scene.add_entity(
            gs.morphs.URDF(
                file="urdf/robot.urdf",
                pos=self.base_init_pos.cpu().numpy(),
                quat=self.base_init_quat.cpu().numpy(),
            ),
        )

        # build
        self.scene.build(n_envs=num_envs)

        # names to indices
        self.motor_dofs = [self.robot.get_joint(name).dof_idx_local for name in self.env_cfg["dof_names"]]

        # PD control parameters
        self.robot.set_dofs_kp([self.env_cfg["kp"]] * self.num_actions, self.motor_dofs)
        self.robot.set_dofs_kv([self.env_cfg["kd"]] * self.num_actions, self.motor_dofs)

        # prepare reward functions and multiply reward scales by dt
        self.reward_functions, self.episode_sums = dict(), dict()
        for name in self.reward_scales.keys():
            self.reward_scales[name] *= self.dt
            self.reward_functions[name] = getattr(self, "_reward_" + name)
            self.episode_sums[name] = torch.zeros((self.num_envs,), device=self.device, dtype=gs.tc_float)

        # initialize buffers
        self.base_lin_vel = torch.zeros((self.num_envs, 3), device=self.device, dtype=gs.tc_float)
        self.base_ang_vel = torch.zeros((self.num_envs, 3), device=self.device, dtype=gs.tc_float)
        self.projected_gravity = torch.zeros((self.num_envs, 3), device=self.device, dtype=gs.tc_float)
        self.global_gravity = torch.tensor([0.0, 0.0, -1.0], device=self.device, dtype=gs.tc_float).repeat(
            self.num_envs, 1
        )
        self.obs_buf = torch.zeros((self.num_envs, self.num_obs), device=self.device, dtype=gs.tc_float)
        self.rew_buf = torch.zeros((self.num_envs,), device=self.device, dtype=gs.tc_float)
        self.reset_buf = torch.ones((self.num_envs,), device=self.device, dtype=gs.tc_int)
        self.episode_length_buf = torch.zeros((self.num_envs,), device=self.device, dtype=gs.tc_int)
        self.commands = torch.zeros((self.num_envs, self.num_commands), device=self.device, dtype=gs.tc_float)
        self.commands_scale = torch.tensor(
            [self.obs_scales["lin_vel"], self.obs_scales["lin_vel"], self.obs_scales["ang_vel"]],
            device=self.device,
            dtype=gs.tc_float,
        )
        self.actions = torch.zeros((self.num_envs, self.num_actions), device=self.device, dtype=gs.tc_float)
        self.last_actions = torch.zeros_like(self.actions)
        self.dof_pos = torch.zeros_like(self.actions)
        self.dof_vel = torch.zeros_like(self.actions)
        self.last_dof_vel = torch.zeros_like(self.actions)
        self.base_pos = torch.zeros((self.num_envs, 3), device=self.device, dtype=gs.tc_float)
        self.base_quat = torch.zeros((self.num_envs, 4), device=self.device, dtype=gs.tc_float)
        self.default_dof_pos = torch.tensor(
            [self.env_cfg["default_joint_angles"][name] for name in self.env_cfg["dof_names"]],
            device=self.device,
            dtype=gs.tc_float,
        )
        self.extras = dict()  # extra information for logging

        # Modified Steps
        self.noise_scale_vec = self.get_noise_scale_vec()
        self.contact_forces = self.robot.get_links_net_contact_force()

        self.leg1_foot_link = self.robot.get_link(name='tibia_assembly')
        self.leg2_foot_link = self.robot.get_link(name='tibia_assembly_2')
        self.leg3_foot_link = self.robot.get_link(name='tibia_assembly_3')
        self.leg4_foot_link = self.robot.get_link(name='tibia_assembly_4')
        self.leg5_foot_link = self.robot.get_link(name='tibia_assembly_5')
        self.leg6_foot_link = self.robot.get_link(name='tibia_assembly_6')

        self.leg1_foot_id_local = self.leg1_foot_link.idx_local
        self.leg2_foot_id_local = self.leg2_foot_link.idx_local
        self.leg3_foot_id_local = self.leg3_foot_link.idx_local
        self.leg4_foot_id_local = self.leg4_foot_link.idx_local
        self.leg5_foot_id_local = self.leg5_foot_link.idx_local
        self.leg6_foot_id_local = self.leg6_foot_link.idx_local

        self.feet_indices = [self.leg1_foot_id_local,
                             self.leg2_foot_id_local,
                             self.leg3_foot_id_local,
                             self.leg4_foot_id_local,
                             self.leg5_foot_id_local,
                             self.leg6_foot_id_local]

        self.feet_num = len(self.feet_indices)
        self.links_vel = self.robot.get_links_vel()
        self.feet_vel = self.links_vel[:, self.feet_indices, :]
        self.links_pos = self.robot.get_links_pos()
        self.feet_pos = self.links_pos[:, self.feet_indices, :]
        # Gait Phase
        period = 0.8
        offset = 0.5
        self.phase = (self.episode_length_buf * self.dt) % period / period
        self.phase_left = self.phase
        self.phase_right = (self.phase + offset) % 1
        self.leg_phase = torch.cat([self.phase_left.unsqueeze(1), self.phase_right.unsqueeze(1),
                                    self.phase_left.unsqueeze(1), self.phase_right.unsqueeze(1),
                                    self.phase_left.unsqueeze(1), self.phase_right.unsqueeze(1)], dim=-1)
        self.sin_phase = torch.sin(2 * np.pi * self.phase ).unsqueeze(1)
        self.cos_phase = torch.cos(2 * np.pi * self.phase ).unsqueeze(1)
        self.base_link = self.robot.get_link(name='base_link')
        self.base_mass = self.base_link.get_mass()
        self.base_id_local = self.base_link.idx_local
        self.base_pos = self.links_pos[:, self.base_id_local, :]
        self.original_links_mass = []
        self.counter = 0

        termination_contact_names = self.env_cfg["terminate_after_contacts_on"]
        self.termination_contact_indices = []
        for name in termination_contact_names:
            link = self.robot.get_link(name)
            link_id_local = link.idx_local
            self.termination_contact_indices.append(link_id_local)

        # friction_ratio : torch.Tensor, shape (n_envs, n_links)
        self.robot.set_friction_ratio(friction_ratio=torch.ones(self.num_envs, self.robot.n_links, device=self.device) * 2.0,
                                      link_indices=np.arange(0, self.robot.n_links))
        self.plane.set_friction_ratio(friction_ratio=torch.ones(self.num_envs, self.robot.n_links, device=self.device) * 2.0,
                                      link_indices=np.arange(0, self.plane.n_links))

    def _resample_commands(self, envs_idx):
        # resample commands
        self.commands[envs_idx, 0] = gs_rand_float(*self.command_cfg["lin_vel_x_range"], (len(envs_idx),), self.device)
        self.commands[envs_idx, 1] = gs_rand_float(*self.command_cfg["lin_vel_y_range"], (len(envs_idx),), self.device)
        self.commands[envs_idx, 2] = gs_rand_float(*self.command_cfg["ang_vel_range"], (len(envs_idx),), self.device)

    def step(self, actions):
        # clip actions
        self.actions = torch.clip(actions, -self.env_cfg["clip_actions"], self.env_cfg["clip_actions"])
        # execute actions
        exec_actions = self.last_actions if self.simulate_action_latency else self.actions
        target_dof_pos = exec_actions * self.env_cfg["action_scale"] + self.default_dof_pos
        self.robot.control_dofs_position(target_dof_pos, self.motor_dofs)
        self.scene.step()

        # update buffers
        self.episode_length_buf += 1
        self.base_pos[:] = self.robot.get_pos()
        # For unknown reasons, it gets NaN values in self.robot.get_*() sometimes
        if torch.isnan(self.base_pos).any():
            nan_envs = torch.isnan(self.base_pos).any(dim=1).nonzero(as_tuple=False).flatten()
            self.reset_idx(nan_envs)
        self.base_quat[:] = self.robot.get_quat()
        self.base_euler = quat_to_xyz(
            transform_quat_by_quat(torch.ones_like(self.base_quat) * self.inv_base_init_quat, self.base_quat)
        )
        inv_base_quat = inv_quat(self.base_quat)
        self.base_lin_vel[:] = transform_by_quat(self.robot.get_vel(), inv_base_quat)
        self.base_ang_vel[:] = transform_by_quat(self.robot.get_ang(), inv_base_quat)
        self.projected_gravity = transform_by_quat(self.global_gravity, inv_base_quat)
        self.dof_pos[:] = self.robot.get_dofs_position(self.motor_dofs)
        self.dof_vel[:] = self.robot.get_dofs_velocity(self.motor_dofs)

        # resample commands
        envs_idx = (
            (self.episode_length_buf % int(self.env_cfg["resampling_time_s"] / self.dt) == 0)
            .nonzero(as_tuple=False)
            .flatten()
        )
        self._resample_commands(envs_idx)

        # check termination and reset
        self.base_pos = self.links_pos[:, self.base_id_local, :]
        self.reset_buf = self.episode_length_buf > self.max_episode_length
        self.reset_buf |= torch.abs(self.base_euler[:, 1]) > self.env_cfg["termination_if_pitch_greater_than"]
        self.reset_buf |= torch.abs(self.base_euler[:, 0]) > self.env_cfg["termination_if_roll_greater_than"]
        self.reset_buf |= torch.abs(self.base_pos[:, 2]) < self.env_cfg["termination_if_base_z_less_than"]

        time_out_idx = (self.episode_length_buf > self.max_episode_length).nonzero(as_tuple=False).flatten()
        self.extras["time_outs"] = torch.zeros_like(self.reset_buf, device=self.device, dtype=gs.tc_float)
        self.extras["time_outs"][time_out_idx] = 1.0

        self.reset_idx(self.reset_buf.nonzero(as_tuple=False).flatten())

        # Domain Randomization
        if (self.domain_rand_cfg['randomize_friction']):
            self.randomize_friction()

        if (self.domain_rand_cfg['randomize_mass']):
            self.randomize_mass()

        # Modified Stepd For G1 Robot
        self.contact_forces = self.robot.get_links_net_contact_force()

        self.leg1_foot_link = self.robot.get_link(name='tibia_assembly')
        self.leg2_foot_link = self.robot.get_link(name='tibia_assembly_2')
        self.leg3_foot_link = self.robot.get_link(name='tibia_assembly_3')
        self.leg4_foot_link = self.robot.get_link(name='tibia_assembly_4')
        self.leg5_foot_link = self.robot.get_link(name='tibia_assembly_5')
        self.leg6_foot_link = self.robot.get_link(name='tibia_assembly_6')

        self.leg1_foot_id_local = self.leg1_foot_link.idx_local
        self.leg2_foot_id_local = self.leg2_foot_link.idx_local
        self.leg3_foot_id_local = self.leg3_foot_link.idx_local
        self.leg4_foot_id_local = self.leg4_foot_link.idx_local
        self.leg5_foot_id_local = self.leg5_foot_link.idx_local
        self.leg6_foot_id_local = self.leg6_foot_link.idx_local

        self.feet_indices = [self.leg1_foot_id_local,
                             self.leg2_foot_id_local,
                             self.leg3_foot_id_local,
                             self.leg4_foot_id_local,
                             self.leg5_foot_id_local,
                             self.leg6_foot_id_local]

        self.feet_num = len(self.feet_indices)
        self.links_vel = self.robot.get_links_vel()
        self.feet_vel = self.links_vel[:, self.feet_indices, :]
        self.links_pos = self.robot.get_links_pos()
        self.feet_pos = self.links_pos[:, self.feet_indices, :]
        # Gaits Phase
        period = 0.8
        offset = 0.5
        self.phase = (self.episode_length_buf * self.dt) % period / period
        self.phase_left = self.phase
        self.phase_right = (self.phase + offset) % 1
        self.leg_phase = torch.cat([self.phase_left.unsqueeze(1), self.phase_right.unsqueeze(1),
                                    self.phase_left.unsqueeze(1), self.phase_right.unsqueeze(1),
                                    self.phase_left.unsqueeze(1), self.phase_right.unsqueeze(1)], dim=-1)
        self.sin_phase = torch.sin(2 * np.pi * self.phase ).unsqueeze(1)
        self.cos_phase = torch.cos(2 * np.pi * self.phase ).unsqueeze(1)

        # compute reward
        self.rew_buf[:] = 0.0
        for name, reward_func in self.reward_functions.items():
            rew = reward_func() * self.reward_scales[name]
            self.rew_buf += rew
            self.episode_sums[name] += rew

        # compute observations
        self.obs_buf = torch.cat(
            [
                self.base_ang_vel * self.obs_scales["ang_vel"],  # 3
                self.projected_gravity,  # 3
                self.commands * self.commands_scale,  # 3
                (self.dof_pos - self.default_dof_pos) * self.obs_scales["dof_pos"],  # 18
                self.dof_vel * self.obs_scales["dof_vel"],  # 18
                self.actions,  # 18
                self.sin_phase, # 1
                self.cos_phase, # 1
            ],
            axis=-1,
        )
        # add noise to observations
        if self.obs_cfg["add_noise"]:
            self.obs_buf += (2 * torch.rand_like(self.obs_buf) - 1) * self.noise_scale_vec
        # clip observations
        self.obs_buf = torch.clip(self.obs_buf, -self.env_cfg["clip_observations"], self.env_cfg["clip_observations"])

        self.last_actions[:] = self.actions[:]
        self.last_dof_vel[:] = self.dof_vel[:]

        self.counter += 1

        return self.obs_buf, None, self.rew_buf, self.reset_buf, self.extras

    def randomize_friction(self):
        if(self.counter % int(self.domain_rand_cfg['rand_interval_s']/self.dt) == 0):
            friction_range = self.domain_rand_cfg['friction_range']
            self.robot.set_friction_ratio(
                friction_ratio = friction_range[0] +\
                    torch.rand(self.num_envs, self.robot.n_links) *\
                    (friction_range[1] - friction_range[0]),
                link_indices=np.arange(0, self.robot.n_links))
            self.plane.set_friction_ratio(
                friction_ratio = friction_range[0] +\
                    torch.rand(self.num_envs, self.plane.n_links) *\
                    (friction_range[1] - friction_range[0]),
                link_indices=np.arange(0, self.plane.n_links))
    
    def randomize_mass(self):
        if(self.counter % int(self.domain_rand_cfg['rand_interval_s']/self.dt) == 0):
            added_mass_range = self.domain_rand_cfg['added_mass_range']
            added_mass = float(torch.rand(1).item() * (added_mass_range[1] - added_mass_range[0]) + added_mass_range[0])
            new_mass = max(self.base_mass + added_mass, 0.1)
            self.base_link.set_mass(new_mass)

    def get_observations(self):
        return self.obs_buf

    def get_privileged_observations(self):
        return None

    def get_noise_scale_vec(self):
        noise_vec = torch.zeros_like(self.obs_buf[0])
        noise_vec[:3] = self.noise_scales["lin_vel"] * self.noise_level * self.obs_scales["lin_vel"]
        noise_vec[3:6] = self.noise_scales["ang_vel"] * self.noise_level * self.obs_scales["ang_vel"]
        noise_vec[6:9] = self.noise_scales["gravity"] * self.noise_level
        noise_vec[9:12] = 0. # commands
        noise_vec[12:12+self.num_actions] = self.noise_scales["dof_pos"] * self.noise_level * self.obs_scales["dof_pos"]
        noise_vec[12+self.num_actions:12+2*self.num_actions] = self.noise_scales["dof_vel"] * self.noise_level * self.obs_scales["dof_pos"]
        noise_vec[12+2*self.num_actions:12+3*self.num_actions] = 0.0 # previous actions
        return noise_vec

    def reset_idx(self, envs_idx):
        if len(envs_idx) == 0:
            return

        # reset dofs
        self.dof_pos[envs_idx] = self.default_dof_pos
        self.dof_vel[envs_idx] = 0.0
        self.robot.set_dofs_position(
            position=self.dof_pos[envs_idx],
            dofs_idx_local=self.motor_dofs,
            zero_velocity=True,
            envs_idx=envs_idx,
        )

        # reset base
        self.base_pos[envs_idx] = self.base_init_pos
        self.base_quat[envs_idx] = self.base_init_quat.reshape(1, -1)
        self.base_euler = quat_to_xyz(
            transform_quat_by_quat(torch.ones_like(self.base_quat) * self.inv_base_init_quat, self.base_quat)
        )
        self.robot.set_pos(self.base_pos[envs_idx], zero_velocity=False, envs_idx=envs_idx)
        self.robot.set_quat(self.base_quat[envs_idx], zero_velocity=False, envs_idx=envs_idx)
        self.base_lin_vel[envs_idx] = 0
        self.base_ang_vel[envs_idx] = 0
        self.robot.zero_all_dofs_velocity(envs_idx)

        # reset buffers
        self.last_actions[envs_idx] = 0.0
        self.last_dof_vel[envs_idx] = 0.0
        self.episode_length_buf[envs_idx] = 0
        self.reset_buf[envs_idx] = True

        # fill extras
        self.extras["episode"] = {}
        for key in self.episode_sums.keys():
            self.extras["episode"]["rew_" + key] = (
                torch.mean(self.episode_sums[key][envs_idx]).item() / self.env_cfg["episode_length_s"]
            )
            self.episode_sums[key][envs_idx] = 0.0

        self._resample_commands(envs_idx)

    def reset(self):
        self.reset_buf[:] = True
        self.reset_idx(torch.arange(self.num_envs, device=self.device))
        return self.obs_buf, None

    # ------------ reward functions----------------
    def _reward_tracking_lin_vel(self):
        # Tracking of linear velocity commands (xy axes)
        lin_vel_error = torch.sum(torch.square(self.commands[:, :2] - self.base_lin_vel[:, :2]), dim=1)
        return torch.exp(-lin_vel_error / self.reward_cfg["tracking_sigma"])

    def _reward_tracking_ang_vel(self):
        # Tracking of angular velocity commands (yaw)
        ang_vel_error = torch.square(self.commands[:, 2] - self.base_ang_vel[:, 2])
        return torch.exp(-ang_vel_error / self.reward_cfg["tracking_sigma"])

    def _reward_lin_vel_z(self):
        # Penalize z axis base linear velocity
        return torch.square(self.base_lin_vel[:, 2])

    def _reward_action_rate(self):
        # Penalize changes in actions
        return torch.sum(torch.square(self.last_actions - self.actions), dim=1)

    def _reward_base_height(self):
        # Penalize base height away from target
        return torch.square(self.base_pos[:, 2] - self.reward_cfg[
            "base_height_target"])

    def _reward_alive(self):
        # Function borrowed from https://github.com/unitreerobotics/unitree_rl_gym
        # which is originally under BSD-3 License
        return 1.0

    def _reward_gait_contact(self):
        # Function borrowed from https://github.com/unitreerobotics/unitree_rl_gym
        # which is originally under BSD-3 License
        res = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        for i in range(self.feet_num):
            is_stance = self.leg_phase[:, i] < 0.55
            contact = self.contact_forces[:, self.feet_indices[i], 2] > 1
            res += ~(contact ^ is_stance)
        return res
    
    def _reward_gait_swing(self):
        res = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        for i in range(self.feet_num):
            is_swing = self.leg_phase[:, i] >= 0.55
            contact = self.contact_forces[:, self.feet_indices[i], 2] > 1
            res += ~(contact ^ is_swing)
        return res

    def _reward_contact_no_vel(self):
        # Function borrowed from https://github.com/unitreerobotics/unitree_rl_gym,
        # which is originally under BSD-3 License
        contact = torch.norm(self.contact_forces[:, self.feet_indices, :3],
                             dim=2) > 1.
        contact_feet_vel = self.feet_vel * contact.unsqueeze(-1)
        penalize = torch.square(contact_feet_vel[:, :, :3])
        return torch.sum(penalize, dim=(1, 2))

    def _reward_feet_swing_height(self):
        # Function borrowed from https://github.com/unitreerobotics/unitree_rl_gym,
        # which is originally under BSD-3 License
        contact = torch.norm(self.contact_forces[:, self.feet_indices, :3],
                             dim=2) > 1.0
        pos_error = torch.square(self.feet_pos[:, :, 2] - self.reward_cfg[
            "feet_height_target"]) * ~contact
        return torch.sum(pos_error, dim=(1))

    def _reward_orientation(self):
        # Function borrowed from https://github.com/unitreerobotics/unitree_rl_gym,
        # which is originally under BSD-3 License
        return torch.sum(torch.square(self.projected_gravity[:, :2]), dim=1)
    
    def _reward_ang_vel_xy(self):
        # Function borrowed from https://github.com/unitreerobotics/unitree_rl_gym,
        # which is originally under BSD-3 License
        return torch.sum(torch.square(self.base_ang_vel[:, :2]), dim=1)

    def _reward_dof_vel(self):
        # Function borrowed from https://github.com/unitreerobotics/unitree_rl_gym,
        # which is originally under BSD-3 License
        return torch.sum(torch.square(self.dof_vel), dim=1)

    def _reward_similar_to_default(self):
        # Penalize joint poses far away from default pose
        return torch.sum(torch.abs(self.dof_pos - self.default_dof_pos), dim=1)