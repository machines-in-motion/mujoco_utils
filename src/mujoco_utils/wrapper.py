"""
License: BSD 3-Clause License
Copyright (C) 2022, New York University
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

import warnings
import numpy as np
from dm_control import mujoco


class MujocoWrapper:
    def __init__(self, physics, pinocchio_robot, joint_names, mujoco_endeff_geom_id):
        """
        Args:
            mujoco_endeff_geom_id: The mujoco id of the endeffector to check contact with.
        """
        self.physics = physics

        self.nq = pinocchio_robot.nq
        self.nv = pinocchio_robot.nv
        self.nj = len(joint_names)
        self.nf = len(mujoco_endeff_geom_id)
        self.pinocchio_robot = pinocchio_robot

        self.use_fixed_base = self.nq == self.nv

        if self.use_fixed_base:
            self.nb_dof = self.nv
        else:
            self.nb_dof = self.nv - 6

        self.joint_names = joint_names
        self.mujoco_endeff_geom_id = np.array(mujoco_endeff_geom_id)

    def step_simulation(self):
        self.physics.step()

    def get_state(self):
        q = np.zeros(self.nq)
        v = np.zeros(self.nv)
        qpos = self.physics.named.data.qpos
        qvel = self.physics.named.data.qvel
        joint_names = self.joint_names

        # In case of free-floating robot, need to get the 7-dim
        # base joint first.
        if not self.use_fixed_base:
            q[:7] = qpos[joint_names[0]]
            v[:6] = qvel[joint_names[0]]

            # Convert the Mujoco quaternion into pinocchio convention.
            quat = q[3:7].copy()
            q[3:6] = quat[1:]
            q[6] = quat[0]

            q_offset = 6
            v_offset = 5
        else:
            q[0] = qpos[joint_names[0]]
            v[0] = qvel[joint_names[0]]

            q_offset = 0
            v_offset = 0

        for i in range(1, self.nj):
            q[q_offset + i] = qpos[joint_names[i]]
            v[v_offset + i] = qvel[joint_names[i]]

        return q, v

    def send_joint_command(self, tau):
        ctrl = physics.named.data.ctrl
        joint_names = self.joint_names

        if self.use_fixed_base:
            joint_start = 0
        else:
            joint_start = 1

        for i, ji in enumerate(range(joint_start, self.nj)):
            ctrl[joint_names[ji]] = tau[i]

    def reset_state(self, q, v):
        qpos = self.physics.named.data.qpos
        qvel = self.physics.named.data.qvel
        joint_names = self.joint_names

        with self.physics.reset_context():
            # Treat the first joint differently for free-floating joints.
            q_mjco = q.copy()
            v_mjco = v
            if not self.use_fixed_base:
                q_offset = 6
                v_offset = 5
                q_mjco[3] = q[6]
                q_mjco[4] = q[3]
                q_mjco[5] = q[4]
                q_mjco[6] = q[5]

                qpos[joint_names[0]] = q_mjco[:7]
                qvel[joint_names[0]] = v_mjco[:6]
            else:
                q_offset = 0
                v_offset = 0
                qpos[joint_names[0]] = q_mjco[0]
                qvel[joint_names[0]] = v_mjco[0]

            for ji in range(1, self.nj):
                qpos[joint_names[ji]] = q_mjco[q_offset + ji]
                qvel[joint_names[ji]] = v_mjco[v_offset + ji]

    def get_end_effector_forces(self):
        contact = self.physics.data.contact
        cnt_geom1 = contact.geom1
        cnt_geom2 = contact.geom2

        cnt_status = np.zeros(self.nf)
        cnt_forces = np.zeros((self.nf, 6))

        eff_id = None

        for i in range(len(cnt_geom2)):
            if cnt_geom2[i] in self.mujoco_endeff_geom_id:
                eff_id = np.where(self.mujoco_endeff_geom_id == cnt_geom2[i])[0][0]
            elif cnt_geom1[i] in self.mujoco_endeff_geom_id:
                eff_id = np.where(self.mujoco_endeff_geom_id == cnt_geom1[i])[0][0]
            else:
                continue

            cnt_status[eff_id] = 1
            force, tau = self.physics.data.contact_force(i)

            # Rotate the force and torque into the world frame.
            rot = contact.frame[i].reshape(3, 3)
            force = force @ rot
            tau = tau @ rot

            # Setting small torques to zero.
            tau[np.where(np.abs(tau) < 1e-4)[0]] = 0.

            cnt_forces[i] = np.hstack([force, tau])

        return cnt_status, cnt_forces

    def end_effector_forces(self):
        warnings.warn("The method end_effector_forces is deprecated. Please use get_end_effector_forces() instead.", DeprecationWarning)
        return self.get_end_effector_forces()
