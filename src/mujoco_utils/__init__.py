"""
License: BSD 3-Clause License
Copyright (C) 2022, New York University
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

import numpy as np
from dm_control import mujoco


class MujocoWrapper:
    def __init__(self, physics, pinocchio_robot, joint_names, mujoco_endeff_geom_id, useFixedBase):
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
        self.useFixedBase = useFixedBase
        self.nb_dof = self.nv - 6

        self.joint_names = joint_names
        self.mujoco_endeff_geom_id = np.array(mujoco_endeff_geom_id)

    def step_simulation(self):
        self.physics.step()

    def get_state(self):
        # TODO: Should loop over the joint_names.
        # TODO: Need to make it work for fixed-base robots.
        q = np.array(self.physics.named.data.qpos)
        dq = np.array(self.physics.named.data.qvel)

        # TODO: Need to do the quaternion check general.
        # Need to swap quaternion into pinocchio convention.
        quat = q[3:7].copy()
        q[3:6] = quat[1:]
        q[6] = quat[0]

        return q, dq

    def send_joint_command(self, tau):
        # TODO: Do proper mapping.
        self.physics.data.ctrl = tau

    def reset_state(self, q, dq):
        q_mjco = q.copy()
        q_mjco[3] = q[6]
        q_mjco[4] = q[3]
        q_mjco[5] = q[4]
        q_mjco[6] = q[5]

        with self.physics.reset_context():
            self.physics.data.qpos = q_mjco
            self.physics.data.qvel = dq

    def get_end_effector_forces(self):
        return self.end_effector_forces()

    def end_effector_forces(self):
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
