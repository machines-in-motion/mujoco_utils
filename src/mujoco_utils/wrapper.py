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
        self.compute_numerical_quantities()

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
        ctrl = self.physics.named.data.ctrl
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

    def compute_numerical_quantities(self, dt):
        pass


class MujocoIMUWrapper(MujocoWrapper):
    def __init__(self, physics, pinocchio_robot, joint_names, mujoco_endeff_geom_id, r_base_to_imu):
        super().__init__(physics, pinocchio_robot, joint_names, mujoco_endeff_geom_id)

        # IMU pose offset in base frame
        self.rot_base_to_imu = np.identity(3)
        self.r_base_to_imu = r_base_to_imu

        self.base_linvel_prev = None
        self.base_angvel_prev = None
        self.base_linacc = np.zeros(3)
        self.base_angacc = np.zeros(3)

        self.rng = default_rng()

        self.base_imu_accel_bias = np.zeros(3)
        self.base_imu_gyro_bias = np.zeros(3)
        self.base_imu_accel_thermal = np.zeros(3)
        self.base_imu_gyro_thermal = np.zeros(3)
        self.base_imu_accel_thermal_noise = 0.0001962 # m/(sec^2*sqrt(Hz))
        self.base_imu_gyro_thermal_noise = 0.0000873  # rad/(sec*sqrt(Hz))
        self.base_imu_accel_bias_noise = 0.0001       # m/(sec^3*sqrt(Hz))
        self.base_imu_gyro_bias_noise = 0.000309      # rad/(sec^2*sqrt(Hz))

    def get_base_velocity_world(self):
        """Returns the velocity of the base in the world frame.
        Returns:
            np.array((6,1)) with the translation and angular velocity
        """
        vel, orn = pybullet.getBaseVelocity(self.robot_id)
        return np.array(vel + orn).reshape(6, 1)

    def get_base_acceleration_world(self):
        """Returns the numerically-computed acceleration of the base in the world frame.
        Returns:
            np.array((6,1)) vector of linear and angular acceleration
        """
        return np.concatenate((self.base_linacc, self.base_angacc))

    def get_base_imu_angvel(self):
        """ Returns simulated base IMU gyroscope angular velocity.
        Returns:
            np.array((3,1)) IMU gyroscope angular velocity (base frame)
        """
        base_inertia_pos, base_inertia_quat = pybullet.getBasePositionAndOrientation(self.robot_id)
        rot_base_to_world = np.array(pybullet.getMatrixFromQuaternion(base_inertia_quat)).reshape((3, 3))
        base_linvel, base_angvel = pybullet.getBaseVelocity(self.robot_id)

        return self.rot_base_to_imu.dot(rot_base_to_world.T.dot(np.array(base_angvel))) + self.base_imu_gyro_bias + self.base_imu_gyro_thermal

    def get_base_imu_linacc(self):
        """ Returns simulated base IMU accelerometer acceleration.
        Returns:
            np.array((3,1)) IMU accelerometer acceleration (base frame, gravity offset)
        """
        base_inertia_pos, base_inertia_quat = pybullet.getBasePositionAndOrientation(self.robot_id)
        rot_base_to_world = np.array(pybullet.getMatrixFromQuaternion(base_inertia_quat)).reshape((3, 3))
        base_linvel, base_angvel = pybullet.getBaseVelocity(self.robot_id)

        # Transform the base acceleration to the IMU position, in world frame
        imu_linacc = self.base_linacc + np.cross(self.base_angacc, rot_base_to_world @ self.r_base_to_imu) +\
                     np.cross(base_angvel, np.cross(base_angvel, rot_base_to_world @ self.r_base_to_imu))

        return self.rot_base_to_imu.dot(rot_base_to_world.T.dot(imu_linacc + np.array([0.0, 0.0, 9.81]))) + self.base_imu_accel_bias + self.base_imu_accel_thermal

    def get_imu_frame_position_velocity(self):
        """Returns the position and velocity of IMU frame. Note that the velocity is expressed in the IMU frame.
        Returns:
            np.array((3,1)): IMU frame position expressed in world.
            np.array((3,1)): IMU frame velocity expressed in IMU frame.
        """
        base_pose, base_quat = pybullet.getBasePositionAndOrientation(self.robot_id)
        base_linvel, base_angvel = pybullet.getBaseVelocity(self.robot_id)

        rot_base_to_world = np.array(pybullet.getMatrixFromQuaternion(base_quat)).reshape((3, 3))
        rot_imu_to_world = rot_base_to_world.dot(self.rot_base_to_imu.T)

        imu_position = base_pose + rot_base_to_world.dot(self.r_base_to_imu)
        imu_velocity = rot_imu_to_world.T.dot(base_linvel + np.cross(base_angvel, rot_base_to_world.dot(self.r_base_to_imu)))
        return imu_position, imu_velocity

    def compute_numerical_quantities(self, dt):
        """Compute numerical robot quantities from simulation results.
        Args:
            dt (float): Length of the time step.
        """

        # Compute base acceleration numerically
        linvel, angvel = pybullet.getBaseVelocity(self.robot_id)
        if self.base_linvel_prev is not None and self.base_angvel_prev is not None:
            self.base_linacc = (1.0 / dt) * (np.array(linvel) - self.base_linvel_prev)
            self.base_angacc = (1.0 / dt) * (np.array(angvel) - self.base_angvel_prev)

        self.base_linvel_prev = np.array(linvel)
        self.base_angvel_prev = np.array(angvel)

        # Integrate IMU accelerometer/gyroscope bias terms forward.
        self.base_imu_accel_bias += dt * (self.base_imu_accel_bias_noise / np.sqrt(dt)) * self.rng.standard_normal(3)
        self.base_imu_gyro_bias += dt * (self.base_imu_gyro_bias_noise / np.sqrt(dt)) * self.rng.standard_normal(3)

        # Add simulated IMU sensor thermal noise.
        self.base_imu_accel_thermal = (self.base_imu_accel_thermal_noise / np.sqrt(dt)) * self.rng.standard_normal(3)
        self.base_imu_gyro_thermal = (self.base_imu_gyro_thermal_noise / np.sqrt(dt)) * self.rng.standard_normal(3)
