#!/usr/bin/env python3

import time
import numpy as np


JOINT_LIMIT_DEGREES = {
    0: [-180.0, 180.0],
    1: [-128.9, 128.9],
    2: [-180.0, 180.0],
    3: [-147.8, 147.8],
    4: [-180.0, 180.0],
    5: [-120.3, 120.3],
    6: [-180.0, 180.0],
}

JOINT_LIMIT = {
    0: [-3.141592653589793, 3.141592653589793],
    1: [-2.2497294058206907, 2.2497294058206907],
    2: [-3.141592653589793, 3.141592653589793],
    3: [-2.5795966344476193, 2.5795966344476193],
    4: [-3.141592653589793, 3.141592653589793],
    5: [-2.0996310901491784, 2.0996310901491784],
    6: [-3.141592653589793, 3.141592653589793],
    7: [0, 1],
}

DH_PARAMETERS = {
    0: [np.pi,   0,                  0,     0],
    1: [np.pi/2, 0, -(0.1564 + 0.1284),     0],
    2: [np.pi/2, 0, -(0.0054 + 0.0064), np.pi],
    3: [np.pi/2, 0, -(0.2104 + 0.2104), np.pi],
    4: [np.pi/2, 0, -(0.0064 + 0.0064), np.pi],
    5: [np.pi/2, 0, -(0.2084 + 0.1059), np.pi],
    6: [np.pi/2, 0,                  0, np.pi],
    7: [np.pi,   0, -(0.1059 + 0.0615), np.pi],
    8: [0,       0,              0.140,    0],
}

JOINT_NAME_TO_ID = {
    "joint_1": 0,
    "joint_2": 1,
    "joint_3": 2,
    "joint_4": 3,
    "joint_5": 4,
    "joint_6": 5,
    "joint_7": 6,
}


class KinovaGen3(object):
    """
    robosuite / MuJoCo shim with the same public surface that the PNG code expects.

    This is intentionally a thin compatibility layer, not a faithful Kortex API clone.
    """

    def __init__(self, robot_name: str = "sim_gen3"):
        self.robot_name = robot_name
        self.dof = 7
        self.joint_names = [f"joint_{i}" for i in range(1, 8)]
        self.is_gripper_present = True
        self.HOME_ACTION_IDENTIFIER = 2
        self.JOINT_NAME_TO_ID = JOINT_NAME_TO_ID
        self.JOINT_LIMIT = JOINT_LIMIT

        self.prev_gripper_cmd = None
        self.prev_cv_cmd = None

        self.movement_blocked = False
        self.going_home = False
        self.home_button_pressed = False
        self.just_went_home = False
        self.last_action_notif_type = None

        # robosuite / mujoco handles
        self.env = None
        self.sim = None
        self.robot = None

        # joint / pose state
        self.position = np.zeros(8, dtype=np.float64)  # 7 arm + 1 gripper scalar
        self.vel = np.zeros(8, dtype=np.float64)
        self.cartesian_pose = np.zeros(6, dtype=np.float64)

        # pending commands
        self.pending_joint_velocity_cmd = np.zeros(7, dtype=np.float64)
        self.pending_cartesian_velocity_cmd = np.zeros(6, dtype=np.float64)
        self.pending_gripper_cmd = 0.0

        # gripper state used by shim
        self._gripper_state = 0.0

        # cached indices
        self._arm_qpos_idxs = None
        self._arm_qvel_idxs = None
        self._gripper_qpos_idxs = []

    def attach_env(self, env):
        self.env = env
        self.sim = env.sim
        self.robot = env.robots[0]

        self._arm_qpos_idxs = self._infer_arm_qpos_indices()
        self._arm_qvel_idxs = self._infer_arm_qvel_indices()
        self._gripper_qpos_idxs = self._infer_gripper_qpos_indices()

        self.refresh_state()

    def _infer_arm_qpos_indices(self):
        if hasattr(self.robot, "_ref_joint_pos_indexes"):
            return np.array(self.robot._ref_joint_pos_indexes, dtype=int)

        idxs = []
        joint_names = []

        if hasattr(self.robot, "robot_model") and hasattr(self.robot.robot_model, "joints"):
            joint_names = list(self.robot.robot_model.joints)

        for name in joint_names:
            try:
                jid = self.sim.model.joint_name2id(name)
                idxs.append(int(self.sim.model.jnt_qposadr[jid]))
            except Exception:
                pass

        if len(idxs) != 7:
            raise RuntimeError("Could not infer arm qpos indices for Kinova3")

        return np.array(idxs, dtype=int)

    def _infer_arm_qvel_indices(self):
        if hasattr(self.robot, "_ref_joint_vel_indexes"):
            return np.array(self.robot._ref_joint_vel_indexes, dtype=int)

        idxs = []
        joint_names = []

        if hasattr(self.robot, "robot_model") and hasattr(self.robot.robot_model, "joints"):
            joint_names = list(self.robot.robot_model.joints)

        for name in joint_names:
            try:
                jid = self.sim.model.joint_name2id(name)
                idxs.append(int(self.sim.model.jnt_dofadr[jid]))
            except Exception:
                pass

        if len(idxs) != 7:
            raise RuntimeError("Could not infer arm qvel indices for Kinova3")

        return np.array(idxs, dtype=int)

    def _infer_gripper_qpos_indices(self):
        idxs = []

        try:
            if hasattr(self.robot, "gripper") and hasattr(self.robot.gripper, "joints"):
                for name in self.robot.gripper.joints:
                    jid = self.sim.model.joint_name2id(name)
                    idxs.append(int(self.sim.model.jnt_qposadr[jid]))
        except Exception:
            pass

        return idxs

    def refresh_state(self):
        if self.sim is None:
            return

        arm_q = np.array(self.sim.data.qpos[self._arm_qpos_idxs], dtype=np.float64)
        arm_qd = np.array(self.sim.data.qvel[self._arm_qvel_idxs], dtype=np.float64)

        self.position[:7] = arm_q
        self.vel[:7] = arm_qd

        self.position[7] = self._gripper_state
        self.vel[7] = self.pending_gripper_cmd

        T = self.dh_mats((0, 8))
        xyz = T[:3, 3]

        # crude pose export, mainly placeholder to mirror your real class
        self.cartesian_pose[:3] = xyz
        self.cartesian_pose[3:] = 0.0

    def block_movement(self):
        self.movement_blocked = True

    def unblock_movement(self):
        self.movement_blocked = False

    def open_gripper(self):
        while self.position[7] > 0.07:
            self.send_gripper_command(-0.3)
            self.apply_pending_command(1.0 / 30.0)
        self.send_gripper_command(0.0)

    def go_home(self):
        if self.going_home:
            return False

        self.going_home = True
        self.send_joint_speeds_command(np.zeros(7))
        self.send_joint_angles([0.1, 65, -179.9, -120, 0, 100, 90])
        self.going_home = False
        self.home_button_pressed = False
        self.just_went_home = True
        print("Done going home.")
        return True

    def send_joint_angles(
        self,
        angles: list,
        angular_duration: float = 0.0,
        MAX_ANGULAR_DURATION: float = 30.0,
    ):
        if isinstance(angles, list):
            angles = np.array(angles, dtype=np.float64)

        if len(angles) != self.dof:
            print("Invalid angles.")
            return False

        if self.sim is None:
            return False

        q = np.deg2rad(angles)
        for i in range(7):
            q[i] = np.clip(q[i], JOINT_LIMIT[i][0], JOINT_LIMIT[i][1])

        self.sim.data.qpos[self._arm_qpos_idxs] = q
        self.sim.data.qvel[self._arm_qvel_idxs] = 0.0
        self.sim.forward()
        self.refresh_state()
        return True

    def send_joint_speeds_command(self, th_ds):
        if self.movement_blocked:
            th_ds = np.zeros(7)

        if self.going_home:
            return False

        th_ds = np.array(th_ds, dtype=np.float64)
        self.pending_joint_velocity_cmd = th_ds.copy()

        if np.linalg.norm(th_ds) > 1e-4:
            self.just_went_home = False

        return True

    def send_cartesian_velocity(self, axes, ref_frame=1):
        axes = np.array(axes, dtype=np.float64)

        if np.array_equal(axes, self.prev_cv_cmd):
            return False

        self.prev_cv_cmd = axes.copy()
        self.pending_cartesian_velocity_cmd = axes.copy()
        self.just_went_home = False
        return True

    def send_gripper_command(self, value: float):
        if not self.is_gripper_present:
            print("No gripper is present on the arm.")
            return False

        if value == self.prev_gripper_cmd:
            return True

        self.prev_gripper_cmd = value
        self.pending_gripper_cmd = float(value)
        return True

    def apply_pending_command(self, dt):
        """
        Kinematic application of the commanded joint velocities directly into MuJoCo qpos/qvel.
        This bypasses robosuite's internal controller stack on purpose.
        """
        if self.sim is None:
            return

        self.refresh_state()

        q = self.position[:7].copy()
        qd = self.pending_joint_velocity_cmd.copy()

        q_next = q + qd * dt

        for i in range(7):
            q_next[i] = np.clip(q_next[i], JOINT_LIMIT[i][0], JOINT_LIMIT[i][1])

        self.sim.data.qpos[self._arm_qpos_idxs] = q_next
        self.sim.data.qvel[self._arm_qvel_idxs] = qd

        # simple integrated scalar gripper state
        self._gripper_state = np.clip(self._gripper_state + self.pending_gripper_cmd * dt, 0.0, 1.0)

        if len(self._gripper_qpos_idxs) > 0:
            for idx in self._gripper_qpos_idxs:
                self.sim.data.qpos[idx] = self._gripper_state

        self.sim.forward()
        self.refresh_state()

    def dh_mats(self, n=(0, 7)):
        current_angles = np.concatenate(([0], self.position[:7], [0]))
        M = np.identity(4)
        for i in range(n[0], n[1] + 1):
            theta = DH_PARAMETERS[i][3] + current_angles[i]
            M_new = np.array([
                [np.cos(theta), -np.cos(DH_PARAMETERS[i][0]) * np.sin(theta),  np.sin(DH_PARAMETERS[i][0]) * np.sin(theta), DH_PARAMETERS[i][1] * np.cos(theta)],
                [np.sin(theta),  np.cos(DH_PARAMETERS[i][0]) * np.cos(theta), -np.sin(DH_PARAMETERS[i][0]) * np.cos(theta), DH_PARAMETERS[i][1] * np.sin(theta)],
                [0,              np.sin(DH_PARAMETERS[i][0]),                  np.cos(DH_PARAMETERS[i][0]),                  DH_PARAMETERS[i][2]],
                [0,              0,                                             0,                                             1],
            ])
            M = M @ M_new
        return M

    def dh_mat_inv(self, n):
        current_angles = np.concatenate(([0], self.position[:7], [0]))
        theta = DH_PARAMETERS[n][3] + current_angles[n]
        R_T = np.array([
            [np.cos(theta),                                np.sin(theta),                               0],
            [-np.cos(DH_PARAMETERS[n][0]) * np.sin(theta),  np.cos(DH_PARAMETERS[n][0]) * np.cos(theta), np.sin(DH_PARAMETERS[n][0])],
            [np.sin(DH_PARAMETERS[n][0]) * np.sin(theta),  -np.sin(DH_PARAMETERS[n][0]) * np.cos(theta), np.cos(DH_PARAMETERS[n][0])],
        ])

        T = -R_T @ np.array([
            [DH_PARAMETERS[n][1] * np.cos(theta)],
            [DH_PARAMETERS[n][1] * np.sin(theta)],
            [DH_PARAMETERS[n][2]],
        ])

        M_new = np.concatenate((R_T, T), axis=1)
        M_new = np.concatenate((M_new, np.array([[0, 0, 0, 1]])), axis=0)
        return M_new

    def dh_mat(self, n):
        current_angles = np.concatenate(([0], self.position[:7], [0]))
        theta = DH_PARAMETERS[n][3] + current_angles[n]
        M = np.array([
            [np.cos(theta), -np.cos(DH_PARAMETERS[n][0]) * np.sin(theta),  np.sin(DH_PARAMETERS[n][0]) * np.sin(theta), DH_PARAMETERS[n][1] * np.cos(theta)],
            [np.sin(theta),  np.cos(DH_PARAMETERS[n][0]) * np.cos(theta), -np.sin(DH_PARAMETERS[n][0]) * np.cos(theta), DH_PARAMETERS[n][1] * np.sin(theta)],
            [0,              np.sin(DH_PARAMETERS[n][0]),                  np.cos(DH_PARAMETERS[n][0]),                  DH_PARAMETERS[n][2]],
            [0,              0,                                             0,                                             1],
        ])
        return M

    def v_mat_trans(self, jv, n=7):
        jv = np.concatenate((np.array(jv, dtype=np.float64), [0], [0]))
        W = np.zeros((4, 4))
        for i in range(n + 1):
            M = self.dh_mat(i)
            M_inv = self.dh_mat_inv(i)
            W = M_inv @ W @ M
            W[0][1] -= jv[i]
            W[1][0] += jv[i]
        return W

    def __str__(self):
        string = "Kinova Gen3 (Sim)\n"
        string += "  robot_name: {}\n  dof: {}".format(self.robot_name, self.dof)
        return string

def gen_iris(base, env, state, DT):
    class IrisRecord(base):
        def __init__(self):
            super(IrisRecord, self).__init__(None)
            self.attach_env(env)
            self.mode = 0
            self.axes_vector = [0.0] * 6
            self.gripper_cmd = 0.0
            self.gripper_target_state = 0
            
            self.home_array = np.array([0.1, 65, -179.9, -120, 0, 100, 90], dtype=float)
            self.send_joint_angles(self.home_array)

            self.DT = DT
            
            print(f"Current mode = {self.mode}")

        def mode_switch(self):
            self.mode = (self.mode + 1) % 2
            state.set_mode(self.mode) 
            print(f"Current mode = {self.mode}")

        def controller_update(self):
            snap = state.snapshot()
            self.axes_vector = snap["axes"]
            self.gripper_cmd = snap["gripper_cmd"]

            if state.consume_mode_toggle():
                self.mode_switch()

            if state.consume_home():
                self.send_joint_angles(self.home_array)

        def step(self):
            self.refresh_state()
            self.controller_update()

            # Integrate joystick input
            self.gripper_target_state += self.gripper_cmd * self.DT

            super().step(self.axes_vector, self.mode)

    return IrisRecord()