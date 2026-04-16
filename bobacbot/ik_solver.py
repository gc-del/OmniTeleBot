# Author: Jimmy Wu
# Date: October 2024
#
# References:
# - https://github.com/bulletphysics/bullet3/blob/master/examples/ThirdPartyLibs/BussIK/Jacobian.cpp
# - https://github.com/kevinzakka/mjctrl/blob/main/diffik_nullspace.py
# - https://github.com/google-deepmind/dm_control/blob/main/dm_control/utils/inverse_kinematics.py

import mujoco
import numpy as np

class IKSolver:
    def __init__(self, ee_offset=0.0, damping_coeff=1e-6, max_angle_change=np.deg2rad(45)):
        # Load Mujoco model (eco65b)
        self.model = mujoco.MjModel.from_xml_path('models/eco65b/eco65b.xml')
        self.data = mujoco.MjData(self.model)
        self.model.body_gravcomp[:] = 1.0

        self.nq = self.model.nq
        self.nv = self.model.nv
        self.qpos0 = self.model.key('retract').qpos.copy()
        self.site_id = self.model.site('pinch_site').id
        self.model.site(self.site_id).pos[2] += ee_offset

        self.err = np.zeros(6)
        self.err_pos = self.err[:3]
        self.err_rot = self.err[3:]
        self.site_pos = np.zeros(3)
        self.site_mat = np.zeros((3, 3))
        self.site_quat = np.zeros(4)
        self.site_quat_inv = np.zeros(4)
        self.err_quat = np.zeros(4)
        self.jac = np.zeros((6, self.nv))
        self.jac_pos = self.jac[:3]
        self.jac_rot = self.jac[3:]
        self.damping = damping_coeff * np.eye(6)
        self.eye = np.eye(self.nv)
        self.max_angle_change = max_angle_change

    def forward_kinematics(self, qpos):
        """计算给定关节角度下末端位姿，返回位置与MuJoCo四元数[w, x, y, z]。"""
        self.data.qpos[:] = qpos
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_comPos(self.model, self.data)
        pos = self.data.site(self.site_id).xpos.copy()
        mat = self.data.site(self.site_id).xmat.copy().reshape(3, 3)
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, mat.flatten())
        return pos, quat

    def solve(self, pos, quat, curr_qpos, max_iters=40, pos_thresh=1e-4, rot_thresh=1e-4):
        """逆运动学求解函数。

        参数:
            pos: 目标末端位置，形状 (3,)
            quat: 目标四元数，期望格式 [x, y, z, w]
            curr_qpos: 初始关节角度，形状与模型qpos匹配
        返回:
            收敛后的关节角度，形状与curr_qpos一致
        """
        target_quat = np.asarray(quat, dtype=float)
        if target_quat.shape != (4,):
            raise ValueError('quat must be a 4-element array')
        target_quat = target_quat[[3, 0, 1, 2]]  # convert [x, y, z, w] -> [w, x, y, z]

        self.data.qpos[:] = np.asarray(curr_qpos, dtype=float).copy()

        for _ in range(max_iters):
            mujoco.mj_kinematics(self.model, self.data)
            mujoco.mj_comPos(self.model, self.data)

            self.site_pos[:] = self.data.site(self.site_id).xpos
            self.site_mat[:] = self.data.site(self.site_id).xmat.reshape(3, 3)

            self.err_pos[:] = pos - self.site_pos
            mujoco.mju_mat2Quat(self.site_quat, self.site_mat.flatten())
            mujoco.mju_negQuat(self.site_quat_inv, self.site_quat)
            mujoco.mju_mulQuat(self.err_quat, target_quat, self.site_quat_inv)
            mujoco.mju_quat2Vel(self.err_rot, self.err_quat, 1.0)

            if np.linalg.norm(self.err_pos) < pos_thresh and np.linalg.norm(self.err_rot) < rot_thresh:
                break

            mujoco.mj_jacSite(self.model, self.data, self.jac_pos, self.jac_rot, self.site_id)
            lhs = self.jac @ self.jac.T + self.damping
            update = self.jac.T @ np.linalg.solve(lhs, self.err)

            qpos0_err = np.mod(self.qpos0 - self.data.qpos + np.pi, 2 * np.pi) - np.pi
            null_proj = self.eye - self.jac.T @ np.linalg.solve(lhs, self.jac)
            update += 0.05 * (null_proj @ qpos0_err)

            update_max = np.abs(update).max()
            if update_max > self.max_angle_change:
                update *= self.max_angle_change / update_max

            self.data.qpos[:] += update

        return np.mod(self.data.qpos + np.pi, 2 * np.pi) - np.pi
    
if __name__ == '__main__':
    ik_solver = IKSolver()
    
    # 调整测试参数以适应eco65b机械臂
    # 需要重新确定合适的末端位置和姿态
    home_pos, home_quat = np.array([0.3, 0.0, 0.2]), np.array([0.0, 1.0, 0.0, 0.0])  # 调整为eco65b的合理工作空间
    
    # 使用eco65b的retract关节角度 (6个关节)
    # 从eco65b.xml中的retract keyframe: "0.0 0.52 -1.2 0.34 1.57 -1.57"
    retract_qpos = np.array([0.0, 0.52, -1.2, 0.34, 1.57, -1.57])
    
    print(f"eco65b关节数量: {len(retract_qpos)}")
    print(f"收缩姿态 (弧度): {retract_qpos}")
    print(f"收缩姿态 (度数): {np.rad2deg(retract_qpos).round()}")

    import time
    start_time = time.time()
    for _ in range(1000):
        qpos = ik_solver.solve(home_pos, home_quat, retract_qpos)
    elapsed_time = time.time() - start_time
    print(f'平均每次调用时间: {elapsed_time:.3f} ms')

    # 输出逆运动学求解结果
    result_qpos = ik_solver.solve(home_pos, home_quat, retract_qpos)
    print(f"逆运动学求解结果 (度数): {np.rad2deg(result_qpos).round()}")
    print(f"逆运动学求解结果 (弧度): {result_qpos.round(3)}")
