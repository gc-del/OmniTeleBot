#!/usr/bin/env python3
# joycon_real_policy.py (INCREMENTAL STABLE VERSION)

import time
import math
import json
import threading
import numpy as np
import websocket
from scipy.spatial.transform import Rotation as R

from policies import Policy
from ..joyconrobotics import JoyconRobotics


# ======================
# utils
# ======================
def _normalize_stick(v, deadzone=0.08):
    v = (v - 2048.0) / 2048.0
    if abs(v) < deadzone:
        return 0.0
    return float(np.clip(v, -1.0, 1.0))


# ======================
# WS CLIENT
# ======================
class WSClient:
    def __init__(self, url):
        self.url = url
        self.ws = None
        self.connected = False
        self.latest_state = None

        self._connect()
        threading.Thread(target=self._loop, daemon=True).start()

    def _connect(self):
        try:
            self.ws = websocket.WebSocket()
            self.ws.connect(self.url)
            self.connected = True
            print(f"[WS] connected: {self.url}")
        except Exception as e:
            print(f"[WS] connect fail: {e}")
            self.connected = False

    def _loop(self):
        while True:
            if not self.connected:
                time.sleep(1)
                self._connect()
                continue

            try:
                msg = self.ws.recv()
                data = json.loads(msg)
                self.latest_state = data
            except Exception:
                self.connected = False

    def send(self, msg):
        if not self.connected:
            return False
        try:
            self.ws.send(json.dumps(msg))
            return True
        except Exception:
            self.connected = False
            return False


# ======================
# POLICY
# ======================
class JoyconRealPolicy(Policy):
    control_period = 0.05

    def __init__(self, websocket_url, custom_initial_arm_pos, custom_initial_arm_quat):

        print("\n==============================")
        print(" JOYCON INCREMENTAL CONTROL")
        print("==============================\n")

        # WS
        self.ws = WSClient(websocket_url)

        # Joycon
        self.left = JoyconRobotics(device='left', gripper_open=1.0,gripper_close=0.0,limit_dof=True,all_button_return=True)
        self.right = JoyconRobotics(device='right', gripper_open=1.0,gripper_close=0.0,limit_dof=True,all_button_return=True)

        # base
        self.base_cmd = [0.0, 0.0, 0.0]

        # init pose
        self.init_pos = np.array(custom_initial_arm_pos, dtype=np.float64)
        q = np.array(custom_initial_arm_quat, dtype=np.float64)
        self.init_rot = R.from_quat(q / np.linalg.norm(q))

        # arm state
        self.arm_pos = self.init_pos.copy()
        self.arm_quat = self.init_rot.as_quat()

        # FSM
        self.arm_mode = "idle"

        # reference (核心)
        self.ctrl_ref_pose = None
        self.ctrl_ref_rot_inv = None

        self.ref_arm_pos = None
        self.ref_arm_rot = None

        # buttons
        self.prev_r = False
        self.prev_y = False

        # gripper control (自动从 get_control 读取)
        self.gripper_state = 0

        # episode
        self.enabled = False
        self.teleop_state = None
        self.episode_end_flag = False

        print("[READY] Y: episode control")
        print("[READY] R: incremental arm control\n")

    # ======================
    # episode control
    # ======================
    def _episode(self):
        y = self.right.joycon.get_button_y()

        if y and not self.prev_y:
            if self.teleop_state is None:
                self.enabled = True
                self.teleop_state = "episode_started"

            elif self.teleop_state == "episode_started":
                self.enabled = False
                self.teleop_state = "episode_ended"

            elif self.teleop_state == "episode_ended":
                self.teleop_state = "reset_env"

        self.prev_y = y

    # ======================
    # base control
    # ======================
    def _base(self):
        x = _normalize_stick(self.left.joycon.get_stick_left_vertical())
        y = -_normalize_stick(self.left.joycon.get_stick_left_horizontal())

        yaw = 0.0
        if self.left.joycon.get_button_l():
            yaw = 0.5
        if self.left.joycon.get_button_zl():
            yaw = -0.5
        # print(f"[BASE] stick: x={x:+.3f}, y={y:+.3f}, yaw={yaw:+.3f}")
        self.base_cmd = [x * 0.3, y * 0.3, yaw]

    # ======================
    # ARM（真正增量控制）
    # ======================
    def _arm(self):

        pose, gripper_state, _ = self.right.get_control()
        pose = np.asarray(pose, dtype=np.float64)
        
        # 低通滤波平滑手柄数据（对抗陀螺仪漂移）
        if not hasattr(self, '_pose_filter_alpha'):
            self._pose_filter_alpha = 0.2  # 平滑因子
            self._filtered_pose = pose.copy()
        
        # 指数移动平均
        self._filtered_pose = (1.0 - self._pose_filter_alpha) * self._filtered_pose + self._pose_filter_alpha * pose
        pose = self._filtered_pose.copy()
        
        # 保存夹爪状态（0.0~1.0，映射到0~100）
        self.gripper_state = int(gripper_state * 100)

        # ======================
        # 🔍 DEBUG: 手柄姿态 (RPY + Δ)
        # ======================

        # 当前控制器姿态（弧度）
        roll, pitch, yaw = pose[3], pose[4], pose[5]

        # 初始化缓存
        if not hasattr(self, "_dbg_prev_rpy"):
            self._dbg_prev_rpy = np.array([roll, pitch, yaw])

        prev_rpy = self._dbg_prev_rpy
        curr_rpy = np.array([roll, pitch, yaw])

        # 增量
        delta_rpy = curr_rpy - prev_rpy

        # 更新缓存
        self._dbg_prev_rpy = curr_rpy.copy()

        # 打印（控制频率10Hz，不会炸屏）
        print(
            "[JOYCON] "
            f"R={roll:+.3f} P={pitch:+.3f} Y={yaw:+.3f} | "
            f"X={pose[0]:+.3f} Y={pose[1]:+.3f} Z={pose[2]:+.3f} | "
            # f"dR={delta_rpy[0]:+.3f} dP={delta_rpy[1]:+.3f} dY={delta_rpy[2]:+.3f}"
        )

        ctrl_rot = R.from_euler('xyz', [-pose[3], -pose[4], pose[5]])

        state = self.ws.latest_state
        if not state or "arm_state" not in state:
            print("[WARN] No arm_state yet...")
            return False

        p = state["arm_state"]["pose"]["position"]
        q = state["arm_state"]["pose"]["orientation"]

        real_pos = np.array([p["x"], p["y"], p["z"]])
        real_rot = R.from_quat([q["x"], q["y"], q["z"], q["w"]])

        r = self.right.joycon.get_button_r()
        home = self.right.joycon.get_button_home()

        edge_r = r and not self.prev_r
        edge_home = home and not getattr(self, "prev_home", False)

        self.prev_r = r
        self.prev_home = home

        # ======================
        # 🟥 1. Home → 强制回初始位姿
        # ======================
        if edge_home:
            print("[ARM] 🏠 HOME → go to initial pose")
            self.arm_mode = "homing"

        # ======================
        # 🟥 2. Homing（最高优先级）
        # ======================
        if self.arm_mode == "homing":

            self.arm_pos = self.init_pos
            self.arm_quat = self.init_rot.as_quat()

            pos_err = np.linalg.norm(real_pos - self.init_pos)
            rot_err = (real_rot.inv() * self.init_rot).magnitude()

            print(f"[HOME] pos_err={pos_err:.3f}, rot_err={rot_err:.3f}")

            if pos_err < 0.015 and rot_err < 0.08:
                print("[HOME] ✅ reached initial pose")
                
                # 重新校准手柄（消除陀螺仪漂移）
                print("[HOME] 🔄 recalibrating joycons...")
                self.left.reset_joycon()
                self.right.reset_joycon()
                print("[HOME] ✅ joycons recalibrated")
                
                # 重新初始化手柄：重置滤波器和参考框架
                self._filtered_pose = pose.copy()
                self.ctrl_ref_pose = None
                self.ctrl_ref_rot_inv = None
                self.ref_arm_pos = None
                self.ref_arm_rot = None
                
                self.arm_mode = "idle"

            return True   # 🔥 必须持续发送

        # ======================
        # 🟡 3. R → 进入初始化移动
        # ======================
        if edge_r and self.arm_mode == "idle":
            print("[ARM] 🚀 start move to init")
            self.arm_mode = "moving"

        # ======================
        # 🟡 4. moving（自动归位）
        # ======================
        if self.arm_mode == "moving":

            self.arm_pos = self.init_pos
            self.arm_quat = self.init_rot.as_quat()

            pos_err = np.linalg.norm(real_pos - self.init_pos)
            rot_err = (real_rot.inv() * self.init_rot).magnitude()

            print(f"[ARM] moving... pos_err={pos_err:.3f}, rot_err={rot_err:.3f}")

            if pos_err < 0.015 and rot_err < 0.08:
                print("[ARM] ✅ reached init → tracking")

                self.ctrl_ref_pose = pose.copy()
                self.ctrl_ref_rot_inv = ctrl_rot.inv()

                self.ref_arm_pos = real_pos.copy()
                self.ref_arm_rot = real_rot

                self.arm_mode = "tracking"

            return True

        # ======================
        # 🟢 5. tracking（增量控制）
        # ======================
        if self.arm_mode == "tracking" and r:

            delta_p = pose[:3] - self.ctrl_ref_pose[:3]
            delta_r = ctrl_rot * self.ctrl_ref_rot_inv
            
            # 位置缩放因子（0.0~1.0，值越小位置增量越小）
            position_scale = 0.5
            delta_p_scaled = delta_p * position_scale
            
            # 旋转缩放因子（0.0~1.0，值越小旋转增量越小）
            rotation_scale = 0.8
            delta_euler = delta_r.as_euler('xyz')
            delta_euler_scaled = delta_euler * rotation_scale
            delta_r_scaled = R.from_euler('xyz', delta_euler_scaled)

            self.arm_pos = self.ref_arm_pos + delta_p_scaled
            self.arm_quat = (delta_r_scaled * self.ref_arm_rot).as_quat()

            return True

        # ======================
        # 🔵 6. 松开R → 冻结
        # ======================
        if self.arm_mode == "tracking" and not r:

            self.ref_arm_pos = self.arm_pos.copy()
            self.ref_arm_rot = R.from_quat(self.arm_quat)

            self.ctrl_ref_pose = pose.copy()
            self.ctrl_ref_rot_inv = ctrl_rot.inv()

            return False

        return False

    # ======================
    # send
    # ======================
    def _send(self, arm_active):

        # 确定控制模式
        if self.arm_mode == "homing":
            control_mode = "init"
        elif self.arm_mode == "moving":
            control_mode = "init"
        elif self.arm_mode == "tracking":
            control_mode = "tracking"
        else:  # idle
            control_mode = "idle"

        # base
        self.ws.send({
            "teleop_mode": "base",
            "position": {
                "x": float(self.base_cmd[0]),
                "y": float(self.base_cmd[1]),
                "z": 0.0
            },
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": math.sin(self.base_cmd[2] / 2),
                "w": math.cos(self.base_cmd[2] / 2)
            }
        })

        # 🔥 核心修复：moving阶段也要发
        if arm_active or self.arm_mode == "moving":
            self.ws.send({
                "teleop_mode": "arm",
                "control_mode": control_mode,
                "position": {
                    "x": float(self.arm_pos[0]),
                    "y": float(self.arm_pos[1]),
                    "z": float(self.arm_pos[2])
                },
                "orientation": {
                    "x": float(self.arm_quat[0]),
                    "y": float(self.arm_quat[1]),
                    "z": float(self.arm_quat[2]),
                    "w": float(self.arm_quat[3])
                },
                "gripper": int(self.gripper_state)
            })

    # ======================
    # step
    # ======================
    def step(self, obs):

        self._episode()

        if not self.enabled:
            if self.teleop_state == "episode_ended" and not self.episode_end_flag:
                self.episode_end_flag = True
                return "end_episode"

            if self.teleop_state == "reset_env":
                self.arm_mode = "idle"
                return "reset_env"

            return None

        self._base()
        arm_active = self._arm()
        self._send(arm_active)

        return {
            "base_active": True,
            "arm_active": arm_active,
            "base_pose": obs["base_pose"],
            "arm_pos": self.arm_pos.copy(),
            "arm_quat": self.arm_quat.copy(),
            "gripper_pos": np.array([0.0])
        }

    # ======================
    # reset
    # ======================
    def reset(self):
        self.enabled = False
        self.teleop_state = None
        self.arm_mode = "idle"

        self.ctrl_ref_pose = None
        self.ctrl_ref_rot_inv = None
        self.ref_arm_pos = None
        self.ref_arm_rot = None

        self.prev_r = False

        print("[RESET] waiting Y...")

        while self.teleop_state != "episode_started":
            self._episode()
            time.sleep(0.01)

        print("[RESET] start")

    # ======================
    # close
    # ======================
    def close(self):
        try:
            self.left.joycon.disconnect_device()
            self.right.joycon.disconnect_device()
        except:
            pass