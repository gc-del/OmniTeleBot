#!/usr/bin/env python3
"""
RealEnvWS — 实物推理环境（WebSocket + ZMQ 图像流）。

替代旧的 RealEnv (RPC + USB相机)，适配新的 robot_server WebSocket 架构。
配合 RemotePolicy 使用，实现与仿真推理一致的 get_obs / step / reset 接口。

Usage:
    python main.py --real \\
        --ws-url ws://192.168.1.220:6060 \\
        --image-host 192.168.1.220 \\
        --hand-cam-port 55555 --base-cam-port 55556
"""

import json
import logging
import math
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation as R

logger = logging.getLogger(__name__)

# ── WebSocket imports ────────────────────────────────────────
# Use the same ECO65WSClient as telegrip
_HERE = Path(__file__).resolve().parent
_TELEGRIP_ROOT = _HERE.parent / "telegrip"
if str(_TELEGRIP_ROOT) not in sys.path:
    sys.path.insert(0, str(_TELEGRIP_ROOT))

import websocket

# Copy the minimal WS client (avoid heavy telegrip deps)
class _RobotWSClient:
    """Minimal WebSocket client to the ECO65 robot server."""

    def __init__(self, url: str):
        self.url = url
        self.ws: Optional[websocket.WebSocket] = None
        self.connected = False
        self.latest_state: Optional[dict] = None
        self._lock = threading.Lock()
        self._connect()
        threading.Thread(target=self._recv_loop, daemon=True).start()

    def _connect(self):
        try:
            self.ws = websocket.WebSocket()
            self.ws.connect(self.url)
            self.connected = True
            print(f"[RealEnvWS] WebSocket connected: {self.url}")
        except Exception as e:
            print(f"[RealEnvWS] WebSocket connect fail: {e}")
            self.connected = False

    def _recv_loop(self):
        while True:
            if not self.connected:
                time.sleep(1)
                self._connect()
                continue
            try:
                data = json.loads(self.ws.recv())
                with self._lock:
                    self.latest_state = data
            except Exception:
                self.connected = False

    def send(self, msg: dict) -> bool:
        if not self.connected or not self.ws:
            return False
        try:
            self.ws.send(json.dumps(msg))
            return True
        except Exception:
            self.connected = False
            return False

    def get_state(self) -> Optional[dict]:
        with self._lock:
            return self.latest_state


# ═══════════════════════════════════════════════════════════════
#  ZMQ Image Collector
# ═══════════════════════════════════════════════════════════════
class _ImageCollector:
    """Collects camera images via ZMQ SUB in background."""

    def __init__(self, image_host: str, hand_cam_port: int = 0, base_cam_port: int = 0):
        self._host = image_host
        self._cam_ports: dict[str, int] = {}
        if hand_cam_port > 0:
            self._cam_ports["hand_camera"] = hand_cam_port
        if base_cam_port > 0:
            self._cam_ports["base_camera"] = base_cam_port

        self._running = True
        self._images: dict[str, tuple[float, np.ndarray]] = {}  # cam_name → (ts, jpg_bytes)
        self._lock = threading.Lock()

        if not self._cam_ports:
            print("[RealEnvWS] No camera ports configured — running without images")
            return

        # Lazy-import teleimager ZMQ subscriber
        _teleimager_roots = [
            _HERE.parent / "teleimager",
            _HERE.parent / "telegrip" / "teleimager",
        ]
        for _root in _teleimager_roots:
            _src = _root / "src"
            if _src.is_dir() and str(_src) not in sys.path:
                sys.path.insert(0, str(_src))

        try:
            from teleimager.image_client import ZMQ_SubscriberManager
        except ModuleNotFoundError:
            try:
                from teleimager.src.teleimager.image_client import ZMQ_SubscriberManager
            except ModuleNotFoundError:
                print("[RealEnvWS] WARNING: teleimager not found, running without images")
                return

        self._sub_manager = ZMQ_SubscriberManager.get_instance()
        threading.Thread(target=self._loop, daemon=True).start()
        print(f"[RealEnvWS] Image collector started: {list(self._cam_ports.keys())} @ {self._host}")

    def stop(self):
        self._running = False

    def _loop(self):
        while self._running:
            for cam_name, port in self._cam_ports.items():
                try:
                    img = self._sub_manager.subscribe(self._host, port, request_bgr=False)
                    if img and img.jpg:
                        with self._lock:
                            self._images[cam_name] = (img.timestamp, img.jpg)
                except Exception:
                    pass
            time.sleep(0.005)

    def get_rgb(self, cam_name: str) -> Optional[np.ndarray]:
        """Return latest image as RGB uint8 numpy array, or None."""
        import cv2 as cv
        with self._lock:
            entry = self._images.get(cam_name)
        if entry is None:
            return None
        _, jpg_bytes = entry
        # Decode JPEG (OpenCV reads as BGR)
        bgr = cv.imdecode(np.frombuffer(jpg_bytes, np.uint8), cv.IMREAD_COLOR)
        if bgr is None:
            return None
        return cv.cvtColor(bgr, cv.COLOR_BGR2RGB)


# ═══════════════════════════════════════════════════════════════
#  Protocol helpers (lightweight, same as telegrip's protocol_v2)
# ═══════════════════════════════════════════════════════════════
SCHEMA_VERSION = "v2.0.1"

def _make_command(cmd_id: str, control_mode: str, timeout_ms: int = 500) -> dict:
    return {
        "type": "robot_command",
        "schema_version": SCHEMA_VERSION,
        "command_id": cmd_id,
        "timestamp": time.time(),
        "timeout_ms": timeout_ms,
        "preempt_policy": "replace",
        "control_mode": control_mode,
    }


class RealEnvWS:
    """
    实物推理环境 — 接口兼容 MujocoEnv。

    get_obs() → {base_pose, arm_pos, arm_quat, gripper_pos, base_image, wrist_image}
    step(action) → 发送 arm ee_pose + base twist + gripper 命令
    """

    # 底盘 P 控制器参数（绝对位姿目标 → 速度命令）
    KP_LINEAR = 0.5        # 线速度 P 增益
    KP_ANGULAR = 1.0       # 角速度 P 增益
    MAX_LINEAR_VEL = 0.5   # m/s
    MAX_ANGULAR_VEL = 1.57 # rad/s

    def __init__(self, ws_url: str, image_host: str = "127.0.0.1",
                 hand_cam_port: int = 0, base_cam_port: int = 0):
        # WebSocket to robot
        self._ws = _RobotWSClient(ws_url)

        # Image collector
        # Camera name mapping: hand_camera → wrist_image (policy expects wrist_image)
        self._image_collector = _ImageCollector(image_host, hand_cam_port, base_cam_port)
        self._hand_cam_name = "hand_camera"
        self._base_cam_name = "base_camera"

        # Command counter
        self._cmd_counter = 0

        # Base P-controller state
        self._prev_base_target = None

        # Wait for first state
        print("[RealEnvWS] Waiting for robot state...")
        for _ in range(50):  # 5 seconds
            st = self._ws.get_state()
            if st and ("arm" in st or "arm_state" in st):
                print("[RealEnvWS] Robot state received")
                break
            time.sleep(0.1)
        else:
            print("[RealEnvWS] WARNING: No robot state received after 5s, proceeding anyway")

    def _next_cmd_id(self) -> str:
        self._cmd_counter += 1
        return f"policy_{self._cmd_counter:06d}"

    def _get_robot_state(self) -> dict:
        """Extract robot state from latest WS message."""
        st = self._ws.get_state() or {}
        result = {
            "base_pose": np.zeros(3, dtype=np.float64),     # [x, y, yaw]
            "arm_pos": np.zeros(3, dtype=np.float64),       # [x, y, z] base frame
            "arm_quat": np.array([0., 0., 0., 1.], dtype=np.float64),  # [x, y, z, w]
            "gripper_pos": np.array([0.], dtype=np.float64),  # [0-1]
        }

        # v2.0.1 format: {"arm": {"ee_pose": ...}, "base": {"pose": ...}}
        if "arm" in st and "ee_pose" in st.get("arm", {}):
            ee = st["arm"]["ee_pose"]
            result["arm_pos"] = np.array([
                ee["position"]["x"], ee["position"]["y"], ee["position"]["z"]
            ], dtype=np.float64)
            q = ee["orientation"]
            result["arm_quat"] = np.array([q["x"], q["y"], q["z"], q["w"]], dtype=np.float64)

        elif "arm_state" in st:
            # Legacy format
            arm = st["arm_state"]["pose"]
            result["arm_pos"] = np.array([
                arm["position"]["x"], arm["position"]["y"], arm["position"]["z"]
            ], dtype=np.float64)
            q = arm["orientation"]
            result["arm_quat"] = np.array([q["x"], q["y"], q["z"], q["w"]], dtype=np.float64)

        if "base" in st and "pose" in st.get("base", {}):
            bp = st["base"]["pose"]["position"]
            bo = st["base"]["pose"]["orientation"]
            result["base_pose"][:2] = [bp["x"], bp["y"]]
            # Compute yaw from quaternion
            r = R.from_quat([bo["x"], bo["y"], bo["z"], bo["w"]])
            euler = r.as_euler('xyz')
            result["base_pose"][2] = euler[2]  # yaw

        if "gripper" in st and "position_pct" in st.get("gripper", {}):
            result["gripper_pos"] = np.array(
                [float(st["gripper"]["position_pct"]) / 100.0], dtype=np.float64
            )

        return result

    # ── Public API (compatible with MujocoEnv) ─────────────

    def reset(self):
        """Reset for new episode — send stop, clear targets."""
        cmd = _make_command(self._next_cmd_id(), "stop", timeout_ms=100)
        self._ws.send(cmd)
        self._prev_base_target = None
        print("[RealEnvWS] Reset done (stop sent)")

    def get_obs(self):
        """Return observation dict in same format as MujocoEnv."""
        state = self._get_robot_state()

        obs = {
            "base_pose": state["base_pose"].copy(),
            "arm_pos": state["arm_pos"].copy(),
            "arm_quat": state["arm_quat"].copy(),
            "gripper_pos": state["gripper_pos"].copy(),
        }

        # Images (map camera names to what policy expects)
        # policy expects: base_image, wrist_image
        hand_img = self._image_collector.get_rgb(self._hand_cam_name)
        base_img = self._image_collector.get_rgb(self._base_cam_name)

        if hand_img is not None:
            obs["wrist_image"] = hand_img
        else:
            obs["wrist_image"] = np.zeros((480, 640, 3), dtype=np.uint8)

        if base_img is not None:
            obs["base_image"] = base_img
        else:
            obs["base_image"] = np.zeros((360, 640, 3), dtype=np.uint8)

        return obs

    def step(self, action: dict):
        """Execute action on real robot.

        action keys:
            base_pose:   [x, y, theta]  absolute target (world frame)
            arm_pos:     [x, y, z]      absolute target (base frame)
            arm_quat:    [x, y, z, w]   absolute target (base frame)
            gripper_pos: [0-1]          gripper position
            base_active: bool           (set by RemotePolicy)
            arm_active:  bool           (set by RemotePolicy)
        """
        # ── Arm command (absolute Cartesian target) ──
        if action.get("arm_active") and "arm_pos" in action:
            cmd = _make_command(self._next_cmd_id(), "arm_only", timeout_ms=500)
            cmd["arm_cmd"] = {
                "mode": "ee_pose",
                "frame_id": "arm_base",
                "position": {
                    "x": float(action["arm_pos"][0]),
                    "y": float(action["arm_pos"][1]),
                    "z": float(action["arm_pos"][2]),
                },
                "orientation": {
                    "x": float(action["arm_quat"][0]),
                    "y": float(action["arm_quat"][1]),
                    "z": float(action["arm_quat"][2]),
                    "w": float(action["arm_quat"][3]),
                },
            }
            self._ws.send(cmd)

        # ── Base command (P-controller: absolute target → velocity) ──
        if action.get("base_active") and "base_pose" in action:
            current = self._get_robot_state()["base_pose"]
            target = np.asarray(action["base_pose"][:3], dtype=np.float64)

            err_x = target[0] - current[0]
            err_y = target[1] - current[1]
            # Angle error (unwrapped)
            err_yaw = math.atan2(
                math.sin(target[2] - current[2]),
                math.cos(target[2] - current[2]),
            )

            vx = np.clip(self.KP_LINEAR * err_x, -self.MAX_LINEAR_VEL, self.MAX_LINEAR_VEL)
            vy = np.clip(self.KP_LINEAR * err_y, -self.MAX_LINEAR_VEL, self.MAX_LINEAR_VEL)
            omega = np.clip(self.KP_ANGULAR * err_yaw, -self.MAX_ANGULAR_VEL, self.MAX_ANGULAR_VEL)

            cmd = _make_command(self._next_cmd_id(), "base_only", timeout_ms=300)
            cmd["base_cmd"] = {
                "mode": "twist",
                "frame_id": "base_link",
                "linear": {"x": float(vx), "y": float(vy), "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": float(omega)},
            }
            self._ws.send(cmd)

        # ── Gripper command ──
        if "gripper_pos" in action:
            pct = int(np.clip(float(action["gripper_pos"][0]) * 100.0, 0, 100))
            cmd = _make_command(self._next_cmd_id(), "gripper_only", timeout_ms=500)
            cmd["gripper_cmd"] = {
                "mode": "position",
                "position_pct": pct,
            }
            self._ws.send(cmd)

    def close(self):
        """Disconnect and cleanup."""
        cmd = _make_command(self._next_cmd_id(), "stop", timeout_ms=100)
        self._ws.send(cmd)
        self._image_collector.stop()
        print("[RealEnvWS] Closed")
