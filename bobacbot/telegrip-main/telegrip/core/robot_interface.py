"""
Robot interface — supports both ECO65 (WebSocket Cartesian) and
SO100 (serial joint angles).  Joint limits, IK/FK solvers are kept
for PyBullet visualization compatibility when controlling ECO65.
"""

import json
import logging
import os
import sys
import threading
import time
import contextlib
from typing import Dict, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from ..config import (
    TelegripConfig, NUM_JOINTS, JOINT_NAMES,
    GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE,
)

logger = logging.getLogger(__name__)


@contextlib.contextmanager
def suppress_stdout_stderr():
    stdout_fd = sys.stdout.fileno()
    stderr_fd = sys.stderr.fileno()
    saved_stdout_fd = os.dup(stdout_fd)
    saved_stderr_fd = os.dup(stderr_fd)
    try:
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull_fd, stdout_fd)
        os.dup2(devnull_fd, stderr_fd)
        yield
    finally:
        os.dup2(saved_stdout_fd, stdout_fd)
        os.dup2(saved_stderr_fd, stderr_fd)
        os.close(saved_stdout_fd)
        os.close(saved_stderr_fd)
        os.close(devnull_fd)


class ECO65WSClient:
    """WebSocket client to the ECO65 ROS bridge."""

    def __init__(self, url: str):
        import websocket
        self.url = url
        self.ws: Optional[websocket.WebSocket] = None
        self.connected = False
        self.latest_state: Optional[dict] = None
        self._connect()
        threading.Thread(target=self._loop, daemon=True).start()

    def _connect(self):
        import websocket
        try:
            self.ws = websocket.WebSocket()
            self.ws.connect(self.url)
            self.connected = True
            logger.info(f"ECO65 WS connected: {self.url}")
        except Exception as e:
            logger.warning(f"ECO65 WS connect fail: {e}")
            self.connected = False

    def _loop(self):
        while True:
            if not self.connected:
                time.sleep(1)
                self._connect()
                continue
            try:
                self.latest_state = json.loads(self.ws.recv())
            except Exception:
                self.connected = False

    def send(self, msg: dict) -> bool:
        if not self.connected:
            return False
        try:
            self.ws.send(json.dumps(msg))
            return True
        except Exception:
            self.connected = False
            return False


class RobotInterface:
    """High-level interface for ECO65 robot via WebSocket Cartesian control."""

    def __init__(self, config: TelegripConfig):
        self.config = config
        self.ws: Optional[ECO65WSClient] = None
        self.is_connected = False
        self.is_engaged = False

        # Arm connection status
        self.left_arm_connected = False
        self.right_arm_connected = False

        # Latest robot state (from WS feedback)
        self.latest_arm_pos: Optional[np.ndarray] = None   # [x, y, z]
        self.latest_arm_quat: Optional[np.ndarray] = None  # [x, y, z, w]

        # Commanded targets (Cartesian)
        self.left_target_pos: Optional[np.ndarray] = None
        self.left_target_quat: Optional[np.ndarray] = None
        self.right_target_pos: Optional[np.ndarray] = None
        self.right_target_quat: Optional[np.ndarray] = None

        # Gripper
        self.left_gripper = 0
        self.right_gripper = 0

        # Control timing
        self.last_send_time = 0
        self.send_period = config.send_interval  # e.g. 0.01 = 100Hz

        # Error tracking
        self.send_errors = 0
        self.max_errors = 10

        # Control mode tracking
        self._arm_tracking = {"left": False, "right": False}

        # Base (chassis) control
        self.base_vx = 0.0
        self.base_vy = 0.0
        self.base_active = False

        # ── Joint angle state (for PyBullet IK / visualization) ──
        self.left_arm_angles = np.zeros(NUM_JOINTS)
        self.right_arm_angles = np.zeros(NUM_JOINTS)
        self.joint_limits_min_deg = np.full(NUM_JOINTS, -180.0)
        self.joint_limits_max_deg = np.full(NUM_JOINTS, 180.0)
        self.fk_solvers = {"left": None, "right": None}
        self.ik_solvers = {"left": None, "right": None}

        # SO100 compatibility (not used for ECO65)
        self.left_robot = None
        self.right_robot = None

        # Error tracking
        self.left_arm_errors = 0
        self.right_arm_errors = 0
        self.general_errors = 0
        self.max_arm_errors = 3
        self.max_general_errors = 8

    # ── IK / FK setup (for PyBullet visualization) ───────

    def setup_kinematics(self, physics_client, robot_ids: Dict, joint_indices: Dict,
                         end_effector_link_indices: Dict, joint_limits_min_deg: np.ndarray,
                         joint_limits_max_deg: np.ndarray):
        """Setup kinematics solvers using PyBullet (kept for visualization)."""
        from .kinematics import ForwardKinematics, IKSolver
        self.joint_limits_min_deg = joint_limits_min_deg.copy()
        self.joint_limits_max_deg = joint_limits_max_deg.copy()

        for arm in ["left", "right"]:
            self.fk_solvers[arm] = ForwardKinematics(
                physics_client, robot_ids[arm], joint_indices[arm],
                end_effector_link_indices[arm])
            self.ik_solvers[arm] = IKSolver(
                physics_client, robot_ids[arm], joint_indices[arm],
                end_effector_link_indices[arm],
                joint_limits_min_deg, joint_limits_max_deg, arm_name=arm)
        logger.info("Kinematics solvers initialized (PyBullet)")

    def get_current_end_effector_position(self, arm: str) -> np.ndarray:
        """Get current end effector position via FK if available, else WS state."""
        if self.fk_solvers.get(arm):
            angles = (self.left_arm_angles if arm == "left"
                      else self.right_arm_angles)
            position, _ = self.fk_solvers[arm].compute(angles)
            return position
        self._update_state()  # try fresh WS read before falling back
        if self.latest_arm_pos is not None:
            return self.latest_arm_pos.copy()
        return np.zeros(3)

    def get_current_end_effector_orientation(self):
        """Return current orientation as scipy Rotation."""
        if self.latest_arm_quat is not None:
            return R.from_quat(self.latest_arm_quat)
        return R.identity()

    def solve_ik(self, arm: str, target_position: np.ndarray,
                 target_orientation: Optional[np.ndarray] = None) -> np.ndarray:
        """Solve IK for the specified arm."""
        current_angles = (self.left_arm_angles if arm == "left"
                          else self.right_arm_angles)
        if self.ik_solvers.get(arm):
            return self.ik_solvers[arm].solve(
                target_position, target_orientation, current_angles)
        return current_angles[:3]

    def clamp_joint_angles(self, joint_angles: np.ndarray) -> np.ndarray:
        processed = joint_angles.copy()
        idx = 0  # shoulder_pan
        ang = processed[idx]
        lo, hi = self.joint_limits_min_deg[idx], self.joint_limits_max_deg[idx]
        if ang < lo or ang > hi:
            for off in [-360.0, 360.0]:
                w = ang + off
                if lo <= w <= hi:
                    processed[idx] = w
                    break
        return np.clip(processed, self.joint_limits_min_deg,
                       self.joint_limits_max_deg)

    def update_arm_angles(self, arm: str, ik_angles: np.ndarray,
                          wrist_flex: float, wrist_roll: float, gripper: float):
        """Update joint angles from IK + direct wrist/gripper."""
        target = (self.left_arm_angles if arm == "left"
                  else self.right_arm_angles)
        target[:3] = ik_angles
        target[3] = wrist_flex
        target[4] = wrist_roll
        target[5] = np.clip(gripper, GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE)
        clamped = self.clamp_joint_angles(target)
        clamped[5] = target[5]
        if arm == "left":
            self.left_arm_angles = clamped
        else:
            self.right_arm_angles = clamped

    def get_arm_angles(self, arm: str) -> np.ndarray:
        return (self.left_arm_angles.copy() if arm == "left"
                else self.right_arm_angles.copy())

    def get_actual_arm_angles(self, arm: str) -> np.ndarray:
        """Return joint angles (for PyBullet viz)."""
        return self.get_arm_angles(arm)

    def get_arm_angles_for_visualization(self, arm: str) -> np.ndarray:
        return self.get_arm_angles(arm)

    # ── Connection ────────────────────────────────────────

    def connect(self) -> bool:
        """Connect to ECO65 via WebSocket."""
        if self.is_connected:
            return True

        if not self.config.enable_robot:
            logger.info("Robot interface disabled in config")
            self.is_connected = True
            return True

        try:
            self.ws = ECO65WSClient(self.config.eco65_ws_url)
            # Wait briefly for connection
            for _ in range(30):
                if self.ws.connected:
                    break
                time.sleep(0.1)

            if not self.ws.connected:
                logger.error("Failed to connect to ECO65 WS")
                return False

            self.is_connected = True
            self.left_arm_connected = True
            self.right_arm_connected = True
            logger.info("Robot interface connected to ECO65")
            self._read_initial_state()
            return True

        except Exception as e:
            logger.error(f"Robot connection failed: {e}")
            return False

    def _read_initial_state(self):
        """Read initial arm state from WS feedback."""
        for _ in range(20):
            if self.ws and self.ws.latest_state:
                st = self.ws.latest_state
                if "arm_state" in st:
                    p = st["arm_state"]["pose"]["position"]
                    q = st["arm_state"]["pose"]["orientation"]
                    self.latest_arm_pos = np.array([p["x"], p["y"], p["z"]])
                    self.latest_arm_quat = np.array([q["x"], q["y"], q["z"], q["w"]])
                    logger.info(f"Initial arm state: pos={self.latest_arm_pos.round(2)}")
                    return
            time.sleep(0.05)
        logger.warning("Could not read initial arm state — will use zeros")

    # ── Engage / Disengage ────────────────────────────────

    def engage(self) -> bool:
        if not self.is_connected:
            return False
        self.is_engaged = True
        logger.info("Robot motors ENGAGED")
        return True

    def disengage(self) -> bool:
        self.is_engaged = False
        logger.info("Robot motors DISENGAGED")
        return True

    # ── Base control ──────────────────────────────────────

    def set_base_velocity(self, vx: float, vy: float):
        """Set base (chassis) velocity command."""
        self.base_vx = vx
        self.base_vy = vy
        self.base_active = (abs(vx) > 1e-6 or abs(vy) > 1e-6)

    def _send_base(self) -> bool:
        """Send base velocity command to ECO65."""
        return self.ws.send({
            "teleop_mode": "base",
            "position": {"x": float(self.base_vx), "y": float(self.base_vy), "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        })

    # ── Core send ─────────────────────────────────────────

    def send_command(self) -> bool:
        """Send Cartesian arm + base commands to ECO65 over WebSocket."""
        if not self.is_connected or not self.is_engaged:
            return False

        now = time.time()
        if now - self.last_send_time < self.send_period:
            return True  # rate-limited

        if not self.ws or not self.ws.connected:
            self.send_errors += 1
            return False

        try:
            # Update latest state
            self._update_state()

            success = True

            # Send base (left controller → chassis)
            if self.base_active:
                if not self._send_base():
                    success = False

            # Send arm (right controller → single arm, stored as "left" internally)
            if (self.left_arm_connected and
                self.left_target_pos is not None and
                self.left_target_quat is not None):
                if self._arm_tracking["left"]:
                    mode = "tracking"
                else:
                    mode = "init"
                    self._arm_tracking["left"] = True
                if not self._send_arm("left", self.left_target_pos,
                                      self.left_target_quat,
                                      self.left_gripper, mode):
                    success = False

            self.last_send_time = now
            self.send_errors = 0
            return success

        except Exception as e:
            logger.error(f"Error in send_command: {e}")
            self.send_errors += 1
            return False

    def _send_arm(self, arm: str, pos: np.ndarray, quat: np.ndarray,
                  gripper: int, control_mode: str) -> bool:
        """Send Cartesian arm command to ECO65."""
        return self.ws.send({
            "teleop_mode": "arm",
            "control_mode": control_mode,
            "position": {
                "x": float(pos[0]), "y": float(pos[1]), "z": float(pos[2])
            },
            "orientation": {
                "x": float(quat[0]), "y": float(quat[1]),
                "z": float(quat[2]), "w": float(quat[3])
            },
            "gripper": int(gripper),
        })

    def _update_state(self):
        """Update latest robot state from WS feedback."""
        if self.ws and self.ws.latest_state:
            st = self.ws.latest_state
            if "arm_state" in st:
                p = st["arm_state"]["pose"]["position"]
                q = st["arm_state"]["pose"]["orientation"]
                self.latest_arm_pos = np.array([p["x"], p["y"], p["z"]])
                self.latest_arm_quat = np.array([q["x"], q["y"], q["z"], q["w"]])

    # ── Arm control helpers ───────────────────────────────

    def set_arm_target(self, arm: str, pos: np.ndarray, quat: np.ndarray):
        """Set Cartesian target for specified arm."""
        if arm == "left":
            self.left_target_pos = pos.copy()
            self.left_target_quat = quat.copy()
        else:
            self.right_target_pos = pos.copy()
            self.right_target_quat = quat.copy()

    def set_gripper(self, arm: str, closed: bool):
        """Set gripper for specified arm. closed=True→0, closed=False→100."""
        val = 0 if closed else 100
        self.left_gripper = val   # single arm always stored as "left"

    def reset_arm_tracking(self, arm: str):
        """Reset tracking mode so next send uses 'init'."""
        self._arm_tracking[arm] = False

    # ── Query helpers ─────────────────────────────────────

    def get_arm_connection_status(self, arm: str) -> bool:
        return self.is_connected

    def get_current_end_effector_position(self, arm: str) -> np.ndarray:
        if self.latest_arm_pos is not None:
            return self.latest_arm_pos.copy()
        return np.zeros(3)

    def get_current_end_effector_orientation(self):
        """Return current orientation as scipy Rotation."""
        if self.latest_arm_quat is not None:
            return R.from_quat(self.latest_arm_quat)
        return R.identity()

    def get_arm_angles(self, arm: str) -> np.ndarray:
        """Return placeholder joint angles (6-DOF) for compatibility."""
        return np.zeros(6)

    def get_actual_arm_angles(self, arm: str) -> np.ndarray:
        return np.zeros(6)

    # ── Cleanup ───────────────────────────────────────────

    def return_to_initial_position(self):
        logger.info("Return to initial — not implemented for ECO65")

    def disable_torque(self, arm: str = None):
        logger.info("Disable torque — not implemented for ECO65")

    def disconnect(self):
        self.is_connected = False
        self.is_engaged = False
        self.left_arm_connected = False
        self.right_arm_connected = False
        logger.info("Robot disconnected")

    def update_arm_connection_status(self):
        self.left_arm_connected = self.is_connected
        self.right_arm_connected = self.is_connected

    @property
    def status(self) -> Dict:
        return {
            "connected": self.is_connected,
            "left_arm_connected": self.left_arm_connected,
            "right_arm_connected": self.right_arm_connected,
            "left_arm_angles": [0]*6,
            "right_arm_angles": [0]*6,
        }
