#!/usr/bin/env python3
"""
Robot Control Protocol v2.0.1
Supports the new protocol specification for robot_server communication.

SCHEMA: v2.0.1
- type: robot_command, robot_state, command_ack, command_status
- control_mode: base_only, arm_only, gripper_only, whole_body, stop, hold
"""

import time
from dataclasses import dataclass
from typing import Optional, Dict, Any
from enum import Enum
import numpy as np
from scipy.spatial.transform import Rotation as R


class ControlMode(Enum):
    BASE_ONLY = "base_only"
    ARM_ONLY = "arm_only"
    GRIPPER_ONLY = "gripper_only"
    WHOLE_BODY = "whole_body"
    STOP = "stop"
    HOLD = "hold"


class PreemptPolicy(Enum):
    REPLACE = "replace"
    REJECT = "reject"
    QUEUE = "queue"


@dataclass
class Vector3:
    x: float
    y: float
    z: float

    def to_dict(self):
        return {"x": float(self.x), "y": float(self.y), "z": float(self.z)}


@dataclass
class Quaternion:
    x: float
    y: float
    z: float
    w: float

    def to_dict(self):
        return {"x": float(self.x), "y": float(self.y), "z": float(self.z), "w": float(self.w)}


SCHEMA_VERSION = "v2.0.1"


def _pos_dict(p: np.ndarray) -> dict:
    return Vector3(float(p[0]), float(p[1]), float(p[2])).to_dict()


def _quat_dict(q: np.ndarray) -> dict:
    return Quaternion(float(q[0]), float(q[1]), float(q[2]), float(q[3])).to_dict()


class ProtocolV2Helper:
    """Protocol v2.0.1 command helper — zero-dependency, pure functions."""

    def __init__(self, command_prefix: str = "cmd"):
        self.command_prefix = command_prefix
        self._cmd_counter = 0

    def next_command_id(self) -> str:
        self._cmd_counter += 1
        return f"{self.command_prefix}_{self._cmd_counter:06d}"

    def arm_command_ee_pose(
        self,
        position: np.ndarray,
        orientation: np.ndarray,    # quaternion [x, y, z, w]
        timeout_ms: int = 500,
        preempt_policy: str = "replace",
    ) -> Dict[str, Any]:
        cmd_id = self.next_command_id()
        return {
            "type": "robot_command",
            "schema_version": SCHEMA_VERSION,
            "command_id": cmd_id,
            "timestamp": time.time(),
            "timeout_ms": timeout_ms,
            "preempt_policy": preempt_policy,
            "control_mode": ControlMode.ARM_ONLY.value,
            "arm_cmd": {
                "mode": "ee_pose",
                "frame_id": "arm_base",
                "position": _pos_dict(position),
                "orientation": _quat_dict(orientation),
            },
        }

    def base_command_twist(
        self,
        linear: np.ndarray,
        angular: np.ndarray,
        timeout_ms: int = 300,
        preempt_policy: str = "replace",
    ) -> Dict[str, Any]:
        cmd_id = self.next_command_id()
        return {
            "type": "robot_command",
            "schema_version": SCHEMA_VERSION,
            "command_id": cmd_id,
            "timestamp": time.time(),
            "timeout_ms": timeout_ms,
            "preempt_policy": preempt_policy,
            "control_mode": ControlMode.BASE_ONLY.value,
            "base_cmd": {
                "mode": "twist",
                "frame_id": "base_link",
                "linear": _pos_dict(linear),
                "angular": _pos_dict(angular),
            },
        }

    def gripper_command(
        self,
        position_pct: int,
        timeout_ms: int = 500,
        preempt_policy: str = "replace",
    ) -> Dict[str, Any]:
        cmd_id = self.next_command_id()
        return {
            "type": "robot_command",
            "schema_version": SCHEMA_VERSION,
            "command_id": cmd_id,
            "timestamp": time.time(),
            "timeout_ms": timeout_ms,
            "preempt_policy": preempt_policy,
            "control_mode": ControlMode.GRIPPER_ONLY.value,
            "gripper_cmd": {
                "mode": "position",
                "position_pct": int(position_pct),
            },
        }

    def stop_command(self) -> Dict[str, Any]:
        cmd_id = self.next_command_id()
        return {
            "type": "robot_command",
            "schema_version": SCHEMA_VERSION,
            "command_id": cmd_id,
            "timestamp": time.time(),
            "timeout_ms": 100,
            "preempt_policy": "replace",
            "control_mode": ControlMode.STOP.value,
        }
