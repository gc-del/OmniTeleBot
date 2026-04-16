import threading
import time
import sys

import numpy as np
from pynput import keyboard
from scipy.spatial.transform import Rotation as R

from policies import Policy


class KeyboardRemotePolicy(Policy):
    """
    LeRobot-style keyboard teleop:
    - key -> velocity
    - release -> stop
    - integration in step()
    """

    def __init__(self):

        # ---------------------------
        # threading / keyboard
        # ---------------------------
        self.state_lock = threading.Lock()
        self.keyboard_listener = None
        self.active_keys = set()

        # ---------------------------
        # state machine
        # ---------------------------
        self.teleop_state = None
        self.keyboard_state = 'waiting_start'
        self.enabled = False
        self.episode_ended = False

        # ---------------------------
        # targets (state)
        # ---------------------------
        self.targets_initialized = False
        self.base_target_pose = None
        self.arm_target_pos = None
        self.arm_target_rot = None
        self.gripper_target_pos = None

        # ---------------------------
        # velocity (LeRobot core)
        # ---------------------------
        self.arm_lin_vel = np.zeros(3)
        self.arm_ang_vel = np.zeros(3)
        self.gripper_vel = 0.0

        # speed scale
        self.lin_vel_scale = 0.5
        self.ang_vel_scale = 0.6
        self.gripper_scale = 0.5

        # base velocity (optional)
        self.base_vel = np.zeros(3)
        self.base_scale = 0.5

        # ---------------------------
        # listener
        # ---------------------------
        self.keyboard_listener = keyboard.Listener(
            on_press=self._on_key_press,
            on_release=self._on_key_release
        )
        self.keyboard_listener.start()

        print("\n==============================")
        print(" LeRobot-style Teleop ENABLED")
        print("==============================")
        print("W/S: base x vel")
        print("A/D: base y vel")
        print("Q/E: base yaw vel")
        print("I/K J/L U/O: arm xyz vel")
        print("T/G Y/H F/R: arm rot vel")
        print("Z/X: gripper")
        print("==============================\n")

    # ---------------------------
    # key utils
    # ---------------------------
    def _key_to_char(self, key):
        if not hasattr(key, 'char') or key.char is None:
            return None
        return key.char.lower()

    def _on_key_press(self, key):

        if key == keyboard.Key.space:
            with self.state_lock:
                self._handle_space()
            return

        if not self.enabled or not self.targets_initialized:
            return

        c = self._key_to_char(key)
        if c is None:
            return

        with self.state_lock:
            self.active_keys.add(c)

    def _on_key_release(self, key):
        c = self._key_to_char(key)
        if c is None:
            return
        with self.state_lock:
            self.active_keys.discard(c)

    # ---------------------------
    # space control
    # ---------------------------
    def _handle_space(self):
        if self.keyboard_state == 'waiting_start':
            self.enabled = True
            self.keyboard_state = 'running'
            self.teleop_state = 'episode_started'
            print("[START]")

        elif self.keyboard_state == 'running':
            self.enabled = False
            self.keyboard_state = 'ended'
            self.teleop_state = 'episode_ended'
            print("[END]")

        elif self.keyboard_state == 'ended':
            self.keyboard_state = 'waiting_reset'
            self.teleop_state = 'reset_env'
            print("[RESET]")

        elif self.keyboard_state == 'waiting_reset':
            self.keyboard_state = 'running'
            self.enabled = True
            self.teleop_state = 'episode_started'
            print("[START]")

    # ---------------------------
    # sync obs
    # ---------------------------
    def _sync_targets_to_observation(self, obs):
        self.base_target_pose = obs['base_pose'].copy()
        self.arm_target_pos = obs['arm_pos'].copy()
        self.arm_target_rot = R.from_quat(obs['arm_quat'])
        self.gripper_target_pos = obs['gripper_pos'].copy()

    # ---------------------------
    # LeRobot velocity compute
    # ---------------------------
    def _compute_velocity(self, keys):

        # reset
        self.arm_lin_vel[:] = 0
        self.arm_ang_vel[:] = 0
        self.gripper_vel = 0
        self.base_vel[:] = 0

        # ---------------- base ----------------
        if 'w' in keys:
            self.base_vel[0] += self.base_scale
        if 's' in keys:
            self.base_vel[0] -= self.base_scale
        if 'a' in keys:
            self.base_vel[1] += self.base_scale
        if 'd' in keys:
            self.base_vel[1] -= self.base_scale
        if 'q' in keys:
            self.base_vel[2] += self.base_scale
        if 'e' in keys:
            self.base_vel[2] -= self.base_scale

        # ---------------- arm linear ----------------
        if 'i' in keys:
            self.arm_lin_vel[0] += self.lin_vel_scale
        if 'k' in keys:
            self.arm_lin_vel[0] -= self.lin_vel_scale

        if 'j' in keys:
            self.arm_lin_vel[1] += self.lin_vel_scale
        if 'l' in keys:
            self.arm_lin_vel[1] -= self.lin_vel_scale

        if 'u' in keys:
            self.arm_lin_vel[2] += self.lin_vel_scale
        if 'o' in keys:
            self.arm_lin_vel[2] -= self.lin_vel_scale

        # ---------------- arm angular ----------------
        if 't' in keys:
            self.arm_ang_vel[0] += self.ang_vel_scale
        if 'g' in keys:
            self.arm_ang_vel[0] -= self.ang_vel_scale

        if 'y' in keys:
            self.arm_ang_vel[1] += self.ang_vel_scale
        if 'h' in keys:
            self.arm_ang_vel[1] -= self.ang_vel_scale

        if 'f' in keys:
            self.arm_ang_vel[2] += self.ang_vel_scale
        if 'r' in keys:
            self.arm_ang_vel[2] -= self.ang_vel_scale

        # ---------------- gripper ----------------
        if 'z' in keys:
            self.gripper_vel -= self.gripper_scale
        if 'x' in keys:
            self.gripper_vel += self.gripper_scale

    # ---------------------------
    # integration step (IMPORTANT)
    # ---------------------------
    def _integrate(self, dt):

        # base integrate
        self.base_target_pose += self.base_vel * dt

        # arm position
        self.arm_target_pos += self.arm_lin_vel * dt

        # arm rotation
        delta_rot = R.from_rotvec(self.arm_ang_vel * dt)
        self.arm_target_rot = delta_rot * self.arm_target_rot

        # gripper
        self.gripper_target_pos = np.clip(
            self.gripper_target_pos + self.gripper_vel * dt,
            0.0, 1.0
        )

    # ---------------------------
    # build action
    # ---------------------------
    def _build_action(self):

        arm_quat = self.arm_target_rot.as_quat()
        if arm_quat[3] < 0:
            np.negative(arm_quat, out=arm_quat)

        action = {}

        action['base_active'] = True
        action['base_pose'] = self.base_target_pose.copy()

        action['arm_active'] = True
        action['arm_pos'] = self.arm_target_pos.copy()
        action['arm_quat'] = arm_quat

        action['gripper_pos'] = self.gripper_target_pos.copy()

        return action

    # ---------------------------
    # main step
    # ---------------------------
    def step(self, obs):

        if not self.targets_initialized:
            self._sync_targets_to_observation(obs)
            self.targets_initialized = True

        # End-of-episode / reset handling
        if not self.enabled:
            if not self.episode_ended and self.teleop_state == 'episode_ended':
                self.episode_ended = True
                return 'end_episode'
            if self.teleop_state == 'reset_env':
                return 'reset_env'
            return None

        dt = 0.02  # or POLICY_CONTROL_PERIOD

        with self.state_lock:
            keys = set(self.active_keys)

        self._compute_velocity(keys)
        self._integrate(dt)

        return self._build_action()

    def reset(self):
        self.targets_initialized = False
        self.active_keys.clear()
        self.episode_ended = False
        self.teleop_state = None
        self.keyboard_state = 'waiting_start' if self.keyboard_state not in ['ended', 'waiting_reset'] else 'waiting_reset'

    def close(self):
        if self.keyboard_listener:
            self.keyboard_listener.stop()