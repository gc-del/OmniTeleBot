# Author: ZMAI
#
# Direct keyboard teleoperation for simulation.

import threading
import time

import numpy as np
from pynput import keyboard
from scipy.spatial.transform import Rotation as R

from policies import Policy


class KeyboardRemotePolicy(Policy):
    """Keyboard teleoperation policy for simulation.

    Controls:
    - Space: start / end / reset episode flow
    - W/S: base x
    - A/D: base y
    - Q/E: base yaw
    - I/K: arm x
    - J/L: arm y
    - U/O: arm z
    - T/G: arm roll
    - Y/H: arm pitch
    - F/R: arm yaw
    - Z/X: gripper close / open

        Notes:
        - Base motion and arm motion are tracked independently.
        - Releasing all motion keys stops sending actions.
    """

    def __init__(self):
        self.state_lock = threading.Lock()
        self.keyboard_listener = None
        self.active_keys = set()

        self.teleop_state = None  # episode_started, episode_ended, reset_env
        self.keyboard_state = 'waiting_start'
        self.enabled = False
        self.episode_ended = False

        self.targets_initialized = False
        self.base_target_pose = None
        self.arm_target_pos = None
        self.arm_target_rot = None
        self.gripper_target_pos = None

        self.motion_started = False
        self.stop_command_sent = False

        self.base_step = 0.03
        self.base_yaw_step = 0.08
        self.arm_step = 0.01
        self.arm_rot_step = 0.08
        self.gripper_step = 0.05

        self.keyboard_listener = keyboard.Listener(on_press=self._on_key_press, on_release=self._on_key_release)
        self.keyboard_listener.start()

        print("\n" + "=" * 60)
        print("  Keyboard Teleop Mode Activated")
        print("=" * 60)
        print("  Press SPACE to start episode")
        print("  Base: W/S A/D Q/E")
        print("  Arm: I/K J/L U/O T/G Y/H F/R")
        print("  Gripper: Z/X")
        print("=" * 60 + "\n")

    def _key_to_char(self, key):
        if not hasattr(key, 'char') or key.char is None:
            return None
        return key.char.lower()

    def _on_key_press(self, key):
        try:
            if key == keyboard.Key.space:
                with self.state_lock:
                    self._handle_space_press()
                return

            if not self.enabled or not self.targets_initialized:
                return

            key_char = self._key_to_char(key)
            if key_char is None:
                return

            with self.state_lock:
                self.active_keys.add(key_char)
                self.motion_started = True
                self.stop_command_sent = False
        except Exception:
            pass

    def _on_key_release(self, key):
        try:
            key_char = self._key_to_char(key)
            if key_char is None:
                return

            with self.state_lock:
                self.active_keys.discard(key_char)
        except Exception:
            pass

    def _handle_space_press(self):
        if self.keyboard_state == 'waiting_start':
            print("\n[Keyboard] SPACE pressed - Starting episode...")
            self.teleop_state = 'episode_started'
            self.enabled = True
            self.keyboard_state = 'running'
            print("[Keyboard] Episode started.")
            print("[Keyboard] Press SPACE again to end episode.")
            print("[Keyboard] Use WASD/QE for base, IJKLUO/TGYHFR for arm, ZX for gripper.\n")

        elif self.keyboard_state == 'running':
            print("\n[Keyboard] SPACE pressed - Ending episode...")
            self.teleop_state = 'episode_ended'
            self.enabled = False
            self.keyboard_state = 'ended'
            print("[Keyboard] Episode ended!")
            print("[Keyboard] Press SPACE to reset environment.\n")

        elif self.keyboard_state == 'ended':
            print("\n[Keyboard] SPACE pressed - Resetting environment...")
            self.teleop_state = 'reset_env'
            self.keyboard_state = 'waiting_reset'
            print("[Keyboard] Environment will reset...")
            print("[Keyboard] Press SPACE to start next episode.\n")

        elif self.keyboard_state == 'waiting_reset':
            print("\n[Keyboard] SPACE pressed - Starting new episode...")
            self.teleop_state = 'episode_started'
            self.enabled = True
            self.keyboard_state = 'running'
            print("[Keyboard] New episode started.")
            print("[Keyboard] Press SPACE again to end episode.")
            print("[Keyboard] Use WASD/QE for base, IJKLUO/TGYHFR for arm, ZX for gripper.\n")

    def _sync_targets_to_observation(self, obs):
        self.base_target_pose = obs['base_pose'].copy()
        self.arm_target_pos = obs['arm_pos'].copy()
        self.arm_target_rot = R.from_quat(obs['arm_quat'])
        self.gripper_target_pos = obs['gripper_pos'].copy()

    def _apply_active_keys(self):
        key_chars = self.active_keys

        if 'w' in key_chars:
            self.base_target_pose[0] += self.base_step
        if 's' in key_chars:
            self.base_target_pose[0] -= self.base_step
        if 'a' in key_chars:
            self.base_target_pose[1] += self.base_step
        if 'd' in key_chars:
            self.base_target_pose[1] -= self.base_step
        if 'q' in key_chars:
            self.base_target_pose[2] += self.base_yaw_step
        if 'e' in key_chars:
            self.base_target_pose[2] -= self.base_yaw_step

        if 'i' in key_chars:
            self.arm_target_pos[0] += self.arm_step
        if 'k' in key_chars:
            self.arm_target_pos[0] -= self.arm_step
        if 'j' in key_chars:
            self.arm_target_pos[1] += self.arm_step
        if 'l' in key_chars:
            self.arm_target_pos[1] -= self.arm_step
        if 'u' in key_chars:
            self.arm_target_pos[2] += self.arm_step
        if 'o' in key_chars:
            self.arm_target_pos[2] -= self.arm_step

        if 't' in key_chars:
            self.arm_target_rot = R.from_rotvec(np.array([1.0, 0.0, 0.0]) * self.arm_rot_step) * self.arm_target_rot
        if 'g' in key_chars:
            self.arm_target_rot = R.from_rotvec(np.array([1.0, 0.0, 0.0]) * -self.arm_rot_step) * self.arm_target_rot
        if 'y' in key_chars:
            self.arm_target_rot = R.from_rotvec(np.array([0.0, 1.0, 0.0]) * self.arm_rot_step) * self.arm_target_rot
        if 'h' in key_chars:
            self.arm_target_rot = R.from_rotvec(np.array([0.0, 1.0, 0.0]) * -self.arm_rot_step) * self.arm_target_rot
        if 'f' in key_chars:
            self.arm_target_rot = R.from_rotvec(np.array([0.0, 0.0, 1.0]) * self.arm_rot_step) * self.arm_target_rot
        if 'r' in key_chars:
            self.arm_target_rot = R.from_rotvec(np.array([0.0, 0.0, 1.0]) * -self.arm_rot_step) * self.arm_target_rot

        if 'z' in key_chars:
            self.gripper_target_pos = np.clip(self.gripper_target_pos - self.gripper_step, 0.0, 1.0)
        if 'x' in key_chars:
            self.gripper_target_pos = np.clip(self.gripper_target_pos + self.gripper_step, 0.0, 1.0)

    def _build_action(self):
        arm_quat = self.arm_target_rot.as_quat()
        if arm_quat[3] < 0.0:
            np.negative(arm_quat, out=arm_quat)
        action = {}

        base_keys = {'w', 'a', 's', 'd', 'q', 'e'}
        arm_keys = {'i', 'j', 'k', 'l', 'u', 'o', 't', 'g', 'y', 'h', 'f', 'r'}
        gripper_keys = {'z', 'x'}

        if self.active_keys & base_keys:
            action['base_active'] = True
            action['base_pose'] = self.base_target_pose.copy()

        if self.active_keys & arm_keys:
            action['arm_active'] = True
            action['arm_pos'] = self.arm_target_pos.copy()
            action['arm_quat'] = arm_quat

        if self.active_keys & gripper_keys:
            action['arm_active'] = True
            action['gripper_pos'] = self.gripper_target_pos.copy()

        return action

    def reset(self):
        self.episode_ended = False

        with self.state_lock:
            self.active_keys.clear()
            self.motion_started = False
            self.stop_command_sent = False
            if not self.enabled:
                self.keyboard_state = 'waiting_start'
                self.teleop_state = None
            self.targets_initialized = False

        print("[Keyboard] Ready. Press SPACE once to start controlling.")

    def step(self, obs):
        if not self.targets_initialized:
            self._sync_targets_to_observation(obs)
            self.targets_initialized = True

        if not self.enabled:
            if self.teleop_state == 'episode_ended' and not self.episode_ended:
                self.episode_ended = True
                return 'end_episode'
            if self.teleop_state == 'reset_env':
                return 'reset_env'
            return None

        with self.state_lock:
            if self.active_keys:
                self._apply_active_keys()
                return self._build_action()

            if self.motion_started and not self.stop_command_sent:
                self._sync_targets_to_observation(obs)
                self.stop_command_sent = True
                return self._build_action()

        return None

    def close(self):
        if self.keyboard_listener is not None:
            self.keyboard_listener.stop()


if __name__ == '__main__':
    import numpy as np
    from constants import POLICY_CONTROL_PERIOD

    obs = {
        'base_pose': np.zeros(3),
        'arm_pos': np.zeros(3),
        'arm_quat': np.array([0.0, 0.0, 0.0, 1.0]),
        'gripper_pos': np.zeros(1),
        'base_image': np.zeros((640, 360, 3), dtype=np.uint8),
        'wrist_image': np.zeros((640, 480, 3), dtype=np.uint8),
    }

    policy = KeyboardRemotePolicy()
    try:
        policy.reset()
        while True:
            print(policy.step(obs))
            time.sleep(POLICY_CONTROL_PERIOD)
    except KeyboardInterrupt:
        pass
    finally:
        policy.close()

