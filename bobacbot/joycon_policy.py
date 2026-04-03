import threading
import time
import os

import hid
import numpy as np
from pynput import keyboard
from scipy.spatial.transform import Rotation as R

from constants import POLICY_CONTROL_PERIOD
from policies import Policy

from joyconrobotics import JoyconRobotics


def _normalize_stick(raw_value, deadzone=0.08):
    value = (raw_value - 2048.0) / 2048.0
    if abs(value) < deadzone:
        return 0.0
    return float(np.clip(value, -1.0, 1.0))


def _wrap_angle(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def _rotation_angle(rotation):
    quat = rotation.as_quat()
    scalar = np.clip(abs(quat[3]), -1.0, 1.0)
    return 2.0 * np.arccos(scalar)


def _check_joycon_permissions():
    joycon_paths = []
    for device in hid.enumerate(0, 0):
        if device.get('vendor_id') == 0x057E and device.get('product_id') in (0x2006, 0x2007):
            path = device.get('path')
            if path is not None:
                joycon_paths.append(path.decode() if isinstance(path, bytes) else str(path))

    if not joycon_paths:
        raise RuntimeError('No Joy-Con devices were detected by hidapi.')

    inaccessible = [path for path in joycon_paths if not os.access(path, os.R_OK | os.W_OK)]
    if inaccessible:
        message = [
            'Joy-Con devices are detected, but the current user cannot open the hidraw device nodes:',
        ]
        message.extend(f'  - {path}' for path in inaccessible)
        message.extend([
            '',
            'Fix options:',
            '  1. Temporary: run the app with sudo using the bobacbot interpreter:',
            '     sudo /home/wufuwei/miniconda3/envs/bobacbot/bin/python main.py --sim --joycon',
            '  2. Permanent: add a udev rule such as:',
            '     SUBSYSTEM=="hidraw", ATTRS{idVendor}=="057e", MODE="0660", GROUP="input"',
            '     then reload udev rules and replug the controllers.',
        ])
        raise PermissionError('\n'.join(message))


class JoyconRemotePolicy(Policy):
    control_period = 0.02

    def __init__(self):
        self.state_lock = threading.Lock()
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
        self.base_motion_active = False
        self.arm_motion_active = False

        self.base_linear_step = 0.025
        self.base_yaw_step = 0.06
        self.gripper_open_pos = 1.0
        self.gripper_close_pos = 0.0
        self.left_motion_deadzone = 0.15
        self.right_pose_deadzone = 0.01
        self.right_rot_deadzone = 0.03
        self.right_home_button_prev = False
        self.right_stationary_frames = 0
        self.right_stationary_frames_required = 30

        self.keyboard_listener = keyboard.Listener(on_press=self._on_key_press)

        print("\n" + "=" * 60)
        print("  Joy-Con Teleop Mode Activated")
        print("=" * 60)
        print("  Press SPACE to start episode")
        print("  Left Joy-Con: forward/back, left/right, yaw")
        print("  Right Joy-Con: end-effector pose + gripper")
        print("=" * 60 + "\n")

        _check_joycon_permissions()
        self.left = JoyconRobotics(device='left', without_rest_init=False)
        self.right = JoyconRobotics(device='right', without_rest_init=False)
        self.keyboard_listener.start()

    def _on_key_press(self, key):
        try:
            if key == keyboard.Key.space:
                with self.state_lock:
                    self._handle_space_press()
        except Exception:
            pass

    def _handle_space_press(self):
        if self.keyboard_state == 'waiting_start':
            print("\n[JoyCon] SPACE pressed - Starting episode...")
            self.teleop_state = 'episode_started'
            self.enabled = True
            self.keyboard_state = 'running'
            print("[JoyCon] Episode started.")
            print("[JoyCon] Press SPACE again to end episode.\n")

        elif self.keyboard_state == 'running':
            print("\n[JoyCon] SPACE pressed - Ending episode...")
            self.teleop_state = 'episode_ended'
            self.enabled = False
            self.keyboard_state = 'ended'
            print("[JoyCon] Episode ended!")
            print("[JoyCon] Press SPACE to reset environment.\n")

        elif self.keyboard_state == 'ended':
            print("\n[JoyCon] SPACE pressed - Resetting environment...")
            self.teleop_state = 'reset_env'
            self.keyboard_state = 'waiting_reset'
            print("[JoyCon] Environment will reset...")
            print("[JoyCon] Press SPACE to start next episode.\n")

        elif self.keyboard_state == 'waiting_reset':
            print("\n[JoyCon] SPACE pressed - Starting new episode...")
            self.teleop_state = 'episode_started'
            self.enabled = True
            print("[JoyCon] Press SPACE again to end episode.\n")

    def _sync_targets_to_observation(self, obs):
        self.base_target_pose = obs['base_pose'].copy()
        self.arm_target_pos = obs['arm_pos'].copy()
        current_arm_rot = R.from_quat(obs['arm_quat'])
        _, _, current_yaw = current_arm_rot.as_euler('xyz')
        self.arm_target_rot = R.from_euler('xyz', [0.0, -np.pi / 2.0, current_yaw])
        self.gripper_target_pos = obs['gripper_pos'].copy()
        self.gripper_target_pos[0] = self.gripper_open_pos if self.gripper_target_pos[0] > 0.5 else self.gripper_close_pos

    def _read_left_base_motion(self):
        base_x = _normalize_stick(self.left.joycon.get_stick_left_vertical())
        base_y = -_normalize_stick(self.left.joycon.get_stick_left_horizontal())

        self.base_target_pose[0] += self.base_linear_step * base_x
        self.base_target_pose[1] += self.base_linear_step * base_y

        if self.left.joycon.get_button_l():
            self.base_target_pose[2] += self.base_yaw_step
        if self.left.joycon.get_button_zl():
            self.base_target_pose[2] -= self.base_yaw_step

        self.base_target_pose[2] = _wrap_angle(self.base_target_pose[2])

    def _left_base_motion_active(self):
        base_x = _normalize_stick(self.left.joycon.get_stick_left_vertical())
        base_y = _normalize_stick(self.left.joycon.get_stick_left_horizontal())
        return (
            abs(base_x) >= self.left_motion_deadzone or
            abs(base_y) >= self.left_motion_deadzone or
            self.left.joycon.get_button_l() or
            self.left.joycon.get_button_zl()
        )

    def _read_right_arm_motion(self):
        pose, gripper_state, _ = self.right.get_control()
        pose = np.asarray(pose, dtype=np.float64)

        controller_rot = R.from_euler('xyz', [-pose[3], -pose[4], pose[5]])

        home_pressed = bool(self.right.joycon.get_button_home())
        if home_pressed and not self.right_home_button_prev:
            self.right_reference_pose = pose.copy()
            self.right_reference_rot_inv = controller_rot.inv()
            self.arm_reference_pos = self.arm_target_pos.copy()
            self.arm_reference_rot = self.arm_target_rot
        self.right_home_button_prev = home_pressed

        if not hasattr(self, 'right_reference_pose') or self.right_reference_pose is None:
            self.right_reference_pose = pose.copy()
            self.right_reference_rot_inv = controller_rot.inv()
            self.arm_reference_pos = self.arm_target_pos.copy()
            self.arm_reference_rot = self.arm_target_rot

        delta_pos = pose[:3] - self.right_reference_pose[:3]
        relative_rot = controller_rot * self.right_reference_rot_inv

        if np.linalg.norm(delta_pos) < self.right_pose_deadzone and _rotation_angle(relative_rot) < self.right_rot_deadzone:
            self.right_stationary_frames += 1
            if self.right_stationary_frames >= self.right_stationary_frames_required:
                self.right_reference_pose = pose.copy()
                self.right_reference_rot_inv = controller_rot.inv()
                self.arm_reference_pos = self.arm_target_pos.copy()
                self.arm_reference_rot = self.arm_target_rot
                self.right_stationary_frames = 0
                delta_pos = np.zeros(3, dtype=np.float64)
                relative_rot = R.identity()
        else:
            self.right_stationary_frames = 0

        self.arm_target_pos = self.arm_reference_pos + delta_pos
        self.arm_target_rot = relative_rot * self.arm_reference_rot

        self.gripper_target_pos[0] = self.gripper_open_pos if gripper_state == self.right.gripper_open else self.gripper_close_pos

        self.gripper_target_pos = np.array([self.gripper_target_pos[0]], dtype=np.float64)
        return True

    def _build_action(self):
        arm_quat = self.arm_target_rot.as_quat()
        if arm_quat[3] < 0.0:
            np.negative(arm_quat, out=arm_quat)

        return {
            'base_active': self.base_motion_active,
            'arm_active': self.arm_motion_active,
            'base_pose': self.base_target_pose.copy(),
            'arm_pos': self.arm_target_pos.copy(),
            'arm_quat': arm_quat,
            'gripper_pos': self.gripper_target_pos.copy(),
        }

    def reset(self):
        self.episode_ended = False

        with self.state_lock:
            self.motion_started = False
            self.stop_command_sent = False
            self.base_motion_active = False
            self.arm_motion_active = False
            self.keyboard_state = 'waiting_start' if self.keyboard_state not in ['ended', 'waiting_reset'] else 'waiting_reset'
            self.teleop_state = None
            self.enabled = False
            self.targets_initialized = False
            self.right_reference_pose = None
            self.right_reference_rot_inv = None
            self.arm_reference_pos = None
            self.arm_reference_rot = None
            self.right_home_button_prev = False
            self.right_stationary_frames = 0

        while self.teleop_state != 'episode_started':
            time.sleep(0.01)

        print('[JoyCon] Episode is ready. Use the controllers to teleoperate.')

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
            base_motion_active = self._left_base_motion_active()
            if base_motion_active:
                self._read_left_base_motion()
            else:
                self.base_target_pose = obs['base_pose'].copy()

            self.base_motion_active = base_motion_active
            self.arm_motion_active = self._read_right_arm_motion()

            return self._build_action()

    def close(self):
        if self.keyboard_listener is not None:
            self.keyboard_listener.stop()

        try:
            self.left.running = False
            self.right.running = False
        except Exception:
            pass

        try:
            self.left.joycon.disconnect_device()
        except Exception:
            pass

        try:
            self.right.joycon.disconnect_device()
        except Exception:
            pass


if __name__ == '__main__':
    obs = {
        'base_pose': np.zeros(3),
        'arm_pos': np.zeros(3),
        'arm_quat': np.array([0.0, 0.0, 0.0, 1.0]),
        'gripper_pos': np.zeros(1),
    }

    policy = JoyconRemotePolicy()
    try:
        policy.reset()
        while True:
            print(policy.step(obs))
            time.sleep(policy.control_period)
    except KeyboardInterrupt:
        pass
    finally:
        policy.close()