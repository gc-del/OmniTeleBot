# Author: Jimmy Wu
# Date: October 2024

import logging
import math
import os
import socket
import threading
import time
from queue import Queue, Empty as QueueEmpty
import cv2 as cv
import numpy as np
import zmq
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from scipy.spatial.transform import Rotation as R
from constants import POLICY_SERVER_HOST, POLICY_SERVER_PORT, POLICY_IMAGE_WIDTH, POLICY_IMAGE_HEIGHT

# Fix "Too many packets in payload" on Quest 3 (72 Hz)
import engineio.payload
engineio.payload.Payload.max_packets = 256

class Policy:
    def reset(self):
        raise NotImplementedError

    def step(self, obs):
        raise NotImplementedError

class WebServer:
    def __init__(self, queue):
        self.app = Flask(__name__)
        self.socketio = SocketIO(self.app, async_mode='threading')
        self.queue = queue

        @self.app.route('/')
        def index():
            return render_template('index.html')

        @self.socketio.on('message')
        def handle_message(data):
            # Send the timestamp back for RTT calculation (expected RTT on 5 GHz Wi-Fi is 7 ms)
            emit('echo', data['timestamp'])

            # DEBUG: Print received message (remove when confirmed working)
            if 'teleop_mode' in data:
                print(f"[RX] teleop_mode={data['teleop_mode']} device={data.get('device_id','?')[:6]} pos={data.get('position',{}).get('x',0):.3f},{data.get('position',{}).get('y',0):.3f},{data.get('position',{}).get('z',0):.3f}")
            elif 'state_update' in data:
                print(f"[RX] state_update={data['state_update']}")

            # Add data to queue for processing
            self.queue.put(data)

        # Reduce verbose Flask log output
        logging.getLogger('werkzeug').setLevel(logging.WARNING)

    def run(self):
        # Get IP address
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0)
        try:
            s.connect(('8.8.8.8', 1))
            address = s.getsockname()[0]
        except Exception:
            address = '127.0.0.1'
        finally:
            s.close()
        print(f'Starting server at {address}:5000')
        # Kill any leftover process on port 5000
        import subprocess
        subprocess.call(['fuser', '-k', '5000/tcp'], stderr=subprocess.DEVNULL)
        # Use SSL if cert/key files exist (required for WebXR on Quest 3)
        cert = os.path.expanduser('~/.bobacbot/cert.pem')
        key = os.path.expanduser('~/.bobacbot/key.pem')
        if os.path.exists(cert) and os.path.exists(key):
            print(f'HTTPS enabled. Open https://{address}:5000 on Quest 3')
            self.socketio.run(self.app, host='0.0.0.0', port=5000, ssl_context=(cert, key), allow_unsafe_werkzeug=True)
        else:
            print(f'HTTP mode. Generate certs: mkdir -p ~/.bobacbot && openssl req -x509 -newkey rsa:2048 -keyout ~/.bobacbot/key.pem -out ~/.bobacbot/cert.pem -days 365 -nodes -subj "/CN={address}"')
            self.socketio.run(self.app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)

DEVICE_CAMERA_OFFSET = np.array([0.0, 0.02, -0.04])  # iPhone 14 Pro
ARM_POSE_DEADZONE = 0.003   # 3 mm (was 10 mm)
ARM_ROT_DEADZONE = 0.01     # ~0.6° (was ~1.7°)
ARM_STATIONARY_FRAMES_REQUIRED = 10  # ~140ms (was ~400ms)
ARM_POSITION_SCALE = 1.0    # 1:1 tracking

# Convert coordinate system from WebXR to robot
def convert_webxr_pose(pos, quat):
    # WebXR: +x right, +y up, +z back; Robot: +x forward, +y left, +z up
    pos = np.array([-pos['z'], -pos['x'], pos['y']], dtype=np.float64)
    rot = R.from_quat([-quat['z'], -quat['x'], quat['y'], quat['w']])

    # Apply offset so that rotations are around device center instead of device camera
    pos = pos + rot.apply(DEVICE_CAMERA_OFFSET)

    return pos, rot


def rotation_angle(rotation):
    quat = rotation.as_quat()
    scalar = np.clip(abs(quat[3]), -1.0, 1.0)
    return 2.0 * np.arccos(scalar)

TWO_PI = 2 * math.pi

class TeleopController:
    def __init__(self):
        # Teleop device IDs
        self.primary_device_id = None    # Primary device controls either the arm or the base
        self.secondary_device_id = None  # Optional secondary device controls the base
        self.enabled_counts = {}

        # Mobile base pose
        self.base_pose = None

        # Teleop targets
        self.targets_initialized = False
        self.base_target_pose = None
        self.arm_target_pos = None
        self.arm_target_rot = None
        self.gripper_target_pos = None
        self.base_active = False
        self.arm_active = False

        # WebXR reference poses
        self.base_xr_ref_pos = None
        self.base_xr_ref_rot_inv = None
        self.arm_xr_ref_pos = None
        self.arm_xr_ref_rot_inv = None

        # Robot reference poses
        self.base_ref_pose = None
        self.arm_ref_pos = None
        self.arm_ref_rot = None
        self.arm_ref_base_pose = None  # For optional secondary control of base
        self.gripper_ref_pos = None
        self.arm_stationary_frames = 0
        self.arm_buttons = {'a': False, 'b': False, 'squeeze': False, 'thumbstick': False}

    def _capture_arm_reference(self, pos, rot):
        self.arm_xr_ref_pos = pos.copy()
        self.arm_xr_ref_rot_inv = rot.inv()
        self.arm_ref_pos = self.arm_target_pos.copy()
        self.arm_ref_rot = self.arm_target_rot
        self.arm_ref_base_pose = self.base_pose.copy()
        self.gripper_ref_pos = self.gripper_target_pos.copy()
        self.arm_stationary_frames = 0
        print(f"[ARM] ref captured, arm_ref_pos={self.arm_ref_pos}")

    def process_message(self, data):
        if not self.targets_initialized:
            print(f"[CTRL] targets_initialized=False, dropping message")
            return

        # Use device ID to disambiguate between primary and secondary devices
        device_id = data['device_id']

        # Update enabled count for the device that sent this message
        self.enabled_counts[device_id] = self.enabled_counts.get(device_id, 0) + 1 if 'teleop_mode' in data else 0

        # Assign primary and secondary devices
        if self.enabled_counts[device_id] > 2:
            if self.primary_device_id is None and device_id != self.secondary_device_id:
                # Note: We skip the first 2 steps because WebXR pose updates have higher latency than touch events
                self.primary_device_id = device_id
                print(f"[CTRL] primary_device_id set: {device_id}")
            elif self.secondary_device_id is None and device_id != self.primary_device_id:
                self.secondary_device_id = device_id
                print(f"[CTRL] secondary_device_id set: {device_id}")
        elif self.enabled_counts[device_id] == 0:
            if device_id == self.primary_device_id:
                self.primary_device_id = None
                self.secondary_device_id = None  # Allow other device to re-register as primary
                self.base_xr_ref_pos = None
                self.arm_xr_ref_pos = None
                self.base_active = False
                self.arm_active = False
                self.arm_stationary_frames = 0
                self.arm_buttons = {'a': False, 'b': False, 'squeeze': False, 'thumbstick': False}
                print(f"[CTRL] primary released: {device_id}")
            elif device_id == self.secondary_device_id:
                self.secondary_device_id = None
                if device_id.endswith('_L'):
                    self.base_xr_ref_pos = None
                    self.base_active = False
                elif device_id.endswith('_R'):
                    self.arm_xr_ref_pos = None
                    self.arm_active = False
                    self.arm_stationary_frames = 0
                    self.arm_buttons = {'a': False, 'b': False, 'squeeze': False, 'thumbstick': False}
                print(f"[CTRL] secondary released: {device_id}")

        # Teleop is enabled
        if self.primary_device_id is not None and 'teleop_mode' in data:
            pos, rot = convert_webxr_pose(data['position'], data['orientation'])

            # Base movement
            if data['teleop_mode'] == 'base':
                self.base_active = True
                pos, rot = convert_webxr_pose(data['position'], data['orientation'])
                # Store reference poses
                if self.base_xr_ref_pos is None:
                    self.base_ref_pose = self.base_pose.copy()
                    self.base_xr_ref_pos = pos[:2]
                    self.base_xr_ref_rot_inv = rot.inv()

                # Position
                self.base_target_pose[:2] = self.base_ref_pose[:2] + (pos[:2] - self.base_xr_ref_pos)

                # Orientation
                base_fwd_vec_rotated = (rot * self.base_xr_ref_rot_inv).apply([1.0, 0.0, 0.0])
                base_target_theta = self.base_ref_pose[2] + math.atan2(base_fwd_vec_rotated[1], base_fwd_vec_rotated[0])
                self.base_target_pose[2] += (base_target_theta - self.base_target_pose[2] + math.pi) % TWO_PI - math.pi  # Unwrapped

            # Arm movement
            elif data['teleop_mode'] == 'arm':
                self.arm_active = True
                pos, rot = convert_webxr_pose(data['position'], data['orientation'])
                buttons = data.get('buttons', {})
                recalibrate_arm = bool(buttons.get('b', False) and not self.arm_buttons.get('b', False))
                self.arm_buttons = {
                    'a': bool(buttons.get('a', False)),
                    'b': bool(buttons.get('b', False)),
                    'squeeze': bool(buttons.get('squeeze', False)),
                    'thumbstick': bool(buttons.get('thumbstick', False)),
                }

                if self.arm_xr_ref_pos is None or recalibrate_arm:
                    self._capture_arm_reference(pos, rot)
                    if recalibrate_arm:
                        print('[ARM] recalibrated from right controller B button')

                # Rotations around z-axis to go between global frame (base) and local frame (arm)
                z_rot = R.from_rotvec(np.array([0.0, 0.0, 1.0]) * self.base_pose[2])
                z_rot_inv = z_rot.inv()
                ref_z_rot = R.from_rotvec(np.array([0.0, 0.0, 1.0]) * self.arm_ref_base_pose[2])
                relative_rot = rot * self.arm_xr_ref_rot_inv
                raw_pos_diff = pos - self.arm_xr_ref_pos

                if np.linalg.norm(raw_pos_diff) < ARM_POSE_DEADZONE and rotation_angle(relative_rot) < ARM_ROT_DEADZONE:
                    self.arm_stationary_frames += 1
                    if self.arm_stationary_frames >= ARM_STATIONARY_FRAMES_REQUIRED:
                        self._capture_arm_reference(pos, rot)
                        raw_pos_diff = np.zeros(3, dtype=np.float64)
                        relative_rot = R.identity()
                else:
                    self.arm_stationary_frames = 0

                pos_diff = ARM_POSITION_SCALE * raw_pos_diff
                pos_diff += ref_z_rot.apply(self.arm_ref_pos) - z_rot.apply(self.arm_ref_pos)
                pos_diff[:2] += self.arm_ref_base_pose[:2] - self.base_pose[:2]

                prev = self.arm_target_pos.copy() if self.arm_target_pos is not None else None
                self.arm_target_pos = self.arm_ref_pos + z_rot_inv.apply(pos_diff)
                if prev is not None and np.linalg.norm(self.arm_target_pos - prev) > 0.001:
                    print(f"[ARM] moving: pos={self.arm_target_pos}, diff={np.linalg.norm(self.arm_target_pos - prev):.4f}m")

                # Orientation
                self.arm_target_rot = (z_rot_inv * relative_rot * ref_z_rot) * self.arm_ref_rot

                # Gripper position
                if 'a' in buttons:
                    self.gripper_target_pos = np.array([1.0 if buttons['a'] else 0.0], dtype=np.float64)
                elif 'gripper_value' in data:
                    self.gripper_target_pos = np.clip(np.array([data['gripper_value']], dtype=np.float64), 0.0, 1.0)
                else:
                    self.gripper_target_pos = np.clip(self.gripper_ref_pos + data['gripper_delta'], 0.0, 1.0)

        # Teleop is disabled
        elif self.primary_device_id is None:
            # Update target pose in case base is pushed while teleop is disabled
            self.base_target_pose = self.base_pose
            self.base_active = False
            self.arm_active = False

    def step(self, obs):
        # Update robot state
        self.base_pose = obs['base_pose']

        # Initialize targets
        if not self.targets_initialized:
            self.base_target_pose = obs['base_pose']
            self.arm_target_pos = obs['arm_pos']
            self.arm_target_rot = R.from_quat(obs['arm_quat'])
            self.gripper_target_pos = obs['gripper_pos']
            self.targets_initialized = True

        # Return no action if teleop is not enabled
        if self.primary_device_id is None:
            return None

        # Get most recent teleop command
        arm_quat = self.arm_target_rot.as_quat()
        if arm_quat[3] < 0.0:  # Enforce quaternion uniqueness (Note: Not strictly necessary since policy training uses 6D rotation representation)
            np.negative(arm_quat, out=arm_quat)
        action = {
            'base_pose': self.base_target_pose.copy(),
            'arm_pos': self.arm_target_pos.copy(),
            'arm_quat': arm_quat,
            'gripper_pos': self.gripper_target_pos.copy(),
        }

        return action

# Teleop using WebXR phone web app
class TeleopPolicy(Policy):
    def __init__(self):
        self.web_server_queue = Queue()
        self.teleop_controller = None
        self.teleop_state = None  # States: episode_started -> episode_ended -> reset_env
        self.episode_ended = False

        # Web server for serving the WebXR phone web app
        server = WebServer(self.web_server_queue)
        threading.Thread(target=server.run, daemon=True).start()

        # Listener thread to process messages from WebXR client
        threading.Thread(target=self.listener_loop, daemon=True).start()

    def reset(self):
        self.teleop_controller = TeleopController()
        self.episode_ended = False

        # Discard stale messages from previous episode/session
        while not self.web_server_queue.empty():
            self.web_server_queue.get()

        # Wait for user to signal that episode has started
        self.teleop_state = None
        while self.teleop_state != 'episode_started':
            time.sleep(0.01)

    def step(self, obs):
        # Signal that user has ended episode
        if not self.episode_ended and self.teleop_state == 'episode_ended':
            self.episode_ended = True
            return 'end_episode'

        # Signal that user is ready for env reset (after ending the episode)
        if self.teleop_state == 'reset_env':
            return 'reset_env'

        return self._step(obs)

    def _step(self, obs):
        return self.teleop_controller.step(obs)

    def listener_loop(self):
        _stale_warn = 0
        while True:
            try:
                if not self.web_server_queue.empty():
                    data = self.web_server_queue.get()

                    # Update state
                    if 'state_update' in data:
                        self.teleop_state = data['state_update']
                        print(f"[LOOP] state_update={data['state_update']}")

                    # Process message
                    elif self.teleop_controller is not None and ('teleop_mode' in data or 'device_id' in data):
                        # Auto-recover: if episode_started was missed because server restarted
                        # while Quest 3 XR session was already active.
                        # Only fire from None state to avoid overriding episode_ended/reset_env.
                        if self.teleop_state is None:
                            print(f"[LOOP] auto episode_started (reconnect recovery)")
                            self.teleop_state = 'episode_started'
                        self._process_message(data)

            except Exception as e:
                import traceback
                print(f"[LOOP] EXCEPTION in listener_loop: {e}")
                traceback.print_exc()

            time.sleep(0.001)

    def _process_message(self, data):
        self.teleop_controller.process_message(data)

# Execute policy running on remote server
class RemotePolicy(TeleopPolicy):
    control_period = 0.1  # 10Hz — match policy server inference rate

    def __init__(self):
        super().__init__()

        # Use phone as enabling device during policy rollout
        self.enabled = False

        # Connection to policy server
        context = zmq.Context()
        self.socket = context.socket(zmq.REQ)
        self.socket.connect(f'tcp://{POLICY_SERVER_HOST}:{POLICY_SERVER_PORT}')
        print(f'Connected to policy server at {POLICY_SERVER_HOST}:{POLICY_SERVER_PORT}')

    def reset(self):
        # Skip TeleopPolicy.reset() — no phone needed for policy rollout
        self.teleop_controller = TeleopController()
        self.episode_ended = False

        # Discard stale messages from previous episode/session
        while not self.web_server_queue.empty():
            self.web_server_queue.get()

        # Check connection to policy server and reset policy
        default_timeout = self.socket.getsockopt(zmq.RCVTIMEO)
        self.socket.setsockopt(zmq.RCVTIMEO, 1000)  # Temporarily set 1000 ms timeout
        self.socket.send_pyobj({'reset': True})
        try:
            self.socket.recv_pyobj()  # Note: Not secure. Only unpickle data you trust.
        except zmq.error.Again as e:
            raise Exception('Could not communicate with policy server') from e
        self.socket.setsockopt(zmq.RCVTIMEO, default_timeout)  # Put default timeout back

        # Auto-enable policy execution (no phone needed)
        self.enabled = True  # Note: Set to False to require phone confirmation

    def _step(self, obs):
        # Return teleop command if episode has ended
        if self.episode_ended:
            return self.teleop_controller.step(obs)

        # Return no action if robot is not enabled
        if not self.enabled:
            return None

        # Encode images
        encoded_obs = {}
        for k, v in obs.items():
            if v.ndim == 3:
                # Resize image to resolution expected by policy server
                v = cv.resize(v, (POLICY_IMAGE_WIDTH, POLICY_IMAGE_HEIGHT))

                # Encode image as JPEG
                _, v = cv.imencode('.jpg', v)  # Note: Interprets RGB as BGR
                encoded_obs[k] = v
            else:
                encoded_obs[k] = v

        # Send obs to policy server
        req = {'obs': encoded_obs}
        self.socket.send_pyobj(req)

        # Get action from policy server
        rep = self.socket.recv_pyobj()  # Note: Not secure. Only unpickle data you trust.
        action = rep['action']

        # Add required flags for simulation controller
        if action is not None:
            action['base_active'] = True
            action['arm_active'] = True

        return action

    def _process_message(self, data):
        if self.episode_ended:
            # Run teleop controller if episode has ended
            self.teleop_controller.process_message(data)
        else:
            # Enable policy execution if user is pressing on screen
            self.enabled = 'teleop_mode' in data

# VR teleop → real robot via WebSocket
class TeleopRealPolicy(TeleopPolicy):
    control_period = 0.02  # 20 Hz — give robot time to execute each target

    def __init__(self, websocket_url):
        super().__init__()
        import json as _json
        import websocket as _ws
        self._ws_url = websocket_url
        self._ws_conn = None
        self._ws_connected = False
        self._ws_state = None
        self._ws_send_q = Queue()
        self._robot_init_done = False
        self._latest_vr_data = None
        threading.Thread(target=self._ws_worker, args=(_json, _ws), daemon=True).start()

    def _ws_worker(self, json_mod, ws_mod):
        """Single-threaded send+recv, no thread-safety issues."""
        sent_cnt = 0
        recv_cnt = 0
        while True:
            if not self._ws_connected:
                try:
                    self._ws_conn = ws_mod.WebSocket()
                    self._ws_conn.connect(self._ws_url)
                    self._ws_connected = True
                    sent_cnt = recv_cnt = 0
                    print(f'[WS] connected: {self._ws_url}')
                except Exception as e:
                    print(f'[WS] connect fail: {e}')
                    self._ws_connected = False
                    time.sleep(1)
                    continue

            try:
                drained = 0
                while True:
                    try:
                        msg = self._ws_send_q.get_nowait()
                        self._ws_conn.send(json_mod.dumps(msg))
                        drained += 1
                    except QueueEmpty:
                        break
                if drained:
                    sent_cnt += drained
                    if sent_cnt <= 5:
                        print(f'[WS] sent #{sent_cnt} ({drained} drained)')

                self._ws_conn.settimeout(0.1)
                try:
                    raw = self._ws_conn.recv()
                    self._ws_state = json_mod.loads(raw)
                    recv_cnt += 1
                    if recv_cnt <= 3:
                        print(f'[WS] recv #{recv_cnt}')
                except Exception:
                    pass

                time.sleep(0.005)
            except Exception as e:
                print(f'[WS] error (sent={sent_cnt} recv={recv_cnt}): {e}')
                self._ws_connected = False
                try:
                    self._ws_conn.close()
                except Exception:
                    pass
                self._ws_conn = None

    @property
    def ws_connected(self):
        return self._ws_connected

    @property
    def ws_state(self):
        return self._ws_state

    def _ws_send(self, msg):
        if self._ws_send_q.qsize() < 10:
            self._ws_send_q.put(msg)

    def _process_message(self, data):
        if 'teleop_mode' in data and 'position' in data:
            self._latest_vr_data = data
        super()._process_message(data)

    def _step(self, obs):
        # Wait for real robot state before starting
        if not self._robot_init_done:
            if self.ws_state is not None and 'arm_state' in self.ws_state:
                self._robot_init_done = True
                print(f'[VR-REAL] robot state received, ready')
            else:
                if not hasattr(self, '_wait_cnt'):
                    self._wait_cnt = 0
                self._wait_cnt += 1
                if self._wait_cnt <= 10 or self._wait_cnt % 100 == 0:
                    print(f'[VR-REAL] waiting for robot state... (#{self._wait_cnt})')
                return None

        action = self.teleop_controller.step(obs)
        if not self.ws_connected:
            return action

        # Always send base heartbeat every cycle
        if action is not None and action.get('base_active'):
            bp = action['base_pose']
            if not hasattr(self, '_prev_base'):
                self._prev_base = bp.copy()
            vx = (bp[0] - self._prev_base[0]) / self.control_period
            vy = (bp[1] - self._prev_base[1]) / self.control_period
            self._prev_base = bp.copy()
        else:
            vx = vy = 0.0
        self._ws_send({
            'teleop_mode': 'base',
            'position': {'x': float(vx), 'y': float(vy), 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
        })

        # Arm: P-controller — desired = robot_ref + vr_cumulative, step clamped to current pos
        arm_active_now = action is not None and action.get('arm_active')
        if not hasattr(self, '_arm_was_active'):
            self._arm_was_active = False

        if arm_active_now and self._latest_vr_data and self.ws_state and 'arm_state' in self.ws_state:
            rp = self.ws_state['arm_state']['pose']['position']
            rq = self.ws_state['arm_state']['pose']['orientation']
            robot_pos = np.array([rp['x'], rp['y'], rp['z']], dtype=np.float64)
            robot_rot = R.from_quat([rq['x'], rq['y'], rq['z'], rq['w']])

            # ---- capture reference ONCE on trigger press (no auto re-reference) ----
            tc = self.teleop_controller
            if not self._arm_was_active and tc.arm_xr_ref_pos is not None:
                self._vr_ref_pos = tc.arm_xr_ref_pos.copy()
                self._vr_ref_rot_inv = tc.arm_xr_ref_rot_inv
                self._robot_ref_pos = robot_pos.copy()
                self._robot_ref_rot = robot_rot
                self._prev_smoothed = robot_pos.copy()
                print(f'[VR-ARM] ref captured, robot={self._robot_ref_pos}')

            if hasattr(self, '_vr_ref_pos'):
                # Compute VR delta since reference (unbounded — full tracking)
                vr_pos, vr_rot = convert_webxr_pose(
                    self._latest_vr_data['position'],
                    self._latest_vr_data['orientation'])
                raw_pos_delta = (vr_pos - self._vr_ref_pos) * ARM_POSITION_SCALE
                rot_delta_raw = vr_rot * self._vr_ref_rot_inv
                raw_rot_delta = R.from_rotvec(rot_delta_raw.as_rotvec() * 0.5)

                # "Desired" = where the robot SHOULD be according to VR movement
                desired_pos = self._robot_ref_pos + raw_pos_delta
                desired_rot = raw_rot_delta * self._robot_ref_rot

                # ---- P-controller: target = robot_current + clamp(desired - robot_current, max_step) ----
                pos_error = desired_pos - robot_pos
                pos_err_norm = np.linalg.norm(pos_error)
                step = pos_error
                MAX_POS_STEP = 0.012      # 1.2 cm → 0.24 m/s at 20Hz
                if pos_err_norm > MAX_POS_STEP:
                    step = pos_error * (MAX_POS_STEP / pos_err_norm)
                target_pos = robot_pos + step

                # Smooth with EMA
                target_pos = 0.5 * target_pos + 0.5 * self._prev_smoothed
                self._prev_smoothed = target_pos.copy()

                # Clamp rotation per-step
                rot_from_current = desired_rot * robot_rot.inv()
                rot_angle = rotation_angle(rot_from_current)
                MAX_ROT_STEP = 0.025      # ~1.4° → ~28°/s at 20Hz
                if rot_angle > MAX_ROT_STEP:
                    rot_from_current = R.from_rotvec(
                        rot_from_current.as_rotvec() * (MAX_ROT_STEP / rot_angle))
                target_rot = rot_from_current * robot_rot
                target_quat = target_rot.as_quat()

                control_mode = 'init' if not self._arm_was_active else 'tracking'
                gr = int(action['gripper_pos'][0] * 100)
                self._ws_send({
                    'teleop_mode': 'arm',
                    'control_mode': control_mode,
                    'position': {'x': float(target_pos[0]), 'y': float(target_pos[1]), 'z': float(target_pos[2])},
                    'orientation': {'x': float(target_quat[0]), 'y': float(target_quat[1]), 'z': float(target_quat[2]), 'w': float(target_quat[3])},
                    'gripper': gr,
                })
        elif arm_active_now:
            if not hasattr(self, '_diag_cnt'):
                self._diag_cnt = 0
            self._diag_cnt += 1
            if self._diag_cnt <= 5:
                ok_vr = bool(self._latest_vr_data)
                ok_ws = bool(self.ws_state and 'arm_state' in self.ws_state)
                print(f'[DIAG#{self._diag_cnt}] arm active but vr={"OK" if ok_vr else "MISS"} ws={"OK" if ok_ws else "MISS"}')
        self._arm_was_active = arm_active_now

        return action

if __name__ == '__main__':
    # WebServer(Queue()).run(); time.sleep(1000)
    # WebXRListener(); time.sleep(1000)
    from constants import POLICY_CONTROL_PERIOD
    obs = {
        'base_pose': np.zeros(3),
        'arm_pos': np.zeros(3),
        'arm_quat': np.array([0.0, 0.0, 0.0, 1.0]),
        'gripper_pos': np.zeros(1),
        'base_image': np.zeros((640, 360, 3)),
        'wrist_image': np.zeros((640, 480, 3)),
    }
    policy = TeleopPolicy()
    # policy = RemotePolicy()
    while True:
        policy.reset()
        for _ in range(100):
            print(policy.step(obs))
            time.sleep(POLICY_CONTROL_PERIOD)  # Note: Not precise
