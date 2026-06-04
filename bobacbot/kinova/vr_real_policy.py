#!/usr/bin/env python3
"""
VR (WebXR) teleop → real robot via WebSocket.

Usage:  python main.py --sim --vr-real --websocket-url ws://<ip>:6060

Quest 3 right trigger = arm, left trigger = base.
"""

import json
import logging
import math
import os
import socket
import subprocess
import threading
import time
from queue import Queue

import numpy as np
import websocket
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from scipy.spatial.transform import Rotation as R

from policies import Policy

# Fix Quest 3 72 Hz SocketIO overflow
import engineio.payload
engineio.payload.Payload.max_packets = 256

# ═══════════════════════════════════════════════════════════════
#  Tunable constants
# ═══════════════════════════════════════════════════════════════
# Simulation-identical position tracking: target = robot_ref + vr_delta
# Robot's trajectory planner handles speed/safety — we just send the absolute pose.
ARM_POSITION_SCALE = 0.8       # 手柄10cm→机器人8cm
ARM_ROTATION_SCALE = 1.0       # 1:1 VR → robot
MAX_POS_STEP       = 0.05      # safety clamp 5cm/step (large, only catches extreme jumps)
MAX_ROT_STEP       = 0.08      # safety clamp ~5°/step
MIN_SEND_DELTA     = 0.001     # don't re-send if target moved < 1mm
EMA_ALPHA          = 0.5       # light smoothing
WORKSPACE_RADIUS    = 0.25      # max 25cm from ref — prevents hitting joint limits

# Camera-to-controller offset for iPhone / Quest controller
DEVICE_CAMERA_OFFSET = np.array([0.0, 0.02, -0.04])


# ═══════════════════════════════════════════════════════════════
#  Coordinate conversion
# ═══════════════════════════════════════════════════════════════
def _webxr_to_robot(pos, quat):
    """WebXR (+x right, +y up, +z back) → robot (+x forward, +y left, +z up)."""
    p = np.array([-pos['z'], -pos['x'], pos['y']], dtype=np.float64)
    r = R.from_quat([-quat['z'], -quat['x'], quat['y'], quat['w']])
    return p + r.apply(DEVICE_CAMERA_OFFSET), r


def _rotation_angle(rot):
    q = rot.as_quat()
    return 2.0 * np.arccos(np.clip(abs(q[3]), -1.0, 1.0))


# ═══════════════════════════════════════════════════════════════
#  WebSocket client  (proven — same as joycon_real_policy)
# ═══════════════════════════════════════════════════════════════
class _WSClient:
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
            print(f'[WS] connected: {self.url}')
        except Exception as e:
            print(f'[WS] connect fail: {e}')
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

    def send(self, msg):
        if not self.connected:
            return False
        try:
            self.ws.send(json.dumps(msg))
            return True
        except Exception:
            self.connected = False
            return False


# ═══════════════════════════════════════════════════════════════
#  VR Real Policy
# ═══════════════════════════════════════════════════════════════
class VRRealPolicy(Policy):
    control_period = 0.02  # 50 Hz (arm handled in listener, step() is lightweight)

    def __init__(self, websocket_url):
        print('\n================================')
        print(' VR TELEOP → REAL ROBOT')
        print('================================\n')

        # ── robot WebSocket ──
        self._ws = _WSClient(websocket_url)

        # ── WebXR receiver state ──
        self._ep_state  = None   # episode_started / episode_ended / reset_env
        self._vr_queue  = Queue()

        # ── start WebXR server ──
        self._start_webxr()

        # ── tracking state ──
        self._base_on   = False
        self._prev_base = None

        self._base_on   = False
        self._prev_base = None

        print('[VR-REAL] Open https://<ip>:5000 on Quest 3')
        print('[VR-REAL] Right trigger → arm   Left trigger → base')

    # ── WebXR server (Flask + SocketIO) ──────────────────────
    def _start_webxr(self):
        app = Flask(__name__)
        sio = SocketIO(app, async_mode='threading')

        @app.route('/')
        def _index():
            return render_template('index.html')

        @sio.on('message')
        def _on_msg(data):
            emit('echo', data['timestamp'])
            self._vr_queue.put(data)

        logging.getLogger('werkzeug').setLevel(logging.WARNING)

        def _run():
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(0)
            try:
                s.connect(('8.8.8.8', 1))
                addr = s.getsockname()[0]
            except Exception:
                addr = '127.0.0.1'
            finally:
                s.close()
            print(f'[VR] server at https://{addr}:5000')
            subprocess.call(['fuser', '-k', '5000/tcp'], stderr=subprocess.DEVNULL)
            cert = os.path.expanduser('~/.bobacbot/cert.pem')
            key  = os.path.expanduser('~/.bobacbot/key.pem')
            if os.path.exists(cert) and os.path.exists(key):
                sio.run(app, host='0.0.0.0', port=5000, ssl_context=(cert, key))
            else:
                sio.run(app, host='0.0.0.0', port=5000)

        threading.Thread(target=_run, daemon=True).start()
        threading.Thread(target=self._vr_listener, daemon=True).start()

    def _vr_listener(self):
        """Process VR messages.  Arm commands are sent DIRECTLY from here
        (no step() polling delay) — VR data arrives → compute → send ~1 ms."""
        self._vr_arm  = {'pos': np.zeros(3), 'rot': R.identity(), 'active': False,
                         'grip_value': 0.0, 'buttons': {}}
        self._vr_base = {'pos': np.zeros(3), 'rot': R.identity(), 'active': False}

        # Arm tracking state (local to listener — no step() delay)
        _arm_on           = False
        _vr_ref_pos       = None
        _vr_ref_rot_inv   = None
        _robot_ref_pos    = None
        _robot_ref_rot    = None
        _prev_smooth      = None
        _last_sent_pos    = None
        _last_sent_quat   = None
        _last_arm_send    = 0.0

        while True:
            if not self._vr_queue.empty():
                data = self._vr_queue.get()

                if 'state_update' in data:
                    self._ep_state = data['state_update']

                elif 'teleop_mode' in data and 'position' in data:
                    pos, rot = _webxr_to_robot(data['position'],
                                               data['orientation'])

                    if data['teleop_mode'] == 'arm':
                        self._vr_arm = {'pos': pos, 'rot': rot, 'active': True,
                                        'grip_value': data.get('gripper_value', 0.0),
                                        'buttons':    data.get('buttons', {})}

                        # ── send arm command immediately (throttled to 33 Hz) ──
                        now = time.time()
                        if now - _last_arm_send < 0.05:
                            continue  # throttle 20 Hz
                        st = self._ws.latest_state
                        if st is None or 'arm_state' not in st:
                            continue
                        _last_arm_send = now

                        rp = st['arm_state']['pose']['position']
                        rq = st['arm_state']['pose']['orientation']
                        rpos = np.array([rp['x'], rp['y'], rp['z']], dtype=np.float64)
                        rrot = R.from_quat([rq['x'], rq['y'], rq['z'], rq['w']])

                        if not _arm_on:
                            # rising edge — capture reference (same algorithm as simulation TeleopController)
                            _vr_ref_pos     = pos.copy()
                            _vr_ref_rot_inv = rot.inv()
                            _robot_ref_pos  = rpos.copy()
                            _robot_ref_rot  = rrot
                            _prev_smooth    = rpos.copy()
                            _last_sent_pos  = rpos.copy()
                            _last_sent_quat = rrot.as_quat()
                            _arm_on = True
                            tq = rrot.as_quat()
                            self._ws.send({
                                'teleop_mode': 'arm', 'control_mode': 'init',
                                'position':    {'x': float(rpos[0]), 'y': float(rpos[1]), 'z': float(rpos[2])},
                                'orientation': {'x': float(tq[0]), 'y': float(tq[1]), 'z': float(tq[2]), 'w': float(tq[3])},
                                'gripper': 0,
                            })
                        else:
                            # Simulation-identical: target = robot_ref + vr_delta
                            vr_delta = (pos - _vr_ref_pos) * ARM_POSITION_SCALE
                            tp = _robot_ref_pos + vr_delta

                            # Smooth
                            tp = EMA_ALPHA * tp + (1.0 - EMA_ALPHA) * _prev_smooth
                            _prev_smooth = tp.copy()

                            # Workspace bubble: never exceed radius from ref
                            ws_delta = tp - _robot_ref_pos
                            ws_norm = np.linalg.norm(ws_delta)
                            if ws_norm > WORKSPACE_RADIUS:
                                ws_delta *= WORKSPACE_RADIUS / ws_norm
                                tp = _robot_ref_pos + ws_delta

                            # Rotation: same formula
                            rd = rot * _vr_ref_rot_inv
                            rd_scaled = R.from_rotvec(rd.as_rotvec() * ARM_ROTATION_SCALE)
                            tr = rd_scaled * _robot_ref_rot
                            tq = tr.as_quat()

                            # Safety clamp — only catches extreme jumps
                            step = tp - _last_sent_pos
                            sn = np.linalg.norm(step)
                            if sn > MAX_POS_STEP:
                                tp = _last_sent_pos + step * (MAX_POS_STEP / sn)

                            # Only send if target changed meaningfully
                            if sn < MIN_SEND_DELTA and _rotation_angle(tr * R.from_quat(_last_sent_quat).inv()) < 0.003:
                                continue

                            _last_sent_pos = tp.copy()
                            _last_sent_quat = tq.copy()

                            grip = int(data.get('gripper_value', 0) * 100) if data.get('gripper_value', 0) > 0.1 else 0
                            self._ws.send({
                                'teleop_mode': 'arm',
                                'control_mode': 'tracking',
                                'position':    {'x': float(tp[0]), 'y': float(tp[1]), 'z': float(tp[2])},
                                'orientation': {'x': float(tq[0]), 'y': float(tq[1]), 'z': float(tq[2]), 'w': float(tq[3])},
                                'gripper': grip,
                            })

                    elif data['teleop_mode'] == 'base':
                        self._vr_base = {'pos': pos, 'rot': rot, 'active': True}

                elif 'device_id' in data:
                    if data['device_id'].endswith('_R'):
                        self._vr_arm['active'] = False
                        _arm_on = False
                    elif data['device_id'].endswith('_L'):
                        self._vr_base['active'] = False
            else:
                time.sleep(0.001)

    # ── Policy interface ─────────────────────────────────────
    def reset(self):
        self._ep_state = None
        self._base_on   = False
        self._prev_base = None
        self._base_on   = False
        self._prev_base = None
        print('[VR-REAL] Press "Start episode" in VR')
        while self._ep_state != 'episode_started':
            time.sleep(0.01)
        print('[VR-REAL] Episode started')

    def step(self, obs):
        # Arm is handled in _vr_listener (zero polling delay).
        # Here: episode state + base heartbeat only.
        if self._ep_state == 'episode_ended':
            return 'end_episode'
        if self._ep_state == 'reset_env':
            return 'reset_env'

        # ── Base heartbeat ──
        vb = self._vr_base
        if vb['active']:
            bp = vb['pos'][:2]
            if not self._base_on:
                self._prev_base = bp.copy()
                self._base_on = True
            vx = (bp[0] - self._prev_base[0]) / self.control_period
            vy = (bp[1] - self._prev_base[1]) / self.control_period
            self._prev_base = bp.copy()
        else:
            vx = vy = 0.0
            self._base_on = False

        self._ws.send({
            'teleop_mode': 'base',
            'position':    {'x': float(vx), 'y': float(vy), 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
        })

        return None
