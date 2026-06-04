#!/usr/bin/env python3
"""
XLerobot-style VR teleop — dual-controller A-Frame WebXR → real robot.

Usage:  python main.py --sim --xl-vr --websocket-url ws://<ip>:6060

Key differences from vr_real_policy.py:
- A-Frame WebXR frontend (hand models in VR, better 3D visualization)
- Both controllers in one message (arm=right, base=left)
- Quaternion-based rotation tracking (no gimbal lock)
- Same robot WSClient and control algorithm as vr_real_policy.py
"""

import json, logging, math, os, socket, subprocess, threading, time
from queue import Queue

import numpy as np
import websocket
from flask import Flask, render_template_string
from flask_socketio import SocketIO, emit
from scipy.spatial.transform import Rotation as R

from policies import Policy

import engineio.payload
engineio.payload.Payload.max_packets = 256

# ═══════════════════════════════════════════════════════
#  A-Frame WebXR frontend (inline HTML template)
# ═══════════════════════════════════════════════════════
AFRAME_HTML = r"""<!DOCTYPE html>
<html><head>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1,user-scalable=no'>
<title>XL-VR Teleop</title>
<script src='https://aframe.io/releases/1.5.0/aframe.min.js'></script>
<script src='https://cdn.socket.io/4.7.5/socket.io.min.js'></script>
<style>
  body { margin:0; overflow:hidden; font:1rem monospace; }
  #info { position:fixed; top:10px; left:10px; color:#0f0; background:rgba(0,0,0,0.7);
          padding:8px 12px; border-radius:6px; z-index:10; max-width:90vw; }
  .btn { position:fixed; right:20px; padding:14px 28px; border:none; border-radius:8px;
         font-size:1.1rem; cursor:pointer; color:#fff; z-index:10; }
  #btnStart { bottom:120px; background:#2a2; }
  #btnStop  { bottom:60px;  background:#a22; }
</style></head><body>
<div id='info'>Connecting...</div>
<button id='btnStart' class='btn' onclick='sendState("episode_started")'>Start Episode</button>
<button id='btnStop'  class='btn' onclick='sendState("episode_ended")'>End Episode</button>

<a-scene renderer='antialias:true;colorManagement:true' vr-sender>
  <a-entity id='headset'    camera position='0 1.6 0' wasd-controls='enabled:false'></a-entity>
  <a-entity id='rightHand'  oculus-touch-controls='hand:right' hand-controls='hand:right'></a-entity>
  <a-entity id='leftHand'   oculus-touch-controls='hand:left'  hand-controls='hand:left'></a-entity>
  <a-sky color='#111'></a-sky>
</a-scene>

<script>
const socket = io();
const info   = document.getElementById('info');
let connected = false;

socket.on('connect', () => { connected = true; info.innerText = 'XL-VR Ready \\nTrigger→arm  Trigger→base'; });
socket.on('disconnect', () => { connected = false; info.innerText = 'Disconnected'; });
window.sendState = function(s) { socket.send({state_update:s}); info.innerText = 'State: '+s; };

// Track active state — read from navigator.getGamepads() inside A-Frame tick (XR render loop)
let armActive = false, baseActive = false;

AFRAME.registerComponent('vr-sender', {
  schema: { throttleMs: {default: 20} },
  init: function() {
    this.lastSend = 0;
    this.rh = document.getElementById('rightHand');
    this.lh = document.getElementById('leftHand');
    this.hs = document.getElementById('headset');
  },
  tick: function() {
    if (!connected) return;
    const now = Date.now();
    if (now - this.lastSend < this.data.throttleMs) return;
    this.lastSend = now;

    // Read gamepad state inside tick (works because we're in A-Frame's XR render loop)
    try {
      const gps = navigator.getGamepads();
      for (const gp of gps) {
        if (!gp || !gp.hand) continue;
        const trig = gp.buttons[0]?.value || 0;
        if (gp.hand === 'right') armActive = trig > 0.1;
        if (gp.hand === 'left')  baseActive = trig > 0.1;
      }
    } catch(e) {}

    // Read pose from object3D (always current inside A-Frame tick)
    const getPose = (el) => {
      if (!el || !el.object3D) return null;
      const p = el.object3D.position, q = el.object3D.quaternion;
      return {pos:{x:p.x,y:p.y,z:p.z}, quat:{x:q.x,y:q.y,z:q.z,w:q.w}};
    };

    const rp = getPose(this.rh), lp = getPose(this.lh), hp = getPose(this.hs);

    if (rp || lp) {
      socket.send({
        right: rp ? { position:rp.pos, quaternion:rp.quat, trigger: armActive ? 1 : 0, grip: false,
                       thumbX:0, thumbY:0, btnA:false, btnB:false } : null,
        left:  lp ? { position:lp.pos, quaternion:lp.quat, trigger: baseActive ? 1 : 0, grip: false,
                       thumbX:0, thumbY:0 } : null,
        head:  hp ? { position:hp.pos, quaternion:hp.quat } : null,
      });
    }
  }
});
</script></body></html>"""

# ═══════════════════════════════════════════════════════
#  Tunable constants  (same as vr_real_policy.py)
# ═══════════════════════════════════════════════════════
ARM_POSITION_SCALE = 0.8       # VR → robot position
ARM_ROTATION_SCALE = 1.0       # VR → robot rotation
MAX_POS_STEP       = 0.05      # safety clamp 5cm
MAX_ROT_STEP       = 0.08      # safety clamp ~5°
MIN_SEND_DELTA     = 0.001     # 1mm min to re-send
EMA_ALPHA          = 0.5       # smoothing

# WebXR → robot coordinate conversion
DEVICE_CAMERA_OFFSET = np.array([0.0, 0.02, -0.04])

def _webxr_to_robot(pos, quat):
    p = np.array([-pos['z'], -pos['x'], pos['y']], dtype=np.float64)
    r = R.from_quat([-quat['z'], -quat['x'], quat['y'], quat['w']])
    return p + r.apply(DEVICE_CAMERA_OFFSET), r


# ═══════════════════════════════════════════════════════
#  WSClient  (proven — same as joycon_real_policy)
# ═══════════════════════════════════════════════════════
class _WSClient:
    def __init__(self, url):
        self.url = url; self.ws = None; self.connected = False; self.latest_state = None
        self._connect()
        threading.Thread(target=self._loop, daemon=True).start()
    def _connect(self):
        try:
            self.ws = websocket.WebSocket(); self.ws.connect(self.url)
            self.connected = True; print(f'[WS] connected: {self.url}')
        except Exception as e:
            print(f'[WS] connect fail: {e}'); self.connected = False
    def _loop(self):
        while True:
            if not self.connected: time.sleep(1); self._connect(); continue
            try: self.latest_state = json.loads(self.ws.recv())
            except Exception: self.connected = False
    def send(self, msg):
        if not self.connected: return False
        try: self.ws.send(json.dumps(msg)); return True
        except Exception: self.connected = False; return False


# ═══════════════════════════════════════════════════════
#  XL VR Policy
# ═══════════════════════════════════════════════════════
class XLVRPolicy(Policy):
    control_period = 0.05  # 20 Hz

    def __init__(self, websocket_url):
        print('\n================================')
        print(' XL-VR TELEOP → REAL ROBOT')
        print('================================\n')
        self._ws = _WSClient(websocket_url)

        # VR tracking state
        self._ep_state   = None       # episode_started / ended / reset_env
        self._vr_arm_pos = None       # latest right-controller positions
        self._vr_arm_rot = None
        self._vr_arm_on  = False      # right grip = arm active
        self._vr_base_pos = None
        self._vr_base_on  = False     # left grip = base active

        # Arm reference tracking
        self._arm_active     = False
        self._vr_ref_pos     = None
        self._vr_ref_rot_inv = None
        self._robot_ref_pos  = None
        self._robot_ref_rot  = None
        self._prev_smooth    = None
        self._last_sent_pos  = None
        self._last_sent_quat = None

        # Base tracking
        self._base_on   = False
        self._prev_base = None

        # Start server + listener
        self._start_server()

        print('[XL-VR] Open https://<ip>:5000 on Quest 3')

    # ── Flask + SocketIO server ─────────────────────────────
    def _start_server(self):
        app = Flask(__name__)
        sio = SocketIO(app, async_mode='threading')

        @app.route('/')
        def _index():
            return render_template_string(AFRAME_HTML)

        @sio.on('message')
        def _on_msg(data):
            if 'state_update' in data:
                self._ep_state = data['state_update']
                print(f'[XL-VR] state={data["state_update"]}')
            elif 'right' in data or 'left' in data:
                if not hasattr(self, '_vr_msg_cnt'):
                    self._vr_msg_cnt = 0
                self._vr_msg_cnt += 1
                if self._vr_msg_cnt <= 5 or self._vr_msg_cnt % 100 == 0:
                    r = data.get('right', {})
                    l = data.get('left', {})
                    print(f'[XL-VR#{self._vr_msg_cnt}] R_trig={r.get("trigger",0):.2f} R_grip={r.get("grip",False)} '
                          f'L_trig={l.get("trigger",0):.2f} L_grip={l.get("grip",False)}')
                self._process_vr_frame(data)

        logging.getLogger('werkzeug').setLevel(logging.WARNING)

        def _run():
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.settimeout(0)
            try: s.connect(('8.8.8.8', 1)); addr = s.getsockname()[0]
            except: addr = '127.0.0.1'
            finally: s.close()
            print(f'[XL-VR] server at https://{addr}:5000')
            subprocess.call(['fuser', '-k', '5000/tcp'], stderr=subprocess.DEVNULL)
            cert = os.path.expanduser('~/.bobacbot/cert.pem')
            key  = os.path.expanduser('~/.bobacbot/key.pem')
            if os.path.exists(cert) and os.path.exists(key):
                sio.run(app, host='0.0.0.0', port=5000, ssl_context=(cert, key))
            else:
                sio.run(app, host='0.0.0.0', port=5000)

        threading.Thread(target=_run, daemon=True).start()

    def _process_vr_frame(self, data):
        """Parse dual-controller VR data and update tracking state."""
        now = time.time()

        # Right controller → arm (trigger OR grip activates)
        if data.get('right') and data['right'].get('position'):
            rd = data['right']
            pos, rot = _webxr_to_robot(rd['position'], rd['quaternion'])
            trigger_val = rd.get('trigger', 0)
            grip = rd.get('grip', False)
            active = trigger_val > 0.1 or grip

            if active:
                self._vr_arm_pos = pos
                self._vr_arm_rot = rot
                self._vr_arm_on  = True
                self._send_arm(now, pos, rot, trigger_val)
            else:
                self._vr_arm_on = False
                self._arm_active = False

        # Left controller → base (trigger OR grip activates)
        if data.get('left') and data['left'].get('position'):
            ld = data['left']
            pos, _ = _webxr_to_robot(ld['position'], ld['quaternion'])
            trigger_val = ld.get('trigger', 0)
            self._vr_base_pos = pos
            self._vr_base_on  = trigger_val > 0.1 or ld.get('grip', False)

    # ── Arm control (same algorithm as vr_real_policy.py) ────
    def _send_arm(self, now, pos, rot, trigger_val):
        if not hasattr(self, '_last_arm_time'):
            self._last_arm_time = 0
        if now - self._last_arm_time < 0.05:    # throttle 20 Hz
            return
        st = self._ws.latest_state
        if st is None or 'arm_state' not in st:
            if not hasattr(self, '_no_ws_cnt'):
                self._no_ws_cnt = 0
            self._no_ws_cnt += 1
            if self._no_ws_cnt <= 3:
                print(f'[XL-ARM] no robot WS state yet (ws_connected={self._ws.connected})')
            return
        self._last_arm_time = now

        rp = st['arm_state']['pose']['position']
        rq = st['arm_state']['pose']['orientation']
        rpos = np.array([rp['x'], rp['y'], rp['z']], dtype=np.float64)
        rrot = R.from_quat([rq['x'], rq['y'], rq['z'], rq['w']])

        if not self._arm_active:
            # Rising edge: capture references
            self._vr_ref_pos     = pos.copy()
            self._vr_ref_rot_inv = rot.inv()
            self._robot_ref_pos  = rpos.copy()
            self._robot_ref_rot  = rrot
            self._prev_smooth    = rpos.copy()
            self._last_sent_pos  = rpos.copy()
            self._last_sent_quat = rrot.as_quat()
            self._arm_active = True
            tq = rrot.as_quat()
            self._ws.send({
                'teleop_mode': 'arm', 'control_mode': 'init',
                'position':    {'x': float(rpos[0]), 'y': float(rpos[1]), 'z': float(rpos[2])},
                'orientation': {'x': float(tq[0]), 'y': float(tq[1]), 'z': float(tq[2]), 'w': float(tq[3])},
                'gripper': 0,
            })
            print(f'[XL-ARM] ref captured  robot={rpos}')
            return

        # Simulation-style: target = robot_ref + vr_delta
        vr_delta = (pos - self._vr_ref_pos) * ARM_POSITION_SCALE
        tp = self._robot_ref_pos + vr_delta
        tp = EMA_ALPHA * tp + (1.0 - EMA_ALPHA) * self._prev_smooth
        self._prev_smooth = tp.copy()

        rd = rot * self._vr_ref_rot_inv
        rd_scaled = R.from_rotvec(rd.as_rotvec() * ARM_ROTATION_SCALE)
        tr = rd_scaled * self._robot_ref_rot
        tq = tr.as_quat()

        # Safety clamp
        step = tp - self._last_sent_pos
        sn = np.linalg.norm(step)
        if sn > MAX_POS_STEP:
            tp = self._last_sent_pos + step * (MAX_POS_STEP / sn)
        if sn < MIN_SEND_DELTA:
            return
        self._last_sent_pos = tp.copy()
        self._last_sent_quat = tq.copy()

        grip = int(trigger_val * 100) if trigger_val > 0.1 else 0
        self._ws.send({
            'teleop_mode': 'arm',
            'control_mode': 'tracking',
            'position':    {'x': float(tp[0]), 'y': float(tp[1]), 'z': float(tp[2])},
            'orientation': {'x': float(tq[0]), 'y': float(tq[1]), 'z': float(tq[2]), 'w': float(tq[3])},
            'gripper': grip,
        })

    # ── Policy interface ──────────────────────────────────
    def reset(self):
        self._ep_state = None
        self._arm_active = False; self._vr_ref_pos = None
        self._vr_ref_rot_inv = None; self._robot_ref_pos = None
        self._robot_ref_rot = None; self._prev_smooth = None
        self._last_sent_pos = None; self._last_sent_quat = None
        self._base_on = False; self._prev_base = None
        print('[XL-VR] Press "Start Episode" in VR')
        while self._ep_state != 'episode_started':
            time.sleep(0.01)
        print('[XL-VR] Episode started')

    def step(self, obs):
        if self._ep_state == 'episode_ended': return 'end_episode'
        if self._ep_state == 'reset_env':    return 'reset_env'

        # Base heartbeat
        if self._vr_base_on and self._vr_base_pos is not None:
            bp = self._vr_base_pos[:2]
            if not self._base_on:
                self._prev_base = bp.copy(); self._base_on = True
            vx = (bp[0] - self._prev_base[0]) / self.control_period
            vy = (bp[1] - self._prev_base[1]) / self.control_period
            self._prev_base = bp.copy()
        else:
            vx = vy = 0.0; self._base_on = False

        self._ws.send({
            'teleop_mode': 'base',
            'position':    {'x': float(vx), 'y': float(vy), 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
        })
        return None
