#!/usr/bin/env python3
"""
XLerobot VR pipeline → Reilman ECO65 arm + mobile base.
Replace ONLY the robot layer — XLerobot's VR infrastructure is kept intact.

Usage:
  python xlerobot_eco65_vr.py ws://192.168.1.220:6060
"""

import asyncio, json, logging, math, os, sys, threading, time
import numpy as np
import websocket
from scipy.spatial.transform import Rotation as Rot

# ── Add XLerobot paths ────────────────────────────────────
_XLEVR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      "xlevr")
_XLEVR = os.path.normpath(_XLEVR)
if _XLEVR not in sys.path:
    sys.path.insert(0, _XLEVR)

# Fix vr_monitor's hardcoded path before importing
import vr_monitor as _vm
_vm.XLEVR_PATH = _XLEVR

from vr_monitor import VRMonitor

logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s [%(levelname)s] %(message)s')
log = logging.getLogger("eco65-vr")

# ═══════════════════════════════════════════════════════════════
#  Tunable constants
# ═══════════════════════════════════════════════════════════════
POSITION_SCALE  = 0.3  # VR → robot
ROTATION_SCALE  = 1.0   # wrist → robot amplification
MAX_POS_STEP    = 0.05      # safety 5 cm — target-change clamp
MAX_ROT_STEP    = 1.5       # rad (~86°) — glitch guard only, rarely clamps
MIN_DELTA       = 0.0003    # 0.3 mm deadband
MIN_ROT_DELTA   = 0.0005    # ~0.03° deadband (radians)
EMA_ALPHA       = 0.95      # minimal position filtering (τ≈1ms at 100Hz)

# ═══════════════════════════════════════════════════════════════
#  WSClient  (proven, same as joycon_real_policy)
# ═══════════════════════════════════════════════════════════════
class WSClient:
    def __init__(self, url):
        self.url = url; self.ws = None; self.connected = False
        self.latest_state = None
        self._connect()
        threading.Thread(target=self._loop, daemon=True).start()
    def _connect(self):
        try:
            self.ws = websocket.WebSocket(); self.ws.connect(self.url)
            self.connected = True; log.info(f"robot WS connected: {self.url}")
        except Exception as e:
            log.warning(f"robot WS fail: {e}"); self.connected = False
    def _loop(self):
        while True:
            if not self.connected: time.sleep(1); self._connect(); continue
            try: self.latest_state = json.loads(self.ws.recv())
            except: self.connected = False
    def send(self, msg):
        if not self.connected: return False
        try: self.ws.send(json.dumps(msg)); return True
        except: self.connected = False; return False


# ═══════════════════════════════════════════════════════════════
#  Main VR → ECO65 control loop
# ═══════════════════════════════════════════════════════════════
def main():
    ws_url = sys.argv[1] if len(sys.argv) > 1 else "ws://192.168.1.220:6060"
    log.info(f"XLerobot VR → ECO65  ws={ws_url}")

    # ── 1. Start XLerobot VR pipeline ──────────────────────
    os.system('fuser -k 8443/tcp 2>/dev/null; fuser -k 8442/tcp 2>/dev/null; sleep 0.5')
    vr_monitor = VRMonitor()
    if not vr_monitor.initialize():
        log.error("VR monitor init failed"); return
    threading.Thread(
        target=lambda: asyncio.run(vr_monitor.start_monitoring()),
        daemon=True).start()
    log.info("XLerobot VR pipeline running")

    # ── 2. Connect to real robot ───────────────────────────
    ws = WSClient(ws_url)
    time.sleep(1.5)

    # ── 3. Tracking state ──────────────────────────────────
    # Arm
    arm_active     = False
    vr_ref_pos     = None
    robot_ref_pos  = None
    robot_ref_rot  = None
    wrist_r_ref    = 0.0        # wrist_roll_deg at trigger press
    wrist_f_ref    = 0.0        # wrist_flex_deg at trigger press
    prev_smooth    = None
    last_sent_pos  = None
    last_sent_quat = np.array([0.,0.,0.,1.])

    # Base
    base_on       = False
    prev_base     = None

    send_period   = 0.05     # 200 Hz

    log.info("Control loop started.  Right trigger → arm   Left trigger → base")

    _last_diag = 0

    try:
        while True:
            t0 = time.time()

            # ── Get VR data ────────────────────────────
            goals = vr_monitor.get_latest_goal_nowait()
            if goals is None:
                time.sleep(0.01); continue

            right = goals.get("right")
            left  = goals.get("left")

            # Periodic diagnostics: is VR data arriving?
            if time.time() - _last_diag > 5:
                _last_diag = time.time()
                has_r = right is not None
                has_l = left is not None
                ws_ok = ws.connected and ws.latest_state is not None
                r_pos = right.target_position if has_r and right.target_position is not None else None
                l_pos = left.target_position if has_l and left.target_position is not None else None
                log.info(f"[DIAG] VR L={'OK' if has_l else '--'} R={'OK' if has_r else '--'} "
                         f"WS={'OK' if ws_ok else '--'} "
                         f"Rpos={'%.2f,%.2f,%.2f' % tuple(r_pos) if r_pos is not None else '--'} "
                         f"Lpos={'%.2f,%.2f,%.2f' % tuple(l_pos) if l_pos is not None else '--'}")

            st = ws.latest_state
            if st is None or 'arm_state' not in st:
                time.sleep(0.01); continue

            # ── Base (left controller) ────────────────
            if left and left.target_position is not None:
                bp = left.target_position[:2]
                trig = (left.metadata or {}).get('trigger', 0)
                base_active = trig > 0.5

                if base_active:
                    if not base_on:
                        prev_base = bp.copy(); base_on = True
                    vx = (bp[0] - prev_base[0]) / send_period
                    vy = (bp[1] - prev_base[1]) / send_period
                    prev_base = bp.copy()
                else:
                    vx = vy = 0.0; base_on = False
            else:
                vx = vy = 0.0; base_on = False

            ws.send({
                'teleop_mode': 'base',
                'position':    {'x': float(vx), 'y': float(vy), 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            })

            # ── Arm (right controller) ────────────────
            if right and right.target_position is not None:
                vr_pos = right.target_position           # [x, y, z] absolute
                md = right.metadata or {}
                trig = md.get('trigger', 0)
                new_active = trig > 0.5

                if not arm_active and new_active:
                    # Rising edge — capture position + orientation references
                    rp = st['arm_state']['pose']['position']
                    rq = st['arm_state']['pose']['orientation']
                    rpos = np.array([rp['x'], rp['y'], rp['z']], dtype=np.float64)
                    rrot = Rot.from_quat([rq['x'], rq['y'], rq['z'], rq['w']])
                    vr_ref_pos     = vr_pos.copy()
                    robot_ref_pos  = rpos.copy()
                    robot_ref_rot  = rrot
                    # Capture wrist refs from VR server (wrist_roll/flex are pre-computed)
                    wrist_r_ref    = right.wrist_roll_deg or 0.0
                    wrist_f_ref    = right.wrist_flex_deg or 0.0
                    prev_smooth    = rpos.copy()
                    last_sent_pos  = rpos.copy()
                    last_sent_quat = rrot.as_quat()
                    arm_active     = True
                    tq = rrot.as_quat()
                    ws.send({
                        'teleop_mode': 'arm', 'control_mode': 'init',
                        'position':    {'x': float(rpos[0]), 'y': float(rpos[1]), 'z': float(rpos[2])},
                        'orientation': {'x': float(tq[0]), 'y': float(tq[1]), 'z': float(tq[2]), 'w': float(tq[3])},
                        'gripper': 0,
                    })
                    log.info(f"ARM ref captured  pos={rpos} q={tq.round(2)}")

                elif arm_active and new_active:
                    # ---- read robot state ----
                    rp = st['arm_state']['pose']['position']
                    rq = st['arm_state']['pose']['orientation']
                    rpos = np.array([rp['x'], rp['y'], rp['z']], dtype=np.float64)
                    rrot = Rot.from_quat([rq['x'], rq['y'], rq['z'], rq['w']])

                    # ---- position: WebXR delta → robot coords ----
                    webxr_delta = vr_pos - vr_ref_pos
                    robot_delta = np.array([
                        -webxr_delta[2], -webxr_delta[0], webxr_delta[1]
                    ], dtype=np.float64) * POSITION_SCALE
                    tp = robot_ref_pos + robot_delta
                    tp = EMA_ALPHA * tp + (1.0 - EMA_ALPHA) * prev_smooth
                    prev_smooth = tp.copy()

                    # ---- orientation: full quaternion (already in robot coords) ----
                    rel_quat_robot = md.get('rel_quat_robot')
                    if rel_quat_robot is not None:
                        delta_rot = Rot.from_quat(rel_quat_robot)
                        delta_rot = Rot.from_rotvec(
                            delta_rot.as_rotvec() * ROTATION_SCALE)
                    else:
                        # Fallback: reconstruct from axis-projected wrist angles
                        cur_r  = right.wrist_roll_deg or 0.0
                        cur_f  = right.wrist_flex_deg or 0.0
                        dr = math.radians(cur_r - wrist_r_ref) * ROTATION_SCALE
                        df = math.radians(cur_f - wrist_f_ref) * ROTATION_SCALE
                        delta_rot = Rot.from_euler('xyz', [dr, df, 0.0])
                    # Absolute target (same logic as position!)
                    desired_rot = delta_rot * robot_ref_rot
                    tr = desired_rot
                    tq = tr.as_quat()

                    # Target-change clamp: limit jump from last_sent_quat
                    # (NOT from rrot — that ties target speed to robot speed).
                    # Same pattern as position's clamp to last_sent_pos.
                    ts_rot = tr * Rot.from_quat(last_sent_quat).inv()
                    ts_angle = np.linalg.norm(ts_rot.as_rotvec())
                    if ts_angle > MAX_ROT_STEP:
                        ts_rot = Rot.from_rotvec(
                            ts_rot.as_rotvec() * MAX_ROT_STEP / ts_angle)
                        tr = ts_rot * Rot.from_quat(last_sent_quat)
                        tq = tr.as_quat()

                    # Clamp position step
                    step = tp - last_sent_pos
                    sn = np.linalg.norm(step)
                    if sn > MAX_POS_STEP:
                        step *= MAX_POS_STEP / sn
                    tp = last_sent_pos + step

                    # Rotation delta from last sent — decoupled from position
                    rn = np.linalg.norm(
                        (Rot.from_quat(last_sent_quat).inv() * tr).as_rotvec())
                    if sn > MIN_DELTA or rn > MIN_ROT_DELTA:
                        last_sent_pos  = tp.copy()
                        last_sent_quat = tq.copy()
                        ws.send({
                            'teleop_mode': 'arm',
                            'control_mode': 'tracking',
                            'position':    {'x': float(tp[0]), 'y': float(tp[1]), 'z': float(tp[2])},
                            'orientation': {'x': float(tq[0]), 'y': float(tq[1]), 'z': float(tq[2]), 'w': float(tq[3])},
                            'gripper': int(trig * 100) if trig > 0.1 else 0,
                        })

                elif arm_active and not new_active:
                    # Released
                    arm_active = False
                    vr_ref_pos = None
                    log.info("ARM released")

            # ── Rate limit ────────────────────────────
            elapsed = time.time() - t0
            if elapsed < send_period:
                time.sleep(send_period - elapsed)

    except KeyboardInterrupt:
        log.info("Stopped by user")

if __name__ == "__main__":
    main()
