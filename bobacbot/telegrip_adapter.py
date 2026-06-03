#!/usr/bin/env python3
"""
telegrip_adapter.py — bridges telegrip control_loop → bobacbot EpisodeWriter.

Usage (inside the main project):

    from telegrip_adapter import TelegripAdapter
    adapter = TelegripAdapter(
        ws_url="ws://192.168.1.220:6060",
        output_dir="data/episodes",
        save=True,
    )
    adapter.run()   # blocking; Ctrl+C to stop

This adapter does:
1.  Starts telegrip VR WebSocket server + HTTPS web UI
2.  Starts telegrip ControlLoop (robot WebSocket connection)
3.  Injects EpisodeWriter into the control loop so every action + observation
    is recorded automatically during an "episode" (VR grip active period).
"""

import asyncio
import logging
import sys
import time
import threading
from pathlib import Path
from typing import Optional

import cv2 as cv
import numpy as np

# ── path setup ──────────────────────────────────────────
_HERE = Path(__file__).resolve().parent
_TELEGRIP_ROOT = _HERE / "telegrip"   # telegrip/telegrip/  is the package
if str(_TELEGRIP_ROOT) not in sys.path:
    sys.path.insert(0, str(_TELEGRIP_ROOT))

# ── teleimager imports ──────────────────────────────────
_TELEIMAGER_SRC = _HERE / "teleimager" / "src"
if str(_TELEIMAGER_SRC) not in sys.path:
    sys.path.insert(0, str(_TELEIMAGER_SRC))

# ── telegrip imports ────────────────────────────────────
from telegrip.config import TelegripConfig, get_config_data
from telegrip.control_loop import ControlLoop
from telegrip.inputs.vr_ws_server import VRWebSocketServer
from telegrip.inputs.web_keyboard import WebKeyboardHandler
from telegrip.inputs.base import ControlGoal, ControlMode
from telegrip.main import HTTPSServer, TelegripSystem, get_local_ip, create_signal_handler

# ── bobacbot imports ────────────────────────────────────
sys.path.insert(0, str(Path(__file__).resolve().parent))
from episode_storage import EpisodeWriter

logger = logging.getLogger("telegrip.adapter")


# ═══════════════════════════════════════════════════════════
#  Background image collector (ZMQ SUB)
# ═══════════════════════════════════════════════════════════
class _ImageCollector:
    """Collects camera images + timestamps in background via ZMQ SUB.

    Starts immediately on creation — runs continuously in background.
    The EpisodeWriter stores image timestamps per camera for
    cross-modal alignment with robot action/observation data.
    """

    def __init__(self, image_host: str = "127.0.0.1", cam_topics: Optional[dict] = None):
        """
        Args:
            image_host: IP of the teleimager image server.
            cam_topics:  {cam_name: zmq_port}, e.g. {"hand_camera": 55555, "base_camera": 55556}
        """
        self._host = image_host
        self._cam_topics = cam_topics or {}
        self._running = True
        self._images: dict[str, tuple[float, np.ndarray]] = {}  # cam_name → (ts, jpg_bytes)
        self._lock = threading.Lock()

        # Lazy-import teleimager ZMQ subscriber
        from teleimager.image_client import ZMQ_SubscriberManager
        self._sub_manager = ZMQ_SubscriberManager.get_instance()

        # Start pulling frames immediately
        threading.Thread(target=self._loop, daemon=True).start()
        logger.info(f"[IMG] Collector started for {list(self._cam_topics.keys())} @ {self._host}")

    def stop(self):
        self._running = False

    def _loop(self):
        while self._running:
            for cam_name, port in self._cam_topics.items():
                try:
                    img = self._sub_manager.subscribe(self._host, port, request_bgr=False)
                    if img and img.jpg:
                        with self._lock:
                            self._images[cam_name] = (img.timestamp, img.jpg)
                except Exception:
                    pass
            time.sleep(0.005)  # 200 Hz poll

    def get_latest(self) -> dict[str, tuple[float, np.ndarray]]:
        """Return {cam_name: (timestamp_seconds, jpg_bytes)} snapshot."""
        with self._lock:
            return dict(self._images)

    def get_frame(self, cam_name: str) -> Optional[tuple[float, np.ndarray]]:
        with self._lock:
            return self._images.get(cam_name)


# ═══════════════════════════════════════════════════════════
#  Patched ControlLoop that records episodes
# ═══════════════════════════════════════════════════════════
class RecordingControlLoop(ControlLoop):
    """ControlLoop with manual episode recording (SPACE to start/stop)."""

    def __init__(self, *args, episode_writer: Optional[EpisodeWriter] = None,
                 image_collector: Optional[_ImageCollector] = None, output_dir: str = "data/episodes", **kwargs):
        super().__init__(*args, **kwargs)
        self._episode_writer = episode_writer
        self._image_collector = image_collector
        self._output_dir = output_dir
        self._episode_active = False
        self._episode_start_time = 0.0
        self._display_running = True
        self._pending_save = False

        # Start a dedicated recording timer thread — independent of control loop
        self._record_interval = 0.1  # 50 Hz
        self._record_thread = threading.Thread(target=self._record_loop, daemon=True)
        self._record_thread.start()

        print("\n" + "="*50)
        print("  按 空格键 开始/停止录制  |  'q' 关闭画面")
        print("="*50 + "\n")

        if self._image_collector:
            self._display_thread = threading.Thread(target=self._display_loop, daemon=True)
            self._display_thread.start()

    @property
    def episode_active(self) -> bool:
        return self._episode_active

    async def _execute_goal(self, goal: ControlGoal):
        await super()._execute_goal(goal)

    def _record_loop(self):
        """Independent recording thread — does NOT depend on robot connection."""
        _logged_first = False
        while True:
            time.sleep(self._record_interval)
            if not self._episode_active or self._episode_writer is None:
                continue

            if not _logged_first:
                logger.info("[EP] Recording started")
                _logged_first = True

            # Build observation dict — aligned with simulation format
            obs: dict = {}
            ri = self.robot_interface
            if ri is not None:
                # base_pose: [x, y, yaw] — from base.pose odometry
                if ri.latest_base_pos is not None and ri.latest_base_quat is not None:
                    from scipy.spatial.transform import Rotation as _R
                    _yaw = _R.from_quat(ri.latest_base_quat).as_euler('xyz')[2]
                    obs["base_pose"] = np.array([
                        ri.latest_base_pos[0], ri.latest_base_pos[1], _yaw
                    ], dtype=np.float64)
                else:
                    obs["base_pose"] = np.zeros(3, dtype=np.float64)
                # arm_pos
                obs["arm_pos"] = ri.latest_arm_pos.copy() if ri.latest_arm_pos is not None else np.zeros(3, dtype=np.float64)
                # arm_quat
                obs["arm_quat"] = ri.latest_arm_quat.copy() if ri.latest_arm_quat is not None else np.array([0., 0., 0., 1.], dtype=np.float64)
                # gripper_pos: normalized to [0, 1]
                obs["gripper_pos"] = np.array([ri.latest_gripper_pct / 100.0], dtype=np.float64)

            # Image placeholders (images go to MP4 separately, store None as in simulation)
            obs["base_image"] = None   # base_camera mp4
            obs["wrist_image"] = None  # hand_camera mp4

            # Build action dict — actual executed robot state (from odometry feedback)
            action: dict = {}
            if ri is not None:
                # base_pose: actual odometry
                if ri.latest_base_pos is not None and ri.latest_base_quat is not None:
                    from scipy.spatial.transform import Rotation as _R2
                    _yaw_a = _R2.from_quat(ri.latest_base_quat).as_euler('xyz')[2]
                    action["base_pose"] = np.array([ri.latest_base_pos[0], ri.latest_base_pos[1], _yaw_a], dtype=np.float64)
                else:
                    action["base_pose"] = np.zeros(3, dtype=np.float64)
                # arm_pos / arm_quat: actual end-effector pose from feedback
                action["arm_pos"] = ri.latest_arm_pos.copy() if ri.latest_arm_pos is not None else np.zeros(3, dtype=np.float64)
                action["arm_quat"] = ri.latest_arm_quat.copy() if ri.latest_arm_quat is not None else np.array([0., 0., 0., 1.], dtype=np.float64)
                # gripper_pos: actual gripper position from feedback
                action["gripper_pos"] = np.array([ri.latest_gripper_pct / 100.0], dtype=np.float64)

            # Collect images from ZMQ subscriber (store as named keys matching spec image names)
            # base_camera → base_image, hand_camera → wrist_image
            CAM_TO_OBS = {"base_camera": "base_image", "hand_camera": "wrist_image"}
            if self._image_collector:
                for cam_name, (ts, jpg_bytes) in self._image_collector.get_latest().items():
                    obs_key = CAM_TO_OBS.get(cam_name, cam_name)
                    obs[f"{obs_key}_jpg"] = jpg_bytes
                    obs[f"{obs_key}_ts"] = ts

            # Record step
            try:
                self._episode_writer.step(obs, action)
            except Exception as e:
                logger.error(f"[EP] step() failed: {e}", exc_info=True)
                self._episode_active = False

    def _display_loop(self):
        """OpenCV display thread: show camera frames + detect SPACE key.

        Handles y/n save prompt via a flag so input() never blocks this thread.
        """
        # Wait for first image data
        for _ in range(100):  # 10s timeout
            if self._image_collector and self._image_collector.get_latest():
                break
            time.sleep(0.1)

        cam_names = list(self._image_collector._cam_topics.keys()) if self._image_collector else []
        if not cam_names:
            logger.warning("[DISPLAY] No cameras configured, display disabled")
            return

        for name in cam_names:
            cv.namedWindow(name, cv.WINDOW_NORMAL)
            cv.resizeWindow(name, 640, 400)
        logger.info(f"[DISPLAY] Showing {cam_names}")

        def _save_prompt():
            """Ask y/n in a way that doesn't block the display loop."""
            if self._episode_writer is None or len(self._episode_writer) == 0:
                print("⚠️ 没有数据，按空格键重试\n")
                return
            duration = time.time() - self._episode_start_time
            # Use a separate thread for input() so cv2.waitKey still works
            result = [None]

            def _ask():
                try:
                    result[0] = input(f"\n录制时长: {duration:.1f}s, 共 {len(self._episode_writer)} 步\n保存 episode? (y/n): ").strip().lower()
                except EOFError:
                    result[0] = 'n'

            t = threading.Thread(target=_ask, daemon=True)
            t.start()
            t.join(timeout=30)
            if result[0] == 'y':
                self._episode_writer.flush_async()
                print("✅ 正在保存到磁盘...")
            else:
                print("🗑️ 已丢弃")
            print("\n⏸️ 按空格键开始下一轮录制\n")

        while self._display_running:
            if self._image_collector:
                latest = self._image_collector.get_latest()
                status_color = (0, 255, 0) if self._episode_active else (0, 0, 255)
                status_text = "REC" if self._episode_active else "STOP"

                for cam_name in cam_names:
                    frame_data = latest.get(cam_name)
                    if frame_data is not None:
                        ts, jpg_bytes = frame_data
                        np_arr = np.frombuffer(jpg_bytes, np.uint8)
                        bgr = cv.imdecode(np_arr, cv.IMREAD_COLOR)
                        if bgr is not None:
                            cv.putText(bgr, status_text, (10, 30),
                                       cv.FONT_HERSHEY_SIMPLEX, 1.0, status_color, 2)
                            cv.imshow(cam_name, bgr)

            key = cv.waitKey(10) & 0xFF

            # Check pending save prompt
            if self._pending_save:
                _save_prompt()
                self._pending_save = False

            if key == ord(' '):
                self._toggle_recording()
            elif key == ord('q'):
                logger.info("[DISPLAY] 'q' pressed — closing windows")
                cv.destroyAllWindows()
                self._display_running = False
                return

    def _toggle_recording(self):
        """Toggle episode recording on/off."""
        if not self._episode_active:
            # Create a fresh EpisodeWriter for each recording session
            if self._episode_writer is not None:
                self._episode_writer.wait_for_flush()
            record_fps = 1.0 / self._record_interval if self._record_interval > 0 else 30.0
            self._episode_writer = EpisodeWriter(self._output_dir, record_fps=record_fps)
            self._episode_active = True
            self._episode_start_time = time.time()
            logger.info("[EP] === RECORDING STARTED ===")
            print("\n🔴 录制中... 按空格键停止\n")
        else:
            self._episode_active = False
            logger.info(f"[EP] === RECORDING STOPPED ({(time.time() - self._episode_start_time):.1f}s) ===")
            self._pending_save = True


# ═══════════════════════════════════════════════════════════
#  Patched System that uses RecordingControlLoop
# ═══════════════════════════════════════════════════════════
class RecordingTelegripSystem(TelegripSystem):
    """TelegripSystem that creates a RecordingControlLoop."""

    def __init__(self, config: TelegripConfig, episode_writer: Optional[EpisodeWriter] = None,
                 image_collector: Optional[_ImageCollector] = None, output_dir: str = "data/episodes"):
        # Skip parent init — we redo it here with patched ControlLoop
        self.config = config
        self.command_queue = asyncio.Queue()
        self.control_commands_queue = __import__('queue').Queue(maxsize=10)

        self.https_server = HTTPSServer(config)
        self.vr_server = VRWebSocketServer(self.command_queue, config)
        self.web_keyboard_handler = WebKeyboardHandler(self.command_queue, config)
        self.control_loop = RecordingControlLoop(
            self.command_queue, config, self.control_commands_queue,
            episode_writer=episode_writer,
            image_collector=image_collector,
            output_dir=output_dir,
        )

        self.https_server.set_system_ref(self)
        self.control_loop.web_keyboard_handler = self.web_keyboard_handler
        self.web_keyboard_handler.disconnect_callback = lambda: self.add_control_command("robot_disconnect")

        self.tasks = []
        self.is_running = False
        self.main_loop = None


# ═══════════════════════════════════════════════════════════
#  Public adapter class
# ═══════════════════════════════════════════════════════════
class TelegripAdapter:
    """
    Run telegrip VR teleop with bobacbot episode recording.

    Parameters
    ----------
    ws_url : str
        ECO65 WebSocket URL, e.g. ws://192.168.1.220:6060
    output_dir : str
        Where to save episode data
    save : bool
        If False, episodes are discarded
    """

    def __init__(
        self,
        ws_url: str = "ws://192.168.1.220:6060",
        output_dir: str = "data/episodes",
        save: bool = True,
        log_level: str = "info",
        image_host: str = "127.0.0.1",
        cam_topics: Optional[dict] = None,
    ):
        self.ws_url = ws_url
        self.output_dir = output_dir
        self.save = save
        self.log_level = log_level
        self.image_host = image_host
        self.cam_topics = cam_topics or {}

        # Setup logging
        level = getattr(logging, log_level.upper())
        logging.basicConfig(
            level=level,
            format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        )

        # Build config
        self.config = self._build_config()

    def _build_config(self) -> TelegripConfig:
        """Build TelegripConfig with ECO65 settings."""
        # Load file defaults, then override
        config_data = get_config_data()
        config = TelegripConfig()

        # ECO65-specific overrides
        config.eco65_ws_url = self.ws_url
        config.enable_robot = True
        config.enable_pybullet = False   # no simulation needed
        config.enable_pybullet_gui = False
        config.enable_vr = True
        config.enable_keyboard = True
        config.autoconnect = True
        config.log_level = self.log_level
        config.send_interval = 0.05

        return config

    def run(self):
        """Blocking run — Ctrl+C to stop."""
        # Image collector (ZMQ SUB for camera frames + timestamps)
        img_collector = None
        if self.save and self.cam_topics:
            img_collector = _ImageCollector(self.image_host, self.cam_topics)
            logger.info(f"[ADAPTER] Image collector configured: {list(self.cam_topics.keys())}")

        # Create and start the patched system (don't create writer here — RecordingControlLoop
        # will create a fresh writer per episode when SPACE is pressed)
        system = RecordingTelegripSystem(self.config, episode_writer=None,
                                         image_collector=img_collector,
                                         output_dir=self.output_dir)
        loop = asyncio.get_event_loop()

        signal_handler = create_signal_handler(system, loop)
        import signal as sig
        sig.signal(sig.SIGINT, signal_handler)
        sig.signal(sig.SIGTERM, signal_handler)

        host = get_local_ip()
        print(f"\n{'='*50}")
        print(f"  VR Teleop + Episode Recording")
        print(f"{'='*50}")
        print(f"  Quest 3: https://{host}:{self.config.https_port}")
        print(f"  ECO65 WS: {self.ws_url}")
        print(f"  Save: {'ON' if self.save else 'OFF'}")
        print(f"{'='*50}\n")

        try:
            loop.run_until_complete(system.start())
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            try:
                loop.run_until_complete(system.stop())
            except Exception:
                pass


# ═══════════════════════════════════════════════════════════
#  CLI
# ═══════════════════════════════════════════════════════════
def main():
    import argparse
    parser = argparse.ArgumentParser(description="telegrip + bobacbot episode recording")
    parser.add_argument("--ws-url", default="ws://192.168.1.220:6060")
    parser.add_argument("--output-dir", default="data/episodes")
    parser.add_argument("--no-save", action="store_true")
    parser.add_argument("--log-level", default="info", choices=["debug", "info", "warning", "error"])
    parser.add_argument("--image-host", default="127.0.0.1",
                        help="Teleimager image server IP (default: 127.0.0.1)")
    parser.add_argument("--hand-cam-port", type=int, default=55555,
                        help="ZMQ port for hand_camera (0 to disable)")
    parser.add_argument("--base-cam-port", type=int, default=55556,
                        help="ZMQ port for base_camera (0 to disable)")
    args = parser.parse_args()

    cam_topics = {}
    if args.hand_cam_port > 0:
        cam_topics["hand_camera"] = args.hand_cam_port
    if args.base_cam_port > 0:
        cam_topics["base_camera"] = args.base_cam_port

    adapter = TelegripAdapter(
        ws_url=args.ws_url,
        output_dir=args.output_dir,
        save=not args.no_save,
        log_level=args.log_level,
        image_host=args.image_host,
        cam_topics=cam_topics,
    )
    adapter.run()


if __name__ == "__main__":
    main()
