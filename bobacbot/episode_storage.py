# Author: Jimmy Wu
# Date: October 2024

import pickle
import json
import logging
import threading
import time
from datetime import datetime
from pathlib import Path
import cv2 as cv
import numpy as np
from constants import POLICY_CONTROL_FREQ

logger = logging.getLogger("episode_storage")

def write_frames_to_mp4(frames, mp4_path, fps: float = POLICY_CONTROL_FREQ):
    height, width, _ = frames[0].shape
    
    # 尝试多种编码器，提高兼容性
    codecs_to_try = [
        ('mp4v', 'MPEG-4'),  # 放在首位，避免 avc1 的错误日志
        ('XVID', 'Xvid MPEG-4'),
        ('H264', 'H.264 (alternative)'),
        ('avc1', 'H.264/AVC (MPEG-4)'),
    ]
    
    out = None
    original_error_mode = cv.Error
    try:
        # 临时抑制 OpenCV 错误日志
        def suppress_errors(status, func_name, err_msg, file_name, line):
            pass  # 忽略所有错误
        
        cv.Error = suppress_errors
        
        for fourcc_code, description in codecs_to_try:
            try:
                fourcc = cv.VideoWriter_fourcc(*fourcc_code)
                out = cv.VideoWriter(str(mp4_path), fourcc, fps, (width, height))
                
                # 检查 VideoWriter 是否成功初始化
                if out.isOpened():
                    print(f'Using video codec: {fourcc_code} ({description})')
                    break
                else:
                    if out is not None:
                        out.release()
                    out = None
            except Exception:
                if out is not None:
                    out.release()
                    out = None
                continue  # 继续尝试下一个编码器
    
    finally:
        # 恢复原始错误处理
        cv.Error = original_error_mode
    
    if out is None or not out.isOpened():
        raise RuntimeError(
            'Failed to initialize VideoWriter with any codec. '
            'Please install ffmpeg and libx264: sudo apt install ffmpeg libx264-dev'
        )
    
    try:
        for frame in frames:
            bgr_frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)
            out.write(bgr_frame)
    finally:
        out.release()

def read_frames_from_mp4(mp4_path):
    cap = cv.VideoCapture(str(mp4_path))
    frames = []
    while True:
        ret, bgr_frame = cap.read()
        if not ret:
            break
        frames.append(cv.cvtColor(bgr_frame, cv.COLOR_BGR2RGB))
    cap.release()
    return frames

class EpisodeWriter:
    def __init__(self, output_dir, record_fps: float = POLICY_CONTROL_FREQ):
        self.output_dir = Path(output_dir)
        self.episode_dir = self.output_dir / datetime.now().strftime('%Y%m%dT%H%M%S%f')
        assert not self.episode_dir.exists()
        self.record_fps = record_fps  # for MP4 encoding

        # Episode data
        self.timestamps = []
        self.observations = []
        self.actions = []
        # Per-frame image timestamps: {cam_name: [ts1, ts2, ...]}
        self.image_timestamps: dict[str, list[float]] = {}

        # Write to disk in separate thread to avoid blocking main thread
        self.flush_thread = None

    def step(self, obs, action):
        # if len(self.observations) == 0 and not np.allclose(obs['base_pose'], 0.0, atol=0.01):
        #     raise Exception('Initial base pose should be zero. Did the base get pushed?')
        self.timestamps.append(time.time())
        self.observations.append(obs)
        self.actions.append(action)

    def set_image_timestamps(self, cam_name: str, timestamps: list):
        """Set per-frame image capture timestamps for a camera (seconds since epoch)."""
        self.image_timestamps[cam_name] = timestamps

    def __len__(self):
        return len(self.observations)

    def _flush(self):
        assert len(self) > 0

        # Create episode dir
        self.episode_dir.mkdir(parents=True)

        # Separate regular images (ndarray), JPEG bytes, and timestamps from obs dicts
        frames_dict = {}      # {cam_name: [ndarray, ...]} → MP4
        jpg_dict = {}         # {cam_name: [bytes, ...]} → .jpg files
        ts_dict = {}          # {cam_name: [float, ...]} → image_timestamps.json

        for obs in self.observations:
            keys_to_strip = []
            for k, v in list(obs.items()):
                if isinstance(v, np.ndarray) and v.ndim == 3:
                    # RGB/RGBA image → MP4
                    if k not in frames_dict:
                        frames_dict[k] = []
                    frames_dict[k].append(v)
                    keys_to_strip.append(k)
                elif k.endswith('_jpg') and isinstance(v, bytes):
                    # JPEG bytes → strip from pickle, save separately
                    cam_name = k[:-4]  # e.g. "hand_camera_jpg" → "hand_camera"
                    if cam_name not in jpg_dict:
                        jpg_dict[cam_name] = []
                    jpg_dict[cam_name].append(v)
                    keys_to_strip.append(k)
                elif k.endswith('_ts') and isinstance(v, (int, float)):
                    # Image timestamp → strip, aggregate
                    cam_name = k[:-3]  # e.g. "hand_camera_ts" → "hand_camera"
                    if cam_name not in ts_dict:
                        ts_dict[cam_name] = []
                    ts_dict[cam_name].append(v)
                    keys_to_strip.append(k)
            for k in keys_to_strip:
                obs[k] = None

        # Write RGB images as MP4 videos
        for k, frames in frames_dict.items():
            mp4_path = self.episode_dir / f'{k}.mp4'
            write_frames_to_mp4(frames, mp4_path, fps=self.record_fps)

        # Write JPEG bytes as MP4 video (decode → BGR → write)
        for cam_name, jpg_frames in jpg_dict.items():
            # Decode JPEG bytes to BGR numpy arrays
            bgr_frames = []
            for jpg_bytes in jpg_frames:
                np_arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
                bgr = cv.imdecode(np_arr, cv.IMREAD_COLOR)
                if bgr is not None:
                    bgr_frames.append(bgr)
            if bgr_frames:
                mp4_path = self.episode_dir / f'{cam_name}.mp4'
                write_frames_to_mp4(bgr_frames, mp4_path, fps=self.record_fps)
                logger.info(f"Saved {len(bgr_frames)} frames to {mp4_path}")

        # Build image_timestamps dict from ts_dict (per-camera timestamp arrays)
        for cam_name, ts_list in ts_dict.items():
            self.image_timestamps[cam_name] = ts_list

        # Write rest of episode data
        with open(self.episode_dir / 'data.pkl', 'wb') as f:  # Note: Not secure. Only unpickle data you trust.
            pickle.dump({'timestamps': self.timestamps, 'observations': self.observations, 'actions': self.actions}, f)

        # Write image timestamps for cross-modal data alignment
        if self.image_timestamps:
            img_ts_path = self.episode_dir / 'image_timestamps.json'
            with open(img_ts_path, 'w') as f:
                json.dump(self.image_timestamps, f, indent=2)
            logger.info(f"Image timestamps saved: {img_ts_path}")

        num_episodes = len([child for child in self.output_dir.iterdir() if child.is_dir()])
        print(f'Saved episode to {self.episode_dir} ({num_episodes} total)')

    def flush_async(self):
        print('Saving successful episode to disk...')
        # Note: Disk writes may cause latency spikes in low-level controllers
        self.flush_thread = threading.Thread(target=self._flush, daemon=True)
        self.flush_thread.start()

    def wait_for_flush(self):
        if self.flush_thread is not None:
            self.flush_thread.join()
            self.flush_thread = None

class EpisodeReader:
    def __init__(self, episode_dir):
        self.episode_dir = episode_dir

        # Load data
        with open(episode_dir / 'data.pkl', 'rb') as f:  # Note: Not secure. Only unpickle data you trust.
            data = pickle.load(f)
        self.timestamps = data['timestamps']
        self.observations = data['observations']
        self.actions = data['actions']
        assert len(self.timestamps) > 0
        assert len(self.timestamps) == len(self.observations) == len(self.actions)

        # Restore image observations from MP4 videos
        frames_dict = {}
        for step_idx, obs in enumerate(self.observations):
            for k, v in obs.items():
                if v is None:  # Images are stored as MP4 videos
                    # Load images from MP4 file
                    if k not in frames_dict:
                        mp4_path = episode_dir / f'{k}.mp4'
                        frames_dict[k] = read_frames_from_mp4(mp4_path)

                    # Restore image for current step
                    obs[k] = frames_dict[k][step_idx]  # np.uint8

    def __len__(self):
        return len(self.observations)
