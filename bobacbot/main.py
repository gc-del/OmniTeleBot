# Author: Jimmy Wu
# Date: October 2024

import argparse
import time
from itertools import count
import cv2 as cv
from constants import POLICY_CONTROL_PERIOD
from episode_storage import EpisodeWriter
from policies import TeleopPolicy, RemotePolicy

def should_save_episode(writer):
    if len(writer) == 0:
        print('Discarding empty episode')
        return False

    # Prompt user whether to save episode
    while True:
        user_input = input('Save episode (y/n)? ').strip().lower()
        if user_input == 'y':
            return True
        if user_input == 'n':
            print('Discarding episode')
            return False
        print('Invalid response')

def run_episode(env, policy, writer=None):
    # Reset the env
    print('Resetting env...')
    env.reset()
    print('Env has been reset')

    # Wait for user to start the episode with the active control mode
    if policy.__class__.__name__ == 'KeyboardRemotePolicy':
        print('Press SPACE to start a keyboard episode')
    elif policy.__class__.__name__ == 'JoyconRemotePolicy':
        print('Press SPACE to start a Joy-Con episode')
    elif policy.__class__.__name__ == 'JoyconRealPolicy':
        print('Press Y button on right Joy-Con to start/stop episodes')
    else:
        print('Press "Start episode" in the web app when ready to start new episode')
    policy.reset()
    print('Finished reset, Starting new episode')

    episode_ended = False
    start_time = time.time()
    control_period = getattr(policy, 'control_period', POLICY_CONTROL_PERIOD)
    for step_idx in count():
        # Enforce desired control freq
        step_end_time = start_time + step_idx * control_period
        while time.time() < step_end_time:
            time.sleep(0.0001)

        # Get latest observation
        obs = env.get_obs()

        # Get action
        action = policy.step(obs)

        # No action if teleop not enabled
        if action is None:
            continue

        # Execute valid action on robot
        if isinstance(action, dict):
            env.step(action)

            if writer is not None and not episode_ended:
                # Record executed action
                writer.step(obs, action)

        # Episode ended
        elif not episode_ended and action == 'end_episode':
            episode_ended = True
            print('Episode ended')

            if writer is not None and should_save_episode(writer):
                # Save to disk in background thread
                writer.flush_async()

            if policy.__class__.__name__ == 'KeyboardRemotePolicy':
                print('Press SPACE to reset environment, then SPACE again to start next episode.')
            else:
                print('Teleop is now active. Press "Reset env" in the web app when ready to proceed.')

        # Ready for env reset
        elif action == 'reset_env':
            break

    if writer is not None:
        # Wait for writer to finish saving to disk
        writer.wait_for_flush()

def _run_telegrip_vr(args):
    """Launch telegrip-based VR teleop with episode recording."""
    from pathlib import Path
    import sys as _sys

    # Ensure telegrip parent dirs are on sys.path for imports
    _pkg_root = Path(__file__).resolve().parent
    if str(_pkg_root) not in _sys.path:
        _sys.path.insert(0, str(_pkg_root))
    _telegrip_parent = Path(__file__).resolve().parent / "telegrip"
    if str(_telegrip_parent) not in _sys.path:
        _sys.path.insert(0, str(_telegrip_parent))

    # telegrip_adapter.py lives in the outer telegrip/ directory,
    # NOT inside telegrip/telegrip/.  Import it directly.
    import importlib.util
    _adapter_path = _telegrip_parent / "telegrip_adapter.py"
    _spec = importlib.util.spec_from_file_location("telegrip_adapter", _adapter_path)
    _adapter_mod = importlib.util.module_from_spec(_spec)
    _sys.modules["telegrip_adapter"] = _adapter_mod
    _spec.loader.exec_module(_adapter_mod)
    TelegripAdapter = _adapter_mod.TelegripAdapter

    # Build camera topics for image recording
    cam_topics = {}
    if getattr(args, 'hand_cam_port', 0) > 0:
        cam_topics["hand_camera"] = args.hand_cam_port
    if getattr(args, 'base_cam_port', 0) > 0:
        cam_topics["base_camera"] = args.base_cam_port

    adapter = TelegripAdapter(
        ws_url=args.websocket_url or "ws://192.168.1.220:6060",
        output_dir=args.output_dir,
        save=args.save,
        log_level="info",
        image_host=getattr(args, 'image_host', '127.0.0.1'),
        cam_topics=cam_topics if cam_topics else None,
    )
    adapter.run()


def main(args):
    # ── telegrip-based VR real mode ──────────────────────
    if args.vr_real and not args.sim:
        _run_telegrip_vr(args)
        return

    # ── legacy modes ─────────────────────────────────────
    # Create env
    # For joycon-real mode, always use simulation environment for visualization
    # since the actual control commands are sent via WebSocket from the policy
    if args.sim or args.joycon_real or args.vr_real or args.xl_vr:
        from sim.mujoco_env import MujocoEnv
        mjcf_path = args.scene if args.scene else None
        robot_start_pos = list(args.robot_pos) if args.robot_pos else None
        if robot_start_pos:
            robot_start_pos.append(args.robot_yaw)
        if args.xl_vr:
            env = MujocoEnv(render_images=False, show_images=args.show_images, randomize_robot_position=args.random, mjcf_path=mjcf_path, robot_start_pos=robot_start_pos)
        elif args.vr_real:
            env = MujocoEnv(render_images=False, show_images=args.show_images, randomize_robot_position=args.random, mjcf_path=mjcf_path, robot_start_pos=robot_start_pos)
        elif args.teleop:
            env = MujocoEnv(show_images=args.show_images, randomize_robot_position=args.random, mjcf_path=mjcf_path, robot_start_pos=robot_start_pos)
        elif args.joycon:
            env = MujocoEnv(show_images=args.show_images, randomize_robot_position=args.random, mjcf_path=mjcf_path, robot_start_pos=robot_start_pos)
        elif args.joycon_real:
            env = MujocoEnv(show_images=args.show_images, randomize_robot_position=args.random, mjcf_path=mjcf_path, robot_start_pos=robot_start_pos)
        else:
            env = MujocoEnv(show_images=args.show_images, randomize_robot_position=args.random, mjcf_path=mjcf_path, robot_start_pos=robot_start_pos)
    else:
        from kinova.real_env import RealEnv
        env = RealEnv()

    # Create policy
    if args.xl_vr:
        from xl_vr_policy import XLVRPolicy
        if not args.websocket_url:
            raise ValueError('--websocket-url is REQUIRED for --xl-vr mode')
        policy = XLVRPolicy(args.websocket_url)
    elif args.vr_real:
        from vr_real_policy import VRRealPolicy
        if not args.websocket_url:
            raise ValueError('--websocket-url is REQUIRED for --vr-real mode')
        policy = VRRealPolicy(args.websocket_url)
    elif args.teleop:
        policy = TeleopPolicy()
    elif args.joycon:
        from joycon.joycon_policy import JoyconRemotePolicy
        policy = JoyconRemotePolicy()
    elif args.joycon_real:
        from joycon.joycon_real_policy import JoyconRealPolicy
        if args.initial_arm_pos is None or args.initial_arm_quat is None:
            raise ValueError('--initial-arm-pos and --initial-arm-quat are REQUIRED for --joycon-real mode')
        policy = JoyconRealPolicy(websocket_url=args.websocket_url or 'ws://localhost:8765',
                                  custom_initial_arm_pos=args.initial_arm_pos,
                                  custom_initial_arm_quat=args.initial_arm_quat)
    elif args.keyboard:
        from keyboard.keyboard_policy import KeyboardRemotePolicy
        policy = KeyboardRemotePolicy()
    else:
        policy = RemotePolicy()

    try:
        while True:
            writer = EpisodeWriter(args.output_dir) if args.save else None
            run_episode(env, policy, writer)
    finally:
        env.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', action='store_true', help='使用仿真环境')
    parser.add_argument('--teleop', action='store_true', help='启用遥操作模式')
    parser.add_argument('--keyboard', action='store_true', help='启用键盘控制模式（按空格开始/结束 episode，无需手机）')
    parser.add_argument('--joycon', action='store_true', help='启用 Joy-Con 控制模式（左手柄控制底盘，右手柄控制机械臂）')
    parser.add_argument('--joycon-real', action='store_true', help='Use Joy-Con to control real robot via WebSocket')
    parser.add_argument('--vr-real', action='store_true', help='Use VR (Quest 3) to control real robot via WebSocket')
    parser.add_argument('--xl-vr', action='store_true', help='Use XL VR (A-Frame WebXR) to control real robot via WebSocket')
    parser.add_argument('--save', action='store_true', help='保存演示数据')
    parser.add_argument('--random', action='store_true', help='随机化机器人初始位置 (默认使用XML中的固定位置)')
    parser.add_argument('--scene', type=str, default=None,
                        help='自定义场景XML文件路径 (例如: models/bobacbot_competition_scene.xml)')
    parser.add_argument('--robot-pos', type=float, nargs=2, default=None,
                        help='机器人起始位置 [x y] (例如: --robot-pos 1.0 0.0)')
    parser.add_argument('--robot-yaw', type=float, default=0.0,
                        help='机器人起始朝向 (弧度, 逆时针为正)')
    parser.add_argument('--output-dir', default='data/0714', help='演示数据输出目录')
    parser.add_argument('--show-images', action='store_true', help='是否显示仿真图像')
    parser.add_argument('--websocket-url', type=str, default=None,
                        help='WebSocket URL for real robot control (e.g. ws://192.168.1.220:6060)')
    parser.add_argument('--initial-arm-pos', type=float, nargs=3, required=False,
                        help='Initial arm position [x y z] for real robot (required for --joycon-real)')
    parser.add_argument('--initial-arm-quat', type=float, nargs=4, required=False,
                        help='Initial arm quaternion [x y z w] for real robot (required for --joycon-real)')
    parser.add_argument('--image-host', type=str, default='127.0.0.1',
                        help='Teleimager image server IP (used with --vr-real --save)')
    parser.add_argument('--hand-cam-port', type=int, default=0,
                        help='ZMQ port for hand_camera, 0 to disable (used with --vr-real --save)')
    parser.add_argument('--base-cam-port', type=int, default=0,
                        help='ZMQ port for base_camera, 0 to disable (used with --vr-real --save)')
    main(parser.parse_args())
