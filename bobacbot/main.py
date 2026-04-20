# Author: Jimmy Wu
# Date: October 2024

import argparse
import time
from itertools import count
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

            print('Teleop is now active. Press "Reset env" in the web app when ready to proceed.')

        # Ready for env reset
        elif action == 'reset_env':
            break

    if writer is not None:
        # Wait for writer to finish saving to disk
        writer.wait_for_flush()

def main(args):
    # Create env
    # For joycon-real mode, always use simulation environment for visualization
    # since the actual control commands are sent via WebSocket from the policy
    if args.sim or args.joycon_real:
        from mujoco_env import MujocoEnv
        if args.teleop:
            env = MujocoEnv(show_images=args.show_images, randomize_robot_position=args.random)
        elif args.joycon:
            env = MujocoEnv(show_images=args.show_images, randomize_robot_position=args.random)
        elif args.joycon_real:
            env = MujocoEnv(show_images=args.show_images, randomize_robot_position=args.random)
        else:
            env = MujocoEnv(show_images=args.show_images, randomize_robot_position=args.random)
    else:
        from real_env import RealEnv
        env = RealEnv()

    # Create policy
    if args.teleop:
        policy = TeleopPolicy()
    elif args.joycon:
        from joycon_policy import JoyconRemotePolicy
        policy = JoyconRemotePolicy()
    elif args.joycon_real:
        from joycon_real_policy import JoyconRealPolicy
        if args.initial_arm_pos is None or args.initial_arm_quat is None:
            raise ValueError('--initial-arm-pos and --initial-arm-quat are REQUIRED for --joycon-real mode')
        policy = JoyconRealPolicy(websocket_url=args.websocket_url,
                                  custom_initial_arm_pos=args.initial_arm_pos,
                                  custom_initial_arm_quat=args.initial_arm_quat)
    elif args.keyboard:
        from keyboard_policy import KeyboardRemotePolicy
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
    parser.add_argument('--save', action='store_true', help='保存演示数据')
    parser.add_argument('--random', action='store_true', help='随机化机器人初始位置 (默认使用XML中的固定位置)')
    parser.add_argument('--output-dir', default='data/0714', help='演示数据输出目录')
    parser.add_argument('--show-images', action='store_true', help='是否显示仿真图像')
    parser.add_argument('--websocket-url', type=str, default='ws://localhost:8765',
                        help='WebSocket URL for real robot control (default: ws://localhost:8765)')
    parser.add_argument('--initial-arm-pos', type=float, nargs=3, required=False,
                        help='Initial arm position [x y z] for real robot (required for --joycon-real)')
    parser.add_argument('--initial-arm-quat', type=float, nargs=4, required=False,
                        help='Initial arm quaternion [x y z w] for real robot (required for --joycon-real)')
    main(parser.parse_args())