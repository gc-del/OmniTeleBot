"""
Main control loop for the teleoperation system.
Consumes control goals from the command queue and executes them via the robot interface.
"""

import asyncio
import numpy as np
import logging
import time
import queue  # Add import for thread-safe queue
from typing import Dict, Optional

import numpy as np
from scipy.spatial.transform import Rotation as R

from .config import TelegripConfig, NUM_JOINTS, WRIST_FLEX_INDEX, WRIST_ROLL_INDEX, GRIPPER_INDEX
from .core.robot_interface import RobotInterface
# PyBulletVisualizer will be imported on demand
from .inputs.base import ControlGoal, ControlMode
# WebKeyboardHandler will be imported on demand to avoid circular imports

logger = logging.getLogger(__name__)

# EMA smoothing: lower = slower, higher = faster response
POSITION_EMA_ALPHA = 0.5
ROTATION_EMA_ALPHA = 0.5
ROTATION_SCALE = 0.5  # 旋转缩放: <1 减速, >1 加速, 0 关掉

class BaseState:
    """State tracking for the mobile base (chassis)."""

    def __init__(self):
        self.active = False
        self.prev_position = None    # previous VR controller position [x, y]

    def reset(self):
        self.active = False
        self.prev_position = None


class ArmState:
    """State tracking for a single robot arm."""

    def __init__(self, arm_name: str):
        self.arm_name = arm_name
        self.mode = ControlMode.IDLE
        self.target_position = None
        self.prev_smooth = None            # EMA state for position smoothing
        self.target_quat = None            # Cartesian target quat (for ECO65)
        self.goal_position = None
        self.origin_quat = None            # ref quat at grip activation (for quat computation)
        self.origin_wrist_roll_angle = 0.0
        self.origin_wrist_flex_angle = 0.0
        self.current_wrist_roll = 0.0
        self.current_wrist_flex = 0.0
        self.prev_wrist_roll_smooth = None
        self.prev_wrist_flex_smooth = None

    def reset(self):
        """Reset arm state to idle."""
        self.mode = ControlMode.IDLE
        self.target_position = None
        self.prev_smooth = None
        self.target_quat = None
        self.goal_position = None
        self.origin_quat = None
        self.origin_wrist_roll_angle = 0.0
        self.origin_wrist_flex_angle = 0.0
        self.current_wrist_roll = 0.0
        self.current_wrist_flex = 0.0
        self.prev_wrist_roll_smooth = None
        self.prev_wrist_flex_smooth = None


class ControlLoop:
    """Main control loop that processes command queue and controls robot."""

    def __init__(self, command_queue: asyncio.Queue, config: TelegripConfig, control_commands_queue: Optional[queue.Queue] = None):
        self.command_queue = command_queue
        self.control_commands_queue = control_commands_queue
        self.config = config

        # Components
        self.robot_interface = None
        self.visualizer = None
        self.web_keyboard_handler = None  # Reference to web-based keyboard handler

        # Base + single arm (ECO65: left controller→base, right controller→arm)
        self.base_state = BaseState()
        self.arm_state = ArmState("right")

        # Control timing
        self.last_log_time = 0
        self.log_interval = 1.0  # Log status every second

        # Debug flags
        self._queue_debug_logged = False
        self._process_debug_logged = False

        self.is_running = False
    
    def setup(self) -> bool:
        """Setup robot interface and PyBullet visualizer."""
        success = True
        setup_errors = []

        # Setup robot interface
        try:
            self.robot_interface = RobotInterface(self.config)
            if not self.robot_interface.connect():
                error_msg = "Robot interface failed to connect"
                logger.error(error_msg)
                setup_errors.append(error_msg)
                if self.config.enable_robot:
                    success = False
        except Exception as e:
            error_msg = f"Robot interface setup failed with exception: {e}"
            logger.error(error_msg)
            setup_errors.append(error_msg)
            if self.config.enable_robot:
                success = False

        # Setup PyBullet simulation, IK and visualizer
        if self.config.enable_pybullet:
            try:
                from .core.visualizer import PyBulletVisualizer
                self.visualizer = PyBulletVisualizer(
                    self.config.get_absolute_urdf_path(),
                    use_gui=self.config.enable_pybullet_gui,
                    log_level=self.config.log_level)
                if not self.visualizer.setup():
                    error_msg = "PyBullet visualizer setup failed"
                    logger.error(error_msg)
                    setup_errors.append(error_msg)
                    self.visualizer = None
                else:
                    joint_limits_min, joint_limits_max = self.visualizer.get_joint_limits
                    self.robot_interface.setup_kinematics(
                        self.visualizer.physics_client,
                        self.visualizer.robot_ids,
                        self.visualizer.joint_indices,
                        self.visualizer.end_effector_link_indices,
                        joint_limits_min,
                        joint_limits_max)
            except Exception as e:
                error_msg = f"PyBullet visualizer setup failed with exception: {e}"
                logger.error(error_msg)
                setup_errors.append(error_msg)
                self.visualizer = None

        if setup_errors:
            logger.error("Setup failed with the following errors:")
            for i, error in enumerate(setup_errors, 1):
                logger.error(f"  {i}. {error}")

        if self.web_keyboard_handler and self.robot_interface:
            self.web_keyboard_handler.set_robot_interface(self.robot_interface)
            logger.info("Set robot interface on web keyboard handler")

        return success
    
    async def start(self):
        """Start the control loop."""
        if not self.setup():
            logger.error("Control loop setup failed")
            return
        
        self.is_running = True
        logger.info("Control loop started")
        
        # Initialize arm states with current robot positions
        self._initialize_arm_states()
        
        # Main control loop
        while self.is_running:
            try:
                # Process command queue
                await self._process_commands()
                
                # Update robot (with error resilience)
                self._update_robot_safely()
                
                # Update visualization
                if self.visualizer:
                    self._update_visualization()
                
                # Periodic logging
                self._periodic_logging()
                
                # Control rate
                await asyncio.sleep(self.config.send_interval)
                
            except Exception as e:
                logger.error(f"Error in control loop: {e}")
                await asyncio.sleep(0.1)
        
        logger.info("Control loop stopped")
    
    async def stop(self):
        """Stop the control loop."""
        self.is_running = False

        # Cleanup - disengage robot first (returns to home and disables torque)
        if self.robot_interface:
            if self.robot_interface.is_engaged:
                logger.info("🛑 Disengaging robot before shutdown...")
                self.robot_interface.disengage()
            self.robot_interface.disconnect()

        if self.visualizer:
            self.visualizer.disconnect()
    
    def _initialize_arm_states(self):
        """Initialize arm + base states with current robot positions."""
        if self.robot_interface:
            pos = self.robot_interface.get_current_end_effector_position("left")
            self.arm_state.target_position = pos.copy()
            self.arm_state.goal_position = pos.copy()
            angles = self.robot_interface.get_arm_angles("left")
            self.arm_state.current_wrist_roll = angles[WRIST_ROLL_INDEX]
            self.arm_state.current_wrist_flex = angles[WRIST_FLEX_INDEX]
            self.arm_state.origin_wrist_roll_angle = angles[WRIST_ROLL_INDEX]
            self.arm_state.origin_wrist_flex_angle = angles[WRIST_FLEX_INDEX]
            quat = self.robot_interface.get_current_end_effector_orientation().as_quat()
            self.arm_state.origin_quat = quat.copy()
            self.arm_state.target_quat = quat.copy()
            self.arm_state.mode = ControlMode.IDLE
            logger.info(f"Initialized arm: pos={pos.round(3)}")
    
    async def _process_commands(self):
        """Process commands from the command queue."""
        try:
            # Process regular control goals
            while not self.command_queue.empty():
                goal = self.command_queue.get_nowait()
                await self._execute_goal(goal)
        except Exception as e:
            logger.error(f"Error processing commands: {e}")
            import traceback
            logger.error(f"Traceback: {traceback.format_exc()}")
    
    async def _handle_command(self, command):
        """Handle individual commands."""
        action = command.get('action', '')
        logger.info(f"🔌 Processing control command: {action}")
        
        if action == 'enable_keyboard':
            if self.web_keyboard_handler:
                await self.web_keyboard_handler.start()
                logger.info("🎮 Keyboard control ENABLED via API")
        elif action == 'disable_keyboard':
            if self.web_keyboard_handler:
                await self.web_keyboard_handler.stop()
                logger.info("🎮 Keyboard control DISABLED via API")
        elif action == 'web_keypress':
            # Handle individual keypress events from web interface
            key = command.get('key')
            event = command.get('event')  # 'press' or 'release'

            if self.web_keyboard_handler and self.web_keyboard_handler.is_enabled:
                logger.debug(f"🌐 Processing web keypress: {key}_{event}")
                if event == 'press':
                    self.web_keyboard_handler.on_key_press(key)
                elif event == 'release':
                    self.web_keyboard_handler.on_key_release(key)
            else:
                logger.warning("🎮 Web keyboard handler not enabled")
        elif action == 'robot_connect':
            logger.info("🔌 Processing robot_connect command")
            if self.robot_interface and self.robot_interface.is_connected:
                logger.info(f"🔌 Robot interface available and connected: {self.robot_interface.is_connected}")
                success = self.robot_interface.engage()
                if success:
                    logger.info("🔌 Robot motors ENGAGED via API")
                    # No need to sync keyboard targets - unified system handles this automatically
                else:
                    logger.error("❌ Failed to engage robot motors")
            else:
                logger.warning(f"Cannot engage robot: interface={self.robot_interface is not None}, connected={self.robot_interface.is_connected if self.robot_interface else False}")
        elif action == 'robot_disconnect':
            logger.info("🔌 Processing robot_disconnect command")
            if self.robot_interface:
                logger.info(f"🔌 Robot interface available")
                success = self.robot_interface.disengage()
                if success:
                    logger.info("🔌 Robot motors DISENGAGED via API")
                    # Reset arm states to IDLE when robot is disengaged
                    self.arm_state.reset()
                    self.base_state.reset()
                    logger.info("🔓 Arm + base: control DEACTIVATED")
                    
                    # Hide visualization markers
                    if self.visualizer:
                        for a in ["left"]:
                            self.visualizer.hide_marker(f"{a}_goal")
                            self.visualizer.hide_frame(f"{a}_goal_frame")
                            self.visualizer.hide_marker(f"{a}_target")
                            self.visualizer.hide_frame(f"{a}_target_frame")
                else:
                    logger.error("❌ Failed to disengage robot motors")
            else:
                logger.warning("Cannot disengage robot: no robot interface")
        else:
            logger.warning(f"Unknown command: {action}")

    async def _execute_goal(self, goal: ControlGoal):
        """Execute a control goal — supports both IK (wrist angles) and
        full-quaternion Cartesian (ECO65)."""
        # Map: left controller → base, right controller → arm
        if goal.arm == "left":
            self._execute_base_goal(goal)
            return
        arm_state = self.arm_state

        # Handle reset signal
        if goal.metadata and goal.metadata.get("reset_target_to_current", False):
            if self.robot_interface and arm_state.mode == ControlMode.POSITION_CONTROL:
                current_position = self.robot_interface.get_current_end_effector_position(goal.arm)
                current_angles = self.robot_interface.get_arm_angles(goal.arm)
                current_quat = self.robot_interface.get_current_end_effector_orientation().as_quat()
                arm_state.target_position = current_position.copy()
                arm_state.prev_smooth = current_position.copy()
                arm_state.goal_position = current_position.copy()
                arm_state.origin_quat = current_quat.copy()
                arm_state.current_wrist_roll = current_angles[WRIST_ROLL_INDEX]
                arm_state.current_wrist_flex = current_angles[WRIST_FLEX_INDEX]
                arm_state.prev_wrist_roll_smooth = current_angles[WRIST_ROLL_INDEX]
                arm_state.prev_wrist_flex_smooth = current_angles[WRIST_FLEX_INDEX]
                arm_state.origin_wrist_roll_angle = current_angles[WRIST_ROLL_INDEX]
                arm_state.origin_wrist_flex_angle = current_angles[WRIST_FLEX_INDEX]
                logger.info(f"Reset {goal.arm} arm to current position")
            return

        # Handle mode changes
        if goal.mode is not None and goal.mode != arm_state.mode:
            if goal.mode == ControlMode.POSITION_CONTROL:
                arm_state.mode = ControlMode.POSITION_CONTROL
                if self.robot_interface:
                    current_position = self.robot_interface.get_current_end_effector_position(goal.arm)
                    if np.allclose(current_position, 0.0, atol=1e-4):
                        logger.warning(f"{goal.arm.upper()}: POSITION is ZERO — "
                                       "ECO65 WS state may be stale. "
                                       "Check robot WS connection.")
                    current_angles = self.robot_interface.get_arm_angles(goal.arm)
                    current_quat = self.robot_interface.get_current_end_effector_orientation().as_quat()
                    arm_state.target_position = current_position.copy()
                    arm_state.prev_smooth = current_position.copy()
                    arm_state.goal_position = current_position.copy()
                    arm_state.origin_quat = current_quat.copy()
                    arm_state.current_wrist_roll = current_angles[WRIST_ROLL_INDEX]
                    arm_state.current_wrist_flex = current_angles[WRIST_FLEX_INDEX]
                    arm_state.prev_wrist_roll_smooth = current_angles[WRIST_ROLL_INDEX]
                    arm_state.prev_wrist_flex_smooth = current_angles[WRIST_FLEX_INDEX]
                    arm_state.origin_wrist_roll_angle = current_angles[WRIST_ROLL_INDEX]
                    arm_state.origin_wrist_flex_angle = current_angles[WRIST_FLEX_INDEX]
                    self.robot_interface.reset_arm_tracking(goal.arm)
                logger.info(f"{goal.arm.upper()}: POSITION_CONTROL activated")
            elif goal.mode == ControlMode.IDLE:
                arm_state.reset()
                # Clear robot_interface targets so stale poses aren't re-sent
                if self.robot_interface:
                    self.robot_interface.left_target_pos = None
                    self.robot_interface.left_target_quat = None
                if self.visualizer:
                    self.visualizer.hide_marker(f"{goal.arm}_goal")
                    self.visualizer.hide_frame(f"{goal.arm}_goal_frame")
                logger.info(f"{goal.arm.upper()}: IDLE")

        # Position: incremental frame-to-frame accumulation, with EMA smoothing
        if goal.target_position is not None and arm_state.mode == ControlMode.POSITION_CONTROL:
            raw_pos = None
            if goal.metadata and goal.metadata.get("relative_position", False):
                # Incremental mode: accumulate frame delta onto current target
                if arm_state.target_position is not None:
                    raw_pos = arm_state.target_position + goal.target_position
                elif self.robot_interface:
                    raw_pos = (self.robot_interface.get_current_end_effector_position(goal.arm)
                               + goal.target_position)
            else:
                raw_pos = goal.target_position.copy()
            if raw_pos is not None:
                if arm_state.prev_smooth is not None:
                    raw_pos = (POSITION_EMA_ALPHA * raw_pos +
                               (1.0 - POSITION_EMA_ALPHA) * arm_state.prev_smooth)
                arm_state.prev_smooth = raw_pos.copy()
                arm_state.target_position = raw_pos
                arm_state.goal_position = raw_pos.copy()

        # Wrist angles: incremental accumulation with EMA smoothing
        if goal.wrist_roll_deg is not None:
            raw_roll = (arm_state.current_wrist_roll + goal.wrist_roll_deg
                        if (goal.metadata and goal.metadata.get("relative_position", False))
                        else goal.wrist_roll_deg)
            if arm_state.prev_wrist_roll_smooth is not None:
                raw_roll = (ROTATION_EMA_ALPHA * raw_roll +
                            (1.0 - ROTATION_EMA_ALPHA) * arm_state.prev_wrist_roll_smooth)
            arm_state.prev_wrist_roll_smooth = raw_roll
            arm_state.current_wrist_roll = raw_roll
        if goal.wrist_flex_deg is not None:
            raw_flex = (arm_state.current_wrist_flex + goal.wrist_flex_deg
                        if (goal.metadata and goal.metadata.get("relative_position", False))
                        else goal.wrist_flex_deg)
            if arm_state.prev_wrist_flex_smooth is not None:
                raw_flex = (ROTATION_EMA_ALPHA * raw_flex +
                            (1.0 - ROTATION_EMA_ALPHA) * arm_state.prev_wrist_flex_smooth)
            arm_state.prev_wrist_flex_smooth = raw_flex
            arm_state.current_wrist_flex = raw_flex

        # Rotation: prefer full-quaternion (3-axis, for ECO65),
        # fall back to wrist roll/flex (2-axis, for SO100).
        if arm_state.mode == ControlMode.POSITION_CONTROL:
            rel_quat = (goal.metadata or {}).get("rel_quat_robot")
            if rel_quat is not None and arm_state.origin_quat is not None:
                # Full 3-axis relative quaternion (rx+ry+rz all preserved)
                rotvec = R.from_quat(rel_quat).as_rotvec()
                rotvec *= ROTATION_SCALE  # apply scale
                delta_rot = R.from_rotvec(rotvec)
                desired_rot = delta_rot * R.from_quat(arm_state.origin_quat)
                arm_state.target_quat = desired_rot.as_quat()
            else:
                # Fallback: 2-axis wrist roll/flex (original telegrip)
                dr = np.radians(arm_state.current_wrist_roll -
                                arm_state.origin_wrist_roll_angle)
                df = np.radians(arm_state.current_wrist_flex -
                                arm_state.origin_wrist_flex_angle)
                delta_rot = R.from_euler('xyz', [dr, df, 0.0])
                if arm_state.origin_quat is not None:
                    desired_rot = delta_rot * R.from_quat(arm_state.origin_quat)
                    arm_state.target_quat = desired_rot.as_quat()

        # Gripper
        if goal.gripper_closed is not None and self.robot_interface:
            self.robot_interface.set_gripper(goal.arm, goal.gripper_closed)

    def _execute_base_goal(self, goal: ControlGoal):
        """Handle left-controller goals → base (chassis) velocity control."""
        meta = goal.metadata or {}

        # Thumbstick → direct velocity [vx, vy, v_wz]
        if meta.get("thumbstick_direct") and goal.target_position is not None:
            vx = float(goal.target_position[0])
            vy = float(goal.target_position[1])
            v_wz = float(goal.target_position[2]) if len(goal.target_position) >= 3 else 0.0
            self.robot_interface.set_base_velocity(vx, vy, v_wz)
            self.base_state.active = True
            return

        # Trigger + position delta → velocity
        trig = meta.get("trigger", 0)
        base_active = trig > 0.5

        if base_active and goal.target_position is not None:
            bp = goal.target_position[:2]  # VR controller XY
            if not self.base_state.active:
                self.base_state.prev_position = bp.copy()
                self.base_state.active = True
                return  # first frame, no velocity yet
            vx = (bp[0] - self.base_state.prev_position[0]) / self.config.send_interval
            vy = (bp[1] - self.base_state.prev_position[1]) / self.config.send_interval
            self.base_state.prev_position = bp.copy()
            self.robot_interface.set_base_velocity(vx, vy)
        else:
            self.robot_interface.set_base_velocity(0.0, 0.0)
            self.base_state.active = False

    def _update_robot_safely(self):
        """Update robot with current control goals (with error handling)."""
        if not self.robot_interface:
            return
        
        try:
            self._update_robot()
        except Exception as e:
            logger.error(f"Error updating robot: {e}")
            # Don't shutdown, just continue - robot interface will handle connection issues
    
    def _update_robot(self):
        """Update robot: solve IK for PyBullet, send Cartesian to ECO65."""
        if not self.robot_interface:
            return

        # Single arm (right controller → ECO65)
        arm = "left"  # ECO65 uses single-arm; robot_interface uses "left" internally
        arm_state = self.arm_state
        if (arm_state.mode == ControlMode.POSITION_CONTROL and
            arm_state.target_position is not None):
            # IK solve (for PyBullet viz / joint angle tracking)
            ik_solution = self.robot_interface.solve_ik(arm, arm_state.target_position)
            current_gripper = self.robot_interface.get_arm_angles(arm)[GRIPPER_INDEX]
            self.robot_interface.update_arm_angles(
                arm, ik_solution,
                arm_state.current_wrist_flex,
                arm_state.current_wrist_roll,
                current_gripper)
            # Cartesian target (for ECO65 WebSocket)
            quat = (arm_state.target_quat if arm_state.target_quat is not None
                    else np.array([0., 0., 0., 1.]))
            self.robot_interface.set_arm_target(arm,
                arm_state.target_position, quat)

        # Send arm + base to ECO65 over WebSocket
        if self.robot_interface.is_connected and self.robot_interface.is_engaged:
            self.robot_interface.send_command()
    
    def _update_visualization(self):
        """Update PyBullet visualization."""
        if not self.visualizer:
            return
        left_angles = self.robot_interface.get_actual_arm_angles("left")
        right_angles = self.robot_interface.get_actual_arm_angles("right")
        self.visualizer.update_robot_pose(left_angles, 'left')
        self.visualizer.update_robot_pose(right_angles, 'right')
        arm = "left"  # single arm for ECO65
        arm_state = self.arm_state
        if arm_state.mode == ControlMode.POSITION_CONTROL:
            if arm_state.target_position is not None:
                current_pos = self.robot_interface.get_current_end_effector_position(arm)
                self.visualizer.update_marker_position(f"{arm}_target", current_pos)
                self.visualizer.update_coordinate_frame(f"{arm}_target_frame", current_pos)
            if arm_state.goal_position is not None:
                self.visualizer.update_marker_position(f"{arm}_goal", arm_state.goal_position)
                self.visualizer.update_coordinate_frame(f"{arm}_goal_frame", arm_state.goal_position)
        else:
            self.visualizer.hide_marker(f"{arm}_target")
            self.visualizer.hide_marker(f"{arm}_goal")
            self.visualizer.hide_frame(f"{arm}_target_frame")
            self.visualizer.hide_frame(f"{arm}_goal_frame")
        self.visualizer.step_simulation()
    
    def _periodic_logging(self):
        """Log status information periodically."""
        current_time = time.time()
        if current_time - self.last_log_time >= self.log_interval:
            self.last_log_time = current_time
            if (self.arm_state.mode == ControlMode.POSITION_CONTROL and
                self.robot_interface):
                angles = self.robot_interface.get_arm_angles("left")
                pos = self.robot_interface.get_current_end_effector_position("left")
                vx, vy = self.robot_interface.base_vx, self.robot_interface.base_vy
                logger.info(f"Arm active | angles: {angles.round(1)} | pos: {pos.round(3)}"
                            f" | base: ({vx:.2f},{vy:.2f})")
    
    @property
    def status(self) -> Dict:
        """Get current control loop status."""
        return {
            "running": self.is_running,
            "arm_mode": self.arm_state.mode.value,
            "base_active": self.base_state.active,
            "robot_connected": self.robot_interface.is_connected if self.robot_interface else False,
            "left_arm_connected": self.robot_interface.get_arm_connection_status("left") if self.robot_interface else False,
            "visualizer_connected": self.visualizer.is_connected if self.visualizer else False,
        }
