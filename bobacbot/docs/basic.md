# Mobile Base + Single Arm Control Interface

交付接口规范

- 机器人形态: 移动底盘 + 单 6DoF 机械臂 + 单夹爪
- 通讯方式: `WebSocket`
- 地址: `ws://192.168.1.127:6060`
- 编码: `UTF-8 JSON`
- 时间戳: `Unix timestamp`，单位 `second`

## 1. Scope

接口包含三类消息:

1. `robot_state`: 底层主动上报机器人状态
2. `robot_command`: 上层下发控制命令
3. `command_ack` / `command_status`: 底层对命令接收与执行的反馈

## 2. Conventions

- 所有位姿、速度命令都必须带 `frame_id`
- 平移单位为 `meter`
- 角度单位为 `radian`
- 线速度单位为 `m/s`
- 角速度单位为 `rad/s`
- 夹爪开合范围为 `0~100`

推荐坐标系:

- `odom`: 底盘里程计坐标系
- `base_link`: 底盘本体坐标系
- `arm_base`: 机械臂基座坐标系
- `ee_link`: 末端执行器坐标系

## 3. robot_state

底层主动上报状态，建议频率 `20~50 Hz`。

必需字段:

- `type`
- `schema_version`
- `timestamp`
- `base.pose`
- `base.twist`
- `arm.joint_position`
- `arm.ee_pose`
- `gripper.position_pct`
- `motion.active_command_id`
- `motion.status`
- `safety.estop`
- `safety.fault_code`

示例:

```json
{
  "type": "robot_state",
  "schema_version": "v2.0.1",
  "timestamp": 1764560171.721209,
  "base": {
    "pose": {
      "frame_id": "odom",
      "position": {"x": 0.0, "y": 0.0, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    "twist": {
      "frame_id": "base_link",
      "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
      "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
    }
  },
  "arm": {
    "joint_position": [-0.02, 1.19, -1.29, 0.89, 1.56, -0.01],
    "ee_pose": {
      "frame_id": "arm_base",
      "position": {"x": 0.153, "y": -0.062, "z": 0.630},
      "orientation": {"x": -0.272, "y": -0.266, "z": 0.645, "w": 0.663}
    }
  },
  "gripper": {
    "position_pct": 99
  },
  "motion": {
    "active_command_id": "",
    "status": "idle"
  },
  "safety": {
    "estop": false,
    "fault_code": 0,
    "fault_message": ""
  }
}
```

## 4. robot_command

每条命令必须包含:

- `type = robot_command`
- `schema_version = v2.0.1`
- `command_id`
- `timestamp`
- `timeout_ms`
- `preempt_policy`
- `control_mode`

推荐:

- `preempt_policy`: `replace | reject | queue`
- 默认使用 `replace`

`control_mode` 支持:

- `base_only`
- `arm_only`
- `gripper_only`
- `whole_body`
- `stop`
- `hold`

### 4.1 Base Command

底盘建议支持以下两种模式:

1. `twist`: 速度控制
2. `relative_pose`: 相对当前状态的位姿偏移

`twist` 示例:

```json
{
  "type": "robot_command",
  "schema_version": "v2.0.1",
  "command_id": "cmd_base_0001",
  "timestamp": 1764560172.1023,
  "timeout_ms": 300,
  "preempt_policy": "replace",
  "control_mode": "base_only",
  "base_cmd": {
    "mode": "twist",
    "frame_id": "base_link",
    "linear": {"x": 0.2, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.3}
  }
}
```

`relative_pose` 示例:

```json
{
  "type": "robot_command",
  "schema_version": "v2.0.1",
  "command_id": "cmd_base_0002",
  "timestamp": 1764560172.1023,
  "timeout_ms": 3000,
  "preempt_policy": "replace",
  "control_mode": "base_only",
  "base_cmd": {
    "mode": "relative_pose",
    "frame_id": "base_link",
    "position": {"x": 0.5, "y": 0.0, "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.1494, "w": 0.9888}
  }
}
```

### 4.2 Arm Command

机械臂建议支持以下模式:

1. `joint_position`
2. `ee_pose`
3. `ee_relative_pose`



`joint_position` 示例:

```json
{
  "type": "robot_command",
  "schema_version": "v2.0.1",
  "command_id": "cmd_arm_joint_0001",
  "timestamp": 1764560172.1023,
  "timeout_ms": 500,
  "preempt_policy": "replace",
  "control_mode": "arm_only",
  "arm_cmd": {
    "mode": "joint_position",
    "joint_position": [-0.01, 1.15, -1.20, 0.90, 1.50, 0.02]
  }
}
```

`ee_pose` 示例:

```json
{
  "type": "robot_command",
  "schema_version": "v2.0.1",
  "command_id": "cmd_arm_0001",
  "timestamp": 1764560172.1023,
  "timeout_ms": 3000,
  "preempt_policy": "replace",
  "control_mode": "arm_only",
  "arm_cmd": {
    "mode": "ee_pose",
    "frame_id": "arm_base",
    "position": {"x": 0.2, "y": -0.1, "z": 0.5},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
  }
}
```

`ee_relative_pose` 示例:

```json
{
  "type": "robot_command",
  "schema_version": "v2.0.1",
  "command_id": "cmd_arm_0002",
  "timestamp": 1764560172.1023,
  "timeout_ms": 500,
  "preempt_policy": "replace",
  "control_mode": "arm_only",
  "arm_cmd": {
    "mode": "ee_relative_pose",
    "frame_id": "ee_link",
    "position": {"x": 0.02, "y": 0.0, "z": -0.01},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
  }
}
```

### 4.3 Gripper Command

```json
{
  "gripper_cmd": {
    "mode": "position",
    "position_pct": 80
  }
}
```

### 4.4 Whole-Body Command

一条命令允许同时控制底盘、机械臂和夹爪。

```json
{
  "type": "robot_command",
  "schema_version": "v2.0.1",
  "command_id": "cmd_whole_body_0001",
  "timestamp": 1764560172.1023,
  "timeout_ms": 500,
  "preempt_policy": "replace",
  "control_mode": "whole_body",
  "base_cmd": {
    "mode": "twist",
    "frame_id": "base_link",
    "linear": {"x": 0.05, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
  },
  "arm_cmd": {
    "mode": "ee_relative_pose",
    "frame_id": "arm_base",
    "position": {"x": 0.01, "y": 0.0, "z": -0.01},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
  },
  "gripper_cmd": {
    "mode": "position",
    "position_pct": 60
  }
}
```

## 5. command_ack

底层收到命令后，应尽快返回 `command_ack`。

```json
{
  "type": "command_ack",
  "schema_version": "v2.0.1",
  "timestamp": 1764560172.1100,
  "command_id": "cmd_whole_body_0001",
  "accepted": true,
  "reason": ""
}
```

## 6. command_status

命令执行过程中，应异步上报 `command_status`。

推荐状态:

- `received`
- `running`
- `succeeded`
- `failed`
- `rejected`
- `canceled`
- `timeout`

示例:

```json
{
  "type": "command_status",
  "schema_version": "v2.0.1",
  "timestamp": 1764560172.4200,
  "command_id": "cmd_whole_body_0001",
  "status": "running",
  "progress": 0.4,
  "reason": ""
}
```

## 7. Runtime Requirements

- `command_id` 必须唯一，用于闭环追踪
- 同一个 `command_id` 重复到达时，不应重复执行
- `twist` 和 `ee_relative_pose` 超时未续发时，应自动停止
- `ee_pose`、`joint_position`、`relative_pose` 超时后，应返回 `timeout`
- `stop` 优先级最高，要求立即停止相关执行器
- 新命令默认可抢占旧命令，推荐使用 `replace`

## 8. Legacy Compatibility

若需兼容现有初版消息:

- `teleop_mode = base` 映射到 `control_mode = base_only`
- 原底盘 `position + orientation` 映射到 `base_cmd.mode = relative_pose`
- `teleop_mode = arm` 映射到 `control_mode = arm_only`
- 原手臂 `position + orientation` 映射到 `arm_cmd.mode = ee_pose`
- 原 `gripper` 数值映射到 `gripper_cmd.position_pct`


