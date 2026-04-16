# JoyconRobotics Interface Documentation

## 概述

`joyconrobotics.py` 模块提供了一个高级 Python 接口，用于将 Nintendo Joy-Con 控制器用于机器人控制。该模块基于 `joycon.py` 构建，添加了姿态估计、位置控制和按钮映射功能，专为机器人遥操作设计。

## 依赖

- `hid` 或 `hidapi` 库，用于 HID 设备通信
- `glm` 库，用于向量和四元数运算
- `scipy` 库，用于旋转计算
- `numpy` 库，用于数值计算
- Python 3.x

## 安装

确保安装了必要的依赖：

```bash
pip install hidapi glm scipy numpy
```

## 使用示例

### 基本使用

```python
from joyconrobotics.joyconrobotics import JoyconRobotics

# 创建 JoyconRobotics 实例（右 Joy-Con）
joycon_robot = JoyconRobotics(device="right")

# 获取控制数据
posture, gripper_state, button_control = joycon_robot.get_control()

# posture: [x, y, z, roll, pitch, yaw]
# gripper_state: 夹爪状态 (0.0 关闭, 1.0 打开)
# button_control: 按钮控制信号
```

### 自定义配置

```python
joycon_robot = JoyconRobotics(
    device="right",
    gripper_open=1.0,
    gripper_close=0.0,
    limit_dof=True,
    glimit=[[0.0, -0.4, 0.0, -1.57, -1.5, -1.57],
            [0.4, 0.4, 0.3, 1.57, 1.5, 1.57]],
    offset_position_m=[0.2, 0.0, 0.1],
    dof_speed=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
)
```

## 按钮映射

### 摇杆控制

- **垂直摇杆 (Vertical Stick)**:
  - 上推 (>4000): 前进 (沿当前方向向量移动)
  - 下推 (<1000): 后退

- **水平摇杆 (Horizontal Stick)**:
  - 模式 "y": 左推/右推用于左右移动
  - 模式 "yaw_diff": 左推/右推用于调整偏航差 (用于 LeRobot SO100)

### 按钮控制

#### 位置控制

- **R 按钮 (右 Joy-Con) / L 按钮 (左 Joy-Con)**: 上升 (Z 轴正向)
- **ZR 按钮 (右 Joy-Con) / ZL 按钮 (左 Joy-Con)**: 下降 (Z 轴负向) 或 切换夹爪状态 (取决于 `change_down_to_gripper` 设置)
- **右摇杆按钮 (右 Joy-Con) / 左摇杆按钮 (左 Joy-Con)**: 下降 (如果 `change_down_to_gripper=True`)

#### 方向控制

- **X 按钮 (右 Joy-Con) / 上方向按钮 (左 Joy-Con)**: X 轴正向移动 (如果 `pure_dx=True`)
- **B 按钮 (右 Joy-Con) / 下方向按钮 (左 Joy-Con)**: X 轴负向移动 (如果 `pure_dx=True`)

#### 系统控制

- **Home 按钮 (右 Joy-Con) / Capture 按钮 (左 Joy-Con)**: 重置位置和姿态到偏移值
- **Plus 按钮 (右 Joy-Con) / Minus 按钮 (左 Joy-Con)**: 重置校准

#### 集控制 (仅右 Joy-Con)

- **A 按钮**: 下一集 (button_control = 1)
- **Y 按钮**: 重启集 (button_control = -1)

#### 附加按钮 (如果 `all_button_return=True`)

- **B 按钮**: button_control = 2
- **X 按钮**: button_control = 3
- **SL 按钮**: button_control = 4
- **SR 按钮**: button_control = 5
- **ZR/ZL 按钮**: button_control = 6
- **重置按钮**: button_control = 8

## API 参考

### JoyconRobotics 类

#### 初始化

```python
JoyconRobotics(device: str = "right",
               gripper_open: float = 1.0,
               gripper_close: float = 0.0,
               gripper_state: float = 1.0,
               horizontal_stick_mode: str = "y",
               close_y: bool = False,
               limit_dof: bool = False,
               glimit: list = [[0.125, -0.4, 0.046, -1.57, -1.5, -1.57],
                               [0.380, 0.4, 0.23, 1.57, 1.5, 1.57]],
               offset_position_m: list = [0.0, 0.0, 0.0],
               offset_euler_rad: list = [0.0, 0.0, 0.0],
               euler_reverse: list = [1, 1, 1],
               direction_reverse: list = [1, 1, 1],
               dof_speed: list = [1,1,1,1,1,1],
               rotation_filter_alpha_rate: float = 1,
               common_rad: bool = True,
               lerobot: bool = False,
               pitch_down_double: bool = False,
               without_rest_init: bool = False,
               pure_z: bool = True,
               change_down_to_gripper: bool = False,
               lowpassfilter_alpha_rate: float = 0.05,
               pure_dx: bool = True,
               all_button_return: bool = False)
```

主要参数：
- `device`: "right" 或 "left" Joy-Con
- `gripper_open/close`: 夹爪开/关值
- `limit_dof`: 是否限制自由度
- `glimit`: 位置和姿态限制 [[min], [max]]
- `offset_position_m/euler_rad`: 位置和姿态偏移
- `dof_speed`: 各自由度速度倍率 [x, y, z, roll, pitch, yaw]
- `lerobot`: 是否为 LeRobot 模式
- `change_down_to_gripper`: ZR 按钮切换夹爪而不是下降

#### 方法

##### 主要控制方法

- `update() -> (list, float, int)`: 更新状态，返回 (姿态, 夹爪状态, 按钮控制)
- `get_control(out_format="euler_rad") -> (list, float, int)`: 获取控制数据，支持 "euler_rad", "euler_deg", "quaternion" 格式

##### 传感器和状态方法

- `get_orientation() -> list`: 获取当前姿态 [roll, pitch, yaw] (弧度)
- `get_stick() -> (int, int, int)`: 获取摇杆状态 (垂直, 水平, 按钮)

##### 按钮监听

- `listen_button(button: str, show_all=False) -> int or None`: 监听特定按钮事件
  - 按钮名称：'r', 'zr', 'y', 'x', 'a', 'b', 'plus', 'r-stick', 'home', 'sr', 'sl' (右)
  - 'l', 'zl', 'up', 'down', 'right', 'left', 'minus', 'l-stick', 'capture', 'sr', 'sl' (左)

##### 配置方法

- `set_position(set_position: list)`: 设置位置 [x, y, z]
- `set_gripper_open_value(gripper_open: float)`: 设置夹爪打开值
- `set_gripper_close_value(gripper_close: float)`: 设置夹爪关闭值
- `open_gripper()`: 打开夹爪
- `close_gripper()`: 关闭夹爪
- `set_posture_limits(glimit: list)`: 设置姿态限制
- `set_dof_speed(dof_speed: list)`: 设置自由度速度

##### 系统方法

- `reset_joycon()`: 重置和校准 Joy-Con
- `disconnnect()`: 断开连接

## 注意事项

- 首次使用时会进行 2 秒校准，请将 Joy-Con 平放在桌面上。
- 位置和姿态会根据摇杆和按钮输入实时更新。
- 姿态使用互补滤波器融合加速度计和陀螺仪数据。
- 支持多种机器人平台配置 (LeRobot, 通用等)。
- 使用完毕后调用 `disconnnect()` 正确关闭连接。

## 故障排除

- 如果连接失败，确保 Joy-Con 已正确配对并支持机器人模式 (序列号以 '9c:54:' 开头)。
- 校准问题：重新运行 `reset_joycon()`。
- 响应不灵敏：调整 `dof_speed` 或 `lowpassfilter_alpha_rate`。
- 按钮映射错误：检查 `device` 参数和按钮事件监听。