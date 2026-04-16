# 遥操作控制方式说明

本文档详细介绍 Bobacbot 项目的两种遥操作控制方式：**键盘控制**和 **Joy-Con 手柄控制**。

## 📋 目录

- [快速开始](#快速开始)
- [键盘控制模式](#键盘控制模式)
- [Joy-Con 手柄控制模式](#joy-con-手柄控制模式)
- [底盘运动坐标系说明](#底盘运动坐标系说明)
- [常见问题](#常见问题)

---

## 🚀 快速开始

### 环境准备

确保已安装 bobacbot 环境：

```bash
conda activate bobacbot
```

### 启动仿真环境

```bash
# 仅查看仿真（无控制）
python main.py --sim

# 键盘控制模式
python main.py --sim --keyboard

# Joy-Con 控制模式
python main.py --sim --joycon
```

### 数据采集模式

```bash
# 键盘控制 + 数据保存
python main.py --sim --keyboard --save --output-dir data/demos

# Joy-Con 控制 + 数据保存
python main.py --sim --joycon --save --output-dir data/demos
```

---

## ⌨️ 键盘控制模式

### 适用场景

- ✅ 快速测试和调试
- ✅ 无需额外硬件
- ✅ 精确的离散控制
- ❌ 不适合流畅的连续动作采集

### 按键映射

#### 底盘控制（Base Control）

| 按键 | 功能 | 说明 |
|------|------|------|
| **W** | 前进 | 相对于机器人当前朝向 |
| **S** | 后退 | 相对于机器人当前朝向 |
| **A** | 左平移 | 相对于机器人当前朝向 |
| **D** | 右平移 | 相对于机器人当前朝向 |
| **Q** | 逆时针旋转 | 增加 yaw 角 |
| **E** | 顺时针旋转 | 减小 yaw 角 |

#### 机械臂末端控制（Arm End-Effector Control）

**线速度控制：**

| 按键 | 方向 | 轴 |
|------|------|-----|
| **I** | +X | 前后 |
| **K** | -X | 前后 |
| **J** | +Y | 左右 |
| **L** | -Y | 左右 |
| **U** | +Z | 上下 |
| **O** | -Z | 上下 |

**角速度控制：**

| 按键 | 旋转轴 | 说明 |
|------|--------|------|
| **T** | +Roll | 绕 X 轴旋转 |
| **G** | -Roll | 绕 X 轴旋转 |
| **Y** | +Pitch | 绕 Y 轴旋转 |
| **H** | -Pitch | 绕 Y 轴旋转 |
| **F** | +Yaw | 绕 Z 轴旋转 |
| **R** | -Yaw | 绕 Z 轴旋转 |

#### 夹爪控制（Gripper Control）

| 按键 | 功能 |
|------|------|
| **Z** | 闭合夹爪 |
| **X** | 打开夹爪 |

####  episode 控制

| 按键 | 功能 |
|------|------|
| **Space** | 启动/结束 episode，重置环境 |

### 速度参数

默认速度缩放系数（可在 `keyboard_policy.py` 中调整）：

```python
self.lin_vel_scale = 0.5   # 机械臂线速度 (m/s)
self.ang_vel_scale = 0.6   # 机械臂角速度 (rad/s)
self.gripper_scale = 0.5   # 夹爪速度
self.base_scale = 0.5      # 底盘速度 (m/s, rad/s)
```

### 使用示例

```bash
# 1. 启动键盘控制模式
python main.py --sim --keyboard

# 2. 按 Space 开始 episode
# 3. 使用 W/S/A/D/Q/E 控制底盘移动
# 4. 使用 I/K/J/L/U/O 控制机械臂位置
# 5. 使用 T/G/Y/H/F/R 控制机械臂姿态
# 6. 使用 Z/X 控制夹爪
# 7. 再次按 Space 结束 episode
# 8. 按 Space 重置环境，开始新 episode
```

---

## 🎮 Joy-Con 手柄控制模式

### 适用场景

- ✅ 流畅的连续动作采集
- ✅ 直观的 6D 位姿控制
- ✅ 适合模仿学习数据收集
- ❌ 需要 Nintendo Joy-Con 手柄和蓝牙连接

### 硬件要求

- **左手柄** (Left Joy-Con): 底盘控制
- **右手柄** (Right Joy-Con): 机械臂末端控制
- **蓝牙适配器**: Linux 系统需支持蓝牙 HID 协议

### 权限配置

首次使用前需要配置设备权限：

#### 方法 1: 临时方案（推荐测试用）

```bash
sudo python main.py --sim --joycon
```

#### 方法 2: 永久方案（推荐生产用）

创建 udev 规则文件 `/etc/udev/rules.d/99-joycon.rules`：

```bash
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="057e", MODE="0660", GROUP="input"
```

然后重新加载规则并重新插入手柄：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

将当前用户加入 input 组：

```bash
sudo usermod -a -G input $USER
```

**注意**: 需要注销并重新登录才能生效。

### 控制映射

#### 左手柄 - 底盘控制

| 输入 | 功能 | 说明 |
|------|------|------|
| **左摇杆 ↑/↓** | 前进/后退 | 相对于机器人当前朝向 |
| **左摇杆 ←/→** | 左平移/右平移 | 相对于机器人当前朝向 |
| **L 按钮** | 逆时针旋转 | 增加 yaw 角 |
| **ZL 按钮** | 顺时针旋转 | 减小 yaw 角 |

**步进参数**（可在 `joycon_policy.py` 中调整）：

```python
self.base_linear_step = 0.005  # 线性移动步长 (m)
self.base_yaw_step = 0.01      # 旋转步长 (rad)
```

#### 右手柄 - 机械臂末端控制

**基本操作：**

| 输入 | 功能 |
|------|------|
| **R 按钮（按住）** | 激活相对位姿控制 |
| **Home 按钮** | 重置参考位姿 |
| **Y 按钮** | 等同于 Space（启动/结束 episode） |

**位姿控制原理：**

Joy-Con 采用**相对位姿控制**模式：

1. **按下 R 按钮**时，记录当前手柄位姿作为参考点
2. **移动手柄**时，计算相对于参考点的位移和旋转
3. **释放 R 按钮**时，停止更新机械臂目标位姿
4. 机械臂会平滑移动到新的目标位姿

**优势：**
- 不受手柄绝对位姿限制
- 可以无限范围控制机械臂
- 避免漂移累积误差

**死区设置**（防止抖动）：

```python
self.right_pose_deadzone = 0.01    # 位置死区 (m)
self.right_rot_deadzone = 0.03     # 旋转死区 (rad)
self.right_stationary_frames_required = 30  # 静止帧数阈值
```

#### 夹爪控制

右手柄自动检测夹爪状态：

- **捏合手势** → 闭合夹爪
- **松开手势** → 打开夹爪

夹爪状态通过右手柄的 IMU 和按钮组合判断。

### Episode 控制流程

1. **启动程序**后，按 **Space** 或右手柄 **Y 按钮**开始 episode
2. 使用左右手柄协同控制机器人完成任务
3. 再次按 **Space** 或 **Y 按钮**结束 episode
4. 按 **Space** 重置环境
5. 重复步骤 1-4 进行多次演示

### 调试技巧

#### 实时监控控制值

在 `joycon_policy.py` 中取消注释调试代码：

```python
# 取消注释以下行以查看实时控制值
print(f"\n[DEBUG] Right Joy-Con Control Values:")
print(f"  Pose: x={pose[0]:.4f}, y={pose[1]:.4f}, z={pose[2]:.4f}")
print(f"        roll={pose[3]:.4f} rad ({math.degrees(pose[3]):.2f}°)")
print(f"        pitch={pose[4]:.4f} rad ({math.degrees(pose[4]):.2f}°)")
print(f"        yaw={pose[5]:.4f} rad ({math.degrees(pose[5]):.2f}°)")
print(f"  Gripper State: {gripper_state:.2f}")
```

#### 测试 Joy-Con 连接

运行测试脚本验证手柄是否正常工作：

```bash
python test_joycon_control.py
```

预期输出：

```
=== Joy-Con Control Test ===
Device: right
Pose: [0.000, 0.000, 0.000, 0.000, 0.000, 0.000]
Gripper: 0.00
Buttons: {...}
```

---

## 🧭 底盘运动坐标系说明

### 重要概念

两种控制方式都遵循**机体坐标系 → 世界坐标系**的转换规则：

#### 机体坐标系 (Body Frame)

- **X 轴**: 机器人前方
- **Y 轴**: 机器人左侧
- **Z 轴**: 机器人上方

#### 世界坐标系 (World Frame)

- **X 轴**: 固定方向（例如东）
- **Y 轴**: 固定方向（例如北）
- **Z 轴**: 重力反方向（向上）

### 坐标转换原理

当机器人朝向改变时，摇杆/按键的前后左右**始终相对于机器人自身朝向**，而非固定的世界坐标系。

**数学公式：**

```
世界坐标系速度 = 旋转矩阵(yaw) × 机体坐标系速度

[world_vx]   [cos(yaw)  -sin(yaw)] [body_vx]
[world_vy] = [sin(yaw)   cos(yaw)] [body_vy]
```

**代码实现：**

```python
# 获取当前 yaw 角
current_yaw = self.base_target_pose[2]

# 机体坐标系 → 世界坐标系
world_dx = body_x * np.cos(current_yaw) - body_y * np.sin(current_yaw)
world_dy = body_x * np.sin(current_yaw) + body_y * np.cos(current_yaw)

# 更新位置
self.base_target_pose[0] += step_size * world_dx
self.base_target_pose[1] += step_size * world_dy
```

### 实际效果示例

假设机器人初始朝向为 0°（东）：

| 操作 | 机器人朝向 | 推摇杆向前 | 实际移动方向 |
|------|-----------|-----------|-------------|
| 初始状态 | 0° (东) | ✓ | 向东 ✓ |
| 左转 90° | 90° (北) | ✓ | **向北** ✓ |
| 再转 90° | 180° (西) | ✓ | **向西** ✓ |

**如果没有坐标转换**（错误行为）：
- 无论机器人朝向如何，推摇杆向前总是向东移动 ❌

### 为什么这样设计？

1. **符合直觉**: 操作员不需要在脑海中做坐标变换
2. **易于学习**: "向前"就是机器人的前方
3. **减少错误**: 避免因朝向混乱导致的碰撞
4. **行业标准**: 大多数机器人控制系统都采用这种方式

---

## ❓ 常见问题

### Q1: Joy-Con 无法连接怎么办？

**检查清单：**

1. 确认蓝牙已开启：`bluetoothctl show`
2. 配对手柄：
   ```bash
   bluetoothctl
   scan on
   pair <MAC_ADDRESS>
   trust <MAC_ADDRESS>
   connect <MAC_ADDRESS>
   ```
3. 检查权限：`ls -l /dev/hidraw*`
4. 尝试 sudo 运行

### Q2: 键盘控制响应太慢/太快？

调整 `keyboard_policy.py` 中的速度参数：

```python
self.lin_vel_scale = 0.5   # 增大此值提高机械臂速度
self.base_scale = 0.5      # 增大此值提高底盘速度
```

### Q3: Joy-Con 控制有抖动？

调整死区参数：

```python
self.left_motion_deadzone = 0.15       # 增大以减少抖动
self.right_pose_deadzone = 0.01        # 增大位置死区
self.right_rot_deadzone = 0.03         # 增大旋转死区
```

### Q4: 底盘移动方向不对？

确认已应用最新的坐标转换修复（2024-10 之后版本已包含）。

检查代码中是否有以下转换逻辑：

```python
world_dx = base_x * np.cos(current_yaw) - base_y * np.sin(current_yaw)
world_dy = base_x * np.sin(current_yaw) + base_y * np.cos(current_yaw)
```

### Q5: 如何同时使用两种控制方式？

目前不支持同时启用。如需切换，请：

1. 停止当前程序 (`Ctrl+C`)
2. 重新启动并选择另一种模式：
   ```bash
   python main.py --sim --keyboard  # 或 --joycon
   ```

### Q6: 数据采集时如何保证质量？

**最佳实践：**

1. **流畅性**: 避免频繁启停，保持动作连贯
2. **多样性**: 从不同角度、距离执行任务
3. **完整性**: 每个 episode 包含完整的任务流程
4. **标注清晰**: 失败的演示也要保存（用于负样本）
5. **数量充足**: 每个任务至少 50-100 条演示

---

## 📚 相关文档

- [键盘控制详细说明](keyboard_control_usage.md)
- [电话遥操作映射](03_phone_teleop_mapping.md)
- [数据采集流程](06_teleoperation_data_collection.md)
- [仿真环境设置](01_mujoco_simulation_setup.md)

---

## 🔧 技术细节

### 控制周期

- **键盘控制**: 50 Hz (0.02s)
- **Joy-Con 控制**: 50 Hz (0.02s)

### 控制架构

```
用户输入 (Keyboard/Joy-Con)
    ↓
Policy (keyboard_policy.py / joycon_policy.py)
    ↓
动作指令 (base_pose, arm_pos, arm_quat, gripper_pos)
    ↓
Controller (base_controller.py / arm_controller.py)
    ↓
MuJoCo 仿真环境 (mujoco_env.py)
    ↓
观测反馈 (obs: base_pose, arm_pos, arm_quat, gripper_pos)
    ↓
循环回到 Policy
```

### 关键代码位置

- **键盘策略**: `bobacbot/keyboard_policy.py`
- **Joy-Con 策略**: `bobacbot/joycon_policy.py`
- **底盘控制器**: `bobacbot/base_controller.py`
- **机械臂控制器**: `bobacbot/arm_controller.py`
- **IK 求解器**: `bobacbot/ik_solver.py`

---

## 📝 更新日志

### 2024-10-XX

- ✅ 修复键盘控制底盘坐标系转换问题
- ✅ 修复 Joy-Con 控制底盘坐标系转换问题
- ✅ 添加底盘运动坐标系说明文档
- ✅ 统一两种控制方式的坐标转换逻辑

---

**维护者**: Bobacbot Team  
**最后更新**: 2024-10
