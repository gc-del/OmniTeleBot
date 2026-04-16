# JoyCon Robotics Interface Documentation

## 概述

`joycon.py` 模块提供了一个 Python 接口，用于与 Nintendo Joy-Con 控制器通信。该模块允许开发者读取按钮状态、摇杆位置、加速度计和陀螺仪数据，以及控制玩家的灯。

## 依赖

- `hid` 或 `hidapi` 库，用于 HID 设备通信
- Python 3.x

## 安装

确保安装了必要的依赖：

```bash
pip install hidapi
```

## 使用示例

### 基本使用

```python
from joyconrobotics.joycon import JoyCon
from joyconrobotics.device import get_L_id, get_R_id

# 获取 Joy-Con ID（左或右）
ids = get_L_id() if None not in get_L_id() else get_R_id()

if None not in ids:
    # 创建 JoyCon 实例
    joycon = JoyCon(*ids)

    # 获取状态
    status = joycon.get_status()
    print(status)

    # 设置玩家灯
    joycon.set_player_lamp_on(1)

    # 断开连接
    joycon.disconnect_device()
```

### 注册更新钩子

```python
def my_callback(joycon):
    print("Button A pressed:", joycon.get_button_a())

joycon.register_update_hook(my_callback)
```

## API 参考

### JoyCon 类

#### 初始化

```python
JoyCon(vendor_id: int, product_id: int, serial: str = None, simple_mode=False)
```

- `vendor_id`: 供应商 ID（必须是 JOYCON_VENDOR_ID）
- `product_id`: 产品 ID（必须在 JOYCON_PRODUCT_IDS 中）
- `serial`: 序列号（可选）
- `simple_mode`: 简单模式（默认为 False）

#### 属性

- `vendor_id`: 供应商 ID
- `product_id`: 产品 ID
- `serial`: 序列号
- `simple_mode`: 简单模式
- `color_body`: 主体颜色 (R, G, B)
- `color_btn`: 按钮颜色 (R, G, B)

#### 方法

##### 电池相关

- `get_battery_charging() -> int`: 获取充电状态 (0 或 1)
- `get_battery_level() -> int`: 获取电池电量级别 (0-7)

##### 按钮状态

所有按钮方法返回 0（未按下）或 1（按下）：

- `get_button_y()`: Y 按钮
- `get_button_x()`: X 按钮
- `get_button_b()`: B 按钮
- `get_button_a()`: A 按钮
- `get_button_right_sr()`: 右 SR 按钮
- `get_button_right_sl()`: 右 SL 按钮
- `get_button_r()`: R 按钮
- `get_button_zr()`: ZR 按钮
- `get_button_minus()`: Minus 按钮
- `get_button_plus()`: Plus 按钮
- `get_button_r_stick()`: 右摇杆按钮
- `get_button_l_stick()`: 左摇杆按钮
- `get_button_home()`: Home 按钮
- `get_button_capture()`: Capture 按钮
- `get_button_charging_grip()`: 充电握把按钮
- `get_button_down()`: 下方向按钮
- `get_button_up()`: 上方向按钮
- `get_button_right()`: 右方向按钮
- `get_button_left()`: 左方向按钮
- `get_button_left_sr()`: 左 SR 按钮
- `get_button_left_sl()`: 左 SL 按钮
- `get_button_l()`: L 按钮
- `get_button_zl()`: ZL 按钮

##### 摇杆位置

- `get_stick_left_horizontal() -> int`: 左摇杆水平位置 (0-4095)
- `get_stick_left_vertical() -> int`: 左摇杆垂直位置 (0-4095)
- `get_stick_right_horizontal() -> int`: 右摇杆水平位置 (0-4095)
- `get_stick_right_vertical() -> int`: 右摇杆垂直位置 (0-4095)

##### 传感器数据

- `get_accel_x(sample_idx=0) -> float`: X 轴加速度
- `get_accel_y(sample_idx=0) -> float`: Y 轴加速度
- `get_accel_z(sample_idx=0) -> float`: Z 轴加速度
- `get_gyro_x(sample_idx=0) -> float`: X 轴陀螺仪
- `get_gyro_y(sample_idx=0) -> float`: Y 轴陀螺仪
- `get_gyro_z(sample_idx=0) -> float`: Z 轴陀螺仪

`sample_idx` 参数用于选择样本索引 (0, 1, 或 2)。

##### 其他方法

- `get_status() -> dict`: 获取所有状态的字典
- `register_update_hook(callback)`: 注册输入更新回调函数
- `is_left() -> bool`: 检查是否为左 Joy-Con
- `is_right() -> bool`: 检查是否为右 Joy-Con
- `set_player_lamp_on(on_pattern: int)`: 设置玩家灯开启模式
- `set_player_lamp_flashing(flashing_pattern: int)`: 设置玩家灯闪烁模式
- `set_player_lamp(pattern: int)`: 设置玩家灯模式
- `disconnect_device()`: 断开设备连接

## 注意事项

- 确保 Joy-Con 已正确配对并连接到系统。
- 某些方法可能需要管理员权限或特定的 HID 驱动程序。
- 传感器数据经过校准，但可能需要根据具体设备进行调整。
- 使用完毕后调用 `disconnect_device()` 以正确关闭连接。

## 故障排除

- 如果连接失败，检查 Joy-Con 是否正确连接并具有正确的权限。
- 确保安装了正确的 HID 库版本。
- 对于 Linux 系统，可能需要安装 `hidapi` 并配置 udev 规则。