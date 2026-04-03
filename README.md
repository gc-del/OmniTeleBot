# Bobacbot

Bobacbot 是一个用于模仿学习和遥操作研究的全向移动操作机器人项目。当前代码基于 MuJoCo 仿真，集成了 Bobac3 全向底盘、Elephant Robotics eco65b 六自由度机械臂、Robotiq 2F-85 夹爪，以及相机观测、遥操作采集、Episode 回放和 Diffusion Policy 训练链路。

本仓库的目标是把“仿真、控制、采集、训练、推理”串成一条完整闭环，方便你直接采集抓取/放置演示数据，并训练模型用于抓取任务。

## 目录

- [项目概览](#项目概览)
- [核心能力](#核心能力)
- [环境准备](#环境准备)
- [快速开始](#快速开始)
- [遥操作模式](#遥操作模式)
- [数据采集](#数据采集)
- [数据格式](#数据格式)
- [数据转换与训练](#数据转换与训练)
- [策略推理](#策略推理)
- [项目结构](#项目结构)
- [常见问题](#常见问题)

## 项目概览

### 机器人配置

- 底盘：Bobac3 全向移动底盘，支持 `x / y / yaw` 三自由度
- 机械臂：eco65b 六自由度机械臂
- 末端执行器：Robotiq 2F-85 夹爪
- 视觉传感器：
  - 底盘前置相机 `base camera`
  - 手腕相机 `wrist camera`

### 当前工程特点

- 使用 MuJoCo 进行高频仿真
- 支持键盘控制、手机 WebXR 遥操作、Joy-Con 控制
- 支持 Episode 数据采集、回放和导出
- 支持转换为 robomimic / Diffusion Policy 可训练格式
- 支持训练后的策略服务器推理

## 核心能力

- 机器人仿真：`mujoco_env.py`
- 控制策略：`policies.py`、`keyboard_policy.py`、`joycon_policy.py`
- 数据存储：`episode_storage.py`
- 数据转换：`convert_to_robomimic_hdf5.py`
- 回放工具：`replay_episodes.py`
- 机械臂 IK：`ik_solver.py`
- 机械臂低层控制：`arm_controller.py`
- 底盘低层控制：`base_controller.py`

## 环境准备

### 1. 推荐环境

建议使用 Conda / Miniforge：

```bash
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh
```

### 2. 创建运行环境

```bash
conda create -n bobacbot python=3.10.14
conda activate bobacbot
pip install -r requirements.txt
```

### 3. 训练环境

Diffusion Policy 训练建议单独环境：

```bash
cd training/diffusion_policy
conda env create -f conda_environment.yaml
conda activate robodiff
```

### 4. Joy-Con 设备权限

如果使用 Joy-Con 控制，Linux 下可能需要 `hidraw` 权限。仓库中提供了 udev 规则示例：`99-joycon.rules`。

典型处理方式：

```bash
sudo cp 99-joycon.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

重新插拔手柄后再启动程序。

## 快速开始

### 1. 启动仿真

仅查看仿真：

```bash
python main.py --sim
```

键盘控制：

```bash
python main.py --sim --keyboard --save --output-dir data/demos
```

Joy-Con 控制：

```bash
sudo /home/wufuwei/miniconda3/envs/bobacbot/bin/python main.py --sim --joycon
```

手机 WebXR 遥操作：

```bash
python main.py --sim --teleop --save --output-dir data/demos
```

### 2. 运行逻辑

启动后，程序会：

1. 初始化 MuJoCo 仿真
2. 根据模式创建对应策略
3. 周期性读取观测
4. 策略输出动作
5. 将动作送入仿真
6. 如启用 `--save`，同时写出 Episode 数据

## 遥操作模式

### 键盘控制

键盘控制适合调试和采集基础数据。

控制约定：

- `W/S`：底盘前后
- `A/D`：底盘左右
- `Q/E`：底盘旋转
- `I/K`：机械臂位置前后
- `J/L`：机械臂位置左右
- `U/O`：机械臂位置上下
- `T/G`：机械臂 roll
- `Y/H`：机械臂 pitch
- `F/R`：机械臂 yaw
- `Z/X`：夹爪开合

空格键用于 episode 开始 / 结束 / 重置流程。

### Joy-Con 控制

Joy-Con 模式分工如下：

- 左手柄：控制底盘
- 右手柄：控制机械臂和夹爪

右手柄输出遵循官方 Joy-Con 控制链路的位姿约定：

- 前 3 个量：位置 `x / y / z`
- 后 3 个量：姿态 `roll / pitch / yaw`

当前工程里，机械臂控制最终会被转换成末端位姿目标，再交给 IK 求解整条机械臂关节解。

### 手机遥操作

WebXR 遥操作适合采集较自然的演示数据。流程：

1. 启动 `python main.py --sim --teleop --save --output-dir data/demos`
2. 用手机浏览器打开终端输出的地址
3. 根据页面提示进行控制

## 数据采集

### Episode 形式

每次采集会生成一个独立目录，常见结构如下：

```text
data/demos/20241214T153045123456/
├── data.pkl
├── base_image.mp4
└── wrist_image.mp4
```

### 采集建议

- 保证动作连续，不要频繁抖动手柄
- 抓取任务尽量从简单目标开始
- 每个任务尽量采集多样化初始位姿
- 如果用于训练，建议采集几十到上百条演示

### 保存数据

```bash
python main.py --sim --keyboard --save --output-dir data/demos
```

或：

```bash
python main.py --sim --teleop --save --output-dir data/demos
```

### 回放数据

```bash
python replay_episodes.py --sim --input-dir data/demos
python replay_episodes.py --sim --input-dir data/demos --show-images
python replay_episodes.py --sim --input-dir data/demos --execute-obs
```

## 数据格式

### Episode 格式

每个 episode 都是一个目录，包含：

- `data.pkl`：观测和动作序列
- `base_image.mp4`：底盘相机视频
- `wrist_image.mp4`：手腕相机视频

`data.pkl` 中通常包含：

- `timestamps`
- `observations`
- `actions`

### 观测字段

常见观测字段：

- `base_pose`：底盘位姿 `[x, y, theta]`
- `arm_pos`：机械臂末端位置 `[x, y, z]`
- `arm_quat`：机械臂末端四元数 `[x, y, z, w]`
- `gripper_pos`：夹爪开合状态
- `base_image`：底盘相机图像
- `wrist_image`：手腕相机图像

### 动作字段

动作会被整理为可训练向量，通常包含：

- 底盘位姿
- 机械臂末端位置
- 机械臂旋转表示
- 夹爪状态

## 数据转换与训练

### 1. Episode 转 HDF5

```bash
python convert_to_robomimic_hdf5.py --input-dir data/demos --output-path data/demos.hdf5
```

脚本会：

1. 读取每个 episode
2. 缩放图像到训练尺寸
3. 提取观测和动作
4. 输出 robomimic 风格的 HDF5 文件

### 2. 拷贝到训练环境

```bash
cp data/demos.hdf5 training/diffusion_policy/data/
```

### 3. 配置任务

编辑 `training/diffusion_policy/diffusion_policy/config/task/square_image_abs.yaml`，把任务名改成你的数据集名称。

### 4. 应用补丁

```bash
cd training/diffusion_policy
git checkout 548a52b
git apply ../../diffusion-policy.patch
```

### 5. 开始训练

```bash
cd training/diffusion_policy
conda activate robodiff
python train.py --config-name=train_diffusion_unet_real_hybrid_workspace task=square_image_abs
```

### 6. 查看训练结果

```bash
tensorboard --logdir data/outputs
```

## 策略推理

### 1. 启动策略服务器

```bash
cd training/diffusion_policy
conda activate robodiff
python policy_server.py --ckpt-path data/outputs/<your_run>/checkpoints/epoch=<N>-train_loss=<X>.ckpt
```

### 2. 仿真端运行

```bash
conda activate bobacbot
python main.py --sim
```

如果策略服务器不在本机，先建立 SSH 隧道：

```bash
ssh -L 5555:localhost:5555 <gpu-machine>
```

## 项目结构

```text
Bobacbot/
├── arm_controller.py
├── arm_server.py
├── base_controller.py
├── base_server.py
├── cameras.py
├── constants.py
├── convert_to_robomimic_hdf5.py
├── episode_storage.py
├── ik_solver.py
├── joycon_policy.py
├── keyboard_policy.py
├── main.py
├── mujoco_env.py
├── policies.py
├── policy_server.py
├── replay_episodes.py
├── real_env.py
├── requirements.txt
├── README.md
├── README_CN.md
├── data/
├── docs/
├── models/
├── bobac3_description/
├── joycon-robotics-master/
└── training/
```

## 常用命令

```bash
# 仿真
python main.py --sim

# 键盘遥操作
python main.py --sim --keyboard --save --output-dir data/demos

# Joy-Con 遥操作
sudo /home/wufuwei/miniconda3/envs/bobacbot/bin/python main.py --sim --joycon

# 手机遥操作
python main.py --sim --teleop --save --output-dir data/demos

# 回放
python replay_episodes.py --sim --input-dir data/demos

# 转换数据格式
python convert_to_robomimic_hdf5.py --input-dir data/demos --output-path data/demos.hdf5

# 训练
cd training/diffusion_policy
conda activate robodiff
python train.py --config-name=train_diffusion_unet_real_hybrid_workspace task=square_image_abs
```

## 常见问题

### 1. 仿真启动后相机窗口太多

默认仿真现在不会自动弹相机窗口。如果你还看到窗口，检查是否运行了单独的相机脚本或回放时加了 `--show-images`。

### 2. Joy-Con 识别失败

- 确认手柄已连接
- 确认 `hidraw` 权限正确
- 尝试使用 `sudo` 启动

### 3. 训练时报 `catkin_pkg` 缺失

这是 ROS1 相关包的依赖问题。请先安装对应 Python 依赖，或者在只使用仿真和训练部分时避免构建这些 ROS 包。

### 4. 回放不完全成功

如果 episode 包含随机初始位姿，开环回放不一定会成功，这属于正常现象。

### 5. 抓取效果不稳定

- 增加更一致的演示数据
- 保证相机视角稳定
- 检查末端位姿和夹爪开合标注是否正确

## 推荐文档顺序

1. [docs/README.md](docs/README.md)
2. [docs/01_mujoco_simulation_setup.md](docs/01_mujoco_simulation_setup.md)
3. [docs/06_teleoperation_data_collection.md](docs/06_teleoperation_data_collection.md)
4. [docs/10_data_format.md](docs/10_data_format.md)
5. [docs/08_model_training.md](docs/08_model_training.md)
6. [docs/07_policy_deployment.md](docs/07_policy_deployment.md)

## 备注

- 当前仓库以仿真和数据闭环为主，真实机器人部分需要额外硬件和网络环境支持
- 如果你只想先跑通抓取模型训练，建议先从 `--sim` 仿真数据开始采集
- 如果你要做真实抓取，建议先验证控制、相机和数据格式，再进入训练阶段

- `--teleop`: 启用手机遥操作模式
- `--keyboard`: 启用键盘控制模式
- `--save`: 保存 episode 数据
- `--random`: 随机化机器人初始位置
- `--output-dir`: 数据保存目录（默认：`data/0714`）

### 机器人控制

- **底盘**: 3自由度全向移动（x, y, yaw）
- **机械臂**: 6自由度位姿控制（position + quaternion）
- **夹爪**: 0-1 归一化控制（0=闭合，1=打开）
- **控制频率**: 10 Hz（100ms 周期）

### 观测空间

```python
obs = {
    'base_pose': (3,),           # 底盘位姿 [x, y, theta]
    'arm_pos': (3,),             # 机械臂位置 [x, y, z]
    'arm_quat': (4,),            # 机械臂四元数 [x, y, z, w]
    'gripper_pos': (1,),         # 夹爪位置 [0-1]
    'base_image': (360, 640, 3), # 底盘相机图像
    'wrist_image': (480, 640, 3) # 手腕相机图像
}
```

### 动作空间

```python
action = {
    'base_pose': (3,),      # 目标底盘位姿
    'arm_pos': (3,),        # 目标机械臂位置
    'arm_quat': (4,),       # 目标机械臂四元数
    'gripper_pos': (1,),    # 目标夹爪位置
}
```

## 常见问题

### Q1: 仿真运行缓慢？
A: 尝试缩小 MuJoCo viewer 窗口大小，这会加速离屏渲染。

### Q2: 机械臂重置位置不正确？
A: 检查 `mujoco_env.py` 中 `ArmController.reset()` 的 `qpos` 初始值是否与 eco65b 的 retract keyframe 匹配。

### Q3: 训练时 OOM（内存不足）？
A: 减小 batch size 或降低图像分辨率（在 `constants.py` 修改 `POLICY_IMAGE_WIDTH/HEIGHT`）。

### Q4: 策略推理时无响应？
A: 确认策略服务器已启动，且 SSH 隧道连接正常（`ssh -L 5555:localhost:5555`）。

### Q5: 手机 WebXR 无法连接？
A: 确保手机和电脑在同一网络，浏览器支持 WebXR（推荐 Chrome），使用 HTTPS 或 localhost。

## 参考文献

### TidyBot++
```
@inproceedings{wu2024tidybot,
  title = {TidyBot++: An Open-Source Holonomic Mobile Manipulator for Robot Learning},
  author = {Wu, Jimmy and Chong, William and Holmberg, Robert and Prasad, Aaditya and Gao, Yihuai and Khatib, Oussama and Song, Shuran and Rusinkiewicz, Szymon and Bohg, Jeannette},
  booktitle = {Conference on Robot Learning (CoRL)},
  year = {2024}
}
```

- 项目主页: http://tidybot2.github.io
- 代码仓库: https://github.com/jimmyyhwu/tidybot2
- 论文: https://arxiv.org/abs/2412.10447

### Diffusion Policy
```
@inproceedings{chi2023diffusionpolicy,
  title = {Diffusion Policy: Visuomotor Policy Learning via Action Diffusion},
  author = {Chi, Cheng and Feng, Siyuan and Du, Yilun and Xu, Zhenjia and Cousineau, Eric and Burchfiel, Benjamin and Song, Shuran},
  booktitle = {Robotics: Science and Systems (RSS)},
  year = {2023}
}
```

- 项目主页: https://diffusion-policy.cs.columbia.edu/
- 代码仓库: https://github.com/real-stanford/diffusion_policy

## 致谢

本项目基于以下开源项目：
- [TidyBot++](https://github.com/jimmyyhwu/tidybot2): 全向移动操作机器人平台
- [Diffusion Policy](https://github.com/real-stanford/diffusion_policy): 扩散策略模仿学习
- [MuJoCo](https://mujoco.org/): 物理仿真引擎
- [Ruckig](https://github.com/pantor/ruckig): 实时轨迹生成

## 许可证

本项目继承 TidyBot++ 的许可证。详见 [LICENSE](LICENSE) 文件。