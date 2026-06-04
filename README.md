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


## 快速开始

### 1. 启动仿真

仅查看仿真：

```bash
python main.py --sim
```

键盘控制：

```bash
python main.py --sim --keyboard 
```

Joy-Con手柄控制：

```bash
python main.py --sim --joycon
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

### 键盘控制 --- 仿真

```bash
# 键盘控制 + 数据采集（比赛场景）
python main.py --sim --keyboard --save --output-dir data/demos \
    --scene models/bobacbot_competition_scene.xml \
    --robot-pos 0.0 -1.5 --robot-yaw 1.57
```

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

### Joy-Con 控制 --- 仿真

```bash
# Joy-Con 控制 + 数据采集 + 图像显示（比赛场景）
python main.py --sim --joycon --save --output-dir data/demos \
    --scene models/bobacbot_competition_scene.xml \
    --robot-pos 0.0 -1.5 --robot-yaw 1.57 --show-images
```


Joy-Con 模式分工如下：

- 左手柄：控制底盘
- 右手柄：控制机械臂和夹爪

右手柄输出遵循官方 Joy-Con 控制链路的位姿约定：

- 前 3 个量：位置 `x / y / z`
- 后 3 个量：姿态 `roll / pitch / yaw`

当前工程里，机械臂控制最终会被转换成末端位姿目标，再交给 IK 求解整条机械臂关节解。

### Joy-Con 控制 --- 实物

```bash
# Joy-Con 控制真实机器人
python main.py --joycon-real --websocket-url ws://192.168.1.204:6060
```

Joy-Con 模式分工如下：

- 左手柄：控制底盘
- 右手柄：控制机械臂和夹爪

右手柄输出遵循官方 Joy-Con 控制链路的位姿约定：

- 前 3 个量：位置 `x / y / z`
- 后 3 个量：姿态 `roll / pitch / yaw`

当前工程里，机械臂控制最终会被转换成末端位姿目标，再交给 IK 求解整条机械臂关节解。

### Meta Quest3 控制 --- 实物

VR 遥操作，使用 Meta Quest 3 浏览器连接控制。

```bash
# 启动 VR 遥操作 + 数据采集
python main.py --vr-real \
    --save \
    --output-dir data/episodes \
    --websocket-url ws://192.168.1.220:6060 \
    --image-host 192.168.1.220 \
    --hand-cam-port 55555 \
    --base-cam-port 55556
```

Quest 3 打开 `https://<服务器IP>:8443`，点击 **Enter VR** 进入控制。
按空格键开始/停止录制，终端提示 `y/n` 保存。

| 左手柄 | 功能 |
|--------|------|
| 摇杆 Y/X | 底盘前后/平移 |
| trigger（扳机） | 底盘左转（线性调速） |
| grip（握持） | 底盘右转 |

| 右手柄 | 功能 |
|--------|------|
| grip + 移动 | 机械臂位姿控制 |
| trigger | 夹爪闭合 |



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
python main.py --sim --keyboard --save --output-dir data/demos(练习场地)
python main.py --sim --keyboard --save --output-dir data/demos   --scene models/bobacbot_competition_scene.xml   --robot-pos 0.0 -1.5 --robot-yaw 1.57（比赛场地）
```

或：

```bash
python main.py --sim --joycon --save --output-dir data/demos（练习场地）
python main.py --sim --joycon --save --output-dir data/demos   --scene models/bobacbot_competition_scene.xml   --robot-pos 0.0 -1.5 --robot-yaw 1.57（比赛场地）
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
bobac3bot/
├── LICENSE
├── README.md
├── requirements.txt
├── bobacbot/                         # 主代码目录
│   ├── main.py                       # 入口脚本
│   ├── constants.py                  # 常量定义
│   ├── policies.py                   # 策略基类
│   ├── episode_storage.py            # Episode 数据存储
│   ├── ik_solver.py                  # 逆运动学求解器
│   ├── policy_server.py              # 策略推理服务器
│   ├── plot_demos.ipynb              # 演示数据可视化
│   ├── diffusion-policy.patch        # Diffusion Policy 补丁
│   ├── kinova/                       # Kinova 真实机器人控制
│   │   ├── arm_controller.py         #   机械臂低层控制
│   │   ├── arm_server.py             #   机械臂服务端
│   │   ├── base_controller.py        #   底盘低层控制
│   │   ├── base_server.py            #   底盘服务端
│   │   ├── cameras.py                #   相机驱动
│   │   ├── real_env.py               #   真实环境封装
│   │   ├── vr_real_policy.py         #   VR 真实机器人策略
│   │   └── xl_vr_policy.py           #   XL VR 策略
│   ├── sim/                          # 仿真模块
│   │   ├── mujoco_env.py             #   MuJoCo 仿真环境
│   │   ├── replay_episodes.py        #   Episode 回放
│   │   ├── convert_to_robomimic_hdf5.py  #   数据格式转换
│   │   └── plot_base_state.py        #   底盘状态可视化
│   ├── keyboard/                     # 键盘控制策略
│   │   └── keyboard_policy.py
│   ├── joycon/                       # Joy-Con 手柄控制
│   │   ├── joycon_policy.py          #   仿真 Joy-Con 策略
│   │   └── joycon_real_policy.py     #   真实 Joy-Con 策略
│   ├── joyconrobotics/               # Joy-Con 底层驱动库
│   ├── telegrip/                     # Telegrip VR 遥操作子系统
│   │   ├── telegrip_adapter.py       #   bobacbot 适配器
│   │   ├── telegrip/                 #   telegrip 核心包
│   │   │   ├── main.py               #     系统主控
│   │   │   ├── config.py             #     配置管理
│   │   │   ├── control_loop.py       #     控制循环
│   │   │   ├── core/                 #     核心模块（运动学、协议、机器人接口）
│   │   │   ├── inputs/               #     输入模块（VR WS、键盘、基类）
│   │   │   └── utils.py              #     工具函数
│   │   ├── web-ui/                   #   Web 前端界面
│   │   └── URDF/                     #   机器人 URDF 模型
│   ├── models/                       # MuJoCo 模型与场景
│   │   ├── bobacbot.xml              #   基础场景
│   │   ├── bobacbot_competition_scene.xml  #   比赛场景
│   │   ├── bobacbot_demo_scene.xml   #   演示场景
│   │   ├── bobac3/                   #   Bobac3 底盘
│   │   ├── eco65b/                   #   eco65b 机械臂
│   │   ├── 2f85/                     #   2F-85 夹爪
│   │   ├── gen72/                    #   Gen72 机械臂
│   │   ├── kinova_gen3/              #   Kinova Gen3
│   │   ├── competition/              #   比赛场地 STL
│   │   ├── assets/                   #   模型资源
│   │   └── textures/                 #   纹理贴图
│   ├── data/                         # 采集数据目录
│   │   └── episodes/                 #   Episode 数据
│   ├── docs/                         # 文档
│   ├── static/                       # 静态资源（WebXR）
│   └── assets/                       # 资源文件
└── memories/                         # 记忆/笔记
```

## 常用命令

```bash
# 仿真
python main.py --sim

# 键盘遥操作
python main.py --sim --keyboard --save --output-dir data/demos

# Joy-Con 遥操作
python main.py --sim --joycon


# 回放
python replay_episodes.py --sim --input-dir data/demos

# 转换数据格式
python convert_to_robomimic_hdf5.py --input-dir data/demos --output-path data/demos.hdf5

# 训练
cd training/diffusion_policy
conda activate robodiff
python train.py --config-name=train_diffusion_unet_real_hybrid_workspace task=square_image_abs
```

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