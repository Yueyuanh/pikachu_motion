# Gait Playground

Gait Playground是一个用于生成和回放机器人步态数据的交互式工具。它提供了一个基于Web的界面来调整步态参数，并可以将生成的运动数据保存为JSON格式或转换为TXT格式用于硬件执行。

## 功能特性

- 实时调整步态参数（前进速度、侧向速度、旋转速度等）
- Web界面实时预览机器人运动
- 生成JSON格式的运动数据
- 回放现有JSON或TXT格式的运动数据
- 支持多种机器人模型（go_bdx, open_duck_mini, open_duck_mini_v2, pikachu）

## 安装和运行

1. 确保已安装Python 3.10+和依赖库
2. 在项目根目录运行：

```bash
cd reference_motion_generator
python gait_playground.py --duck pikachu
```

## 参数说明

### 基本参数
- `--duck`: 选择机器人类型 (go_bdx, open_duck_mini, open_duck_mini_v2, pikachu)
- `--preset`: 指定预设文件路径
- `--output_dir`: 指定输出目录

### 步态参数
- `--dx`: 前进速度 (m/s)
- `--dy`: 侧向速度 (m/s)
- `--dtheta`: 旋转速度 (rad/s)
- `--double_support_ratio`: 双支撑相比例
- `--startend_double_support_ratio`: 起始/结束双支撑相比例
- `--planned_timesteps`: 规划时间步数
- `--replan_timesteps`: 重规划时间步数
- `--walk_com_height`: 质心高度
- `--walk_foot_height`: 足部抬升高度
- `--walk_trunk_pitch`: 躯干俯仰角 (度)
- `--walk_foot_rise_ratio`: 足部抬升比例
- `--single_support_duration`: 单支撑相持续时间
- `--single_support_timesteps`: 单支撑相时间步数
- `--foot_length`: 足部长度
- `--feet_spacing`: 双足间距
- `--zmp_margin`: 零力矩点边界余量
- `--foot_zmp_target_x`: 足部零力矩点目标X坐标
- `--foot_zmp_target_y`: 足部零力矩点目标Y坐标
- `--walk_max_dtheta`: 最大旋转速度
- `--walk_max_dy`: 最大侧向速度
- `--walk_max_dx_forward`: 最大前进速度
- `--walk_max_dx_backward`: 最大后退速度

### 回放模式参数
- `--json <file.json>`: 指定JSON文件进行回放模式
- `--txt <file.txt>`: 指定TXT文件进行回放模式

## 回放模式

### JSON回放
要回放现有的JSON格式运动数据：

```bash
python gait_playground.py --duck pikachu --json path/to/your/file.json
```

### TXT回放
要回放现有的TXT格式运动数据：

```bash
python gait_playground.py --duck pikachu --txt path/to/your/file.txt
```

在回放模式下，程序将：
- 加载指定的JSON或TXT文件
- 直接播放文件中的运动数据，不使用动力学生成
- 禁用参数控制滑块，防止在回放过程中修改参数
- 在Web界面中显示"Replay Mode Active"状态

## 数据格式说明

### JSON格式
JSON文件包含以下结构：
- `LoopMode`: 循环模式
- `FrameDuration`: 帧持续时间
- `EnableCycleOffsetPosition`: 是否启用循环偏移位置
- `EnableCycleOffsetRotation`: 是否启用循环偏移旋转
- `MotionWeight`: 运动权重
- `Frames`: 帧数据数组，每帧包含位置、姿态、关节角度等信息

### TXT格式
TXT文件使用JSON格式存储，但数据结构按以下顺序排列：
- 根位置 (x, y, z)
- 根姿态四元数 (qx, qy, qz, qw)
- 关节位置
- 左足位置
- 右足位置
- 线速度
- 角速度
- 关节速度
- 左足速度
- 右足速度

## 使用方法

### 生成模式
1. 运行gait_playground.py启动Web服务器
2. 打开浏览器访问 http://127.0.0.1:5000
3. 调整步态参数
4. 点击"Run"开始生成运动数据
5. 生成的数据会保存在episode变量中

### 回放模式
在回放模式下，程序将加载指定文件并在Web界面中显示回放的机器人运动。参数控制将被禁用，确保不会干扰回放过程。

## 输出

生成的运动数据保存为episode格式，包含：
- 帧数据 (Frames)
- 调试信息 (Debug_info)
- 循环设置和运动权重