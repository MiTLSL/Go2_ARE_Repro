# go2_demo 项目复现说明

本文档感谢b站up 猫猫烤肉师傅
本文档根据当前仓库代码和 `RoboGo2.pdf` 的本地识别结果整理，用于说明这个 Isaac Lab 项目的目标、代码结构、训练方式，以及如何围绕论文中的 Adaptive Energy Regularization 方法做 Go2 速度跟踪复现。

当前项目目录：

```text
C:\Users\Administrator\Desktop\Learning\Code\RobotProject\go2_demo
```

## 1. PDF 识别结果与论文核心思想

`RoboGo2.pdf` 可以被本机 PyMuPDF 正常识别，共 7 页。论文题目为：

```text
Adaptive Energy Regularization for Autonomous Gait Transition and
Energy-Efficient Quadruped Locomotion
```

作者在论文中研究的问题是：四足机器人速度跟踪任务中，是否可以减少复杂步态奖励和人工步态先验，只通过“速度跟踪 + 距离归一化能量奖励 + 少量安全辅助项”训练出可以自动切换步态的策略。

论文的主要结论如下：

1. 在强化学习四足运动控制中加入速度相关的能量奖励，可以让机器人在不同速度下自动选择更省能的步态。
2. 低速时策略倾向于四拍步行，中速时倾向于对角小跑，高速时可能出现腾空小跑。
3. 能量奖励权重太小，机器人可能仍然采用高能耗或不自然动作；权重太大，则会牺牲速度跟踪，严重时直接趋向不动。
4. 论文在 Unitree Go1 上验证了该思路，包括直线速度跟踪、角速度跟踪、圆轨迹跟踪、复杂地形和真实机器人部署。

论文中的总奖励结构可以概括为：

```text
R = (R_motion + alpha_en * R_en(vx, wz)) * exp(-R_aux)
```

其中：

```text
R_motion = R_lin + alpha_ang * R_ang
R_lin = exp(-((vx - vx_cmd)^2 + (vy - vy_cmd)^2) / sigma_v)
R_ang = exp(-((wz - wz_cmd)^2) / sigma_w)
R_en = exp(- sum_i(|tau_i| * |qdot_i|) / (sigma_en_x * |vx| + sigma_en_z * |wz|))
```

论文给出的关键参数包括：

```text
sigma_v = 0.25
sigma_w = 0.25
sigma_en_x = 1000
sigma_en_z = 500
alpha_ang = 0.5
alpha_en 可做 0.0、0.5、1.0、1.5 等消融实验
```

这个仓库当前不是论文原始代码的完整复刻，而是把论文方法迁移到 Isaac Lab + RSL-RL + Unitree Go2 模型上的复现实验项目。

## 2. 项目目标

本项目的主要目标是训练 Unitree Go2 机器人完成平面速度跟踪任务，并尝试复现论文中的 AER 思路：

1. 使用 Isaac Lab 的 Manager-Based RL 环境组织场景、观测、动作、奖励、终止条件和课程学习。
2. 使用 RSL-RL PPO 训练 12 关节位置控制策略。
3. 使用能量正则项 `energy_new_actual` 鼓励单位运动距离下更低的电机功率消耗。
4. 使用自定义 `AERRewardManager` 将正奖励和辅助惩罚组合为类似论文的指数门控形式。
5. 通过命令速度随机采样和 debug 可视化观察机器人速度跟踪行为、命令箭头和地形/高度扫描信息。

## 3. 项目结构

```text
go2_demo/
+-- RoboGo2.pdf
+-- README.md
+-- PROJECT_DESCRIPTION.md
+-- pyproject.toml
+-- scripts/
|   +-- list_envs.py
|   +-- random_agent.py
|   +-- zero_agent.py
|   +-- rsl_rl/
|       +-- train.py
|       +-- play.py
|       +-- cli_args.py
+-- source/
    +-- go2_demo/
        +-- setup.py
        +-- pyproject.toml
        +-- config/
        |   +-- extension.toml
        +-- go2_demo/
            +-- __init__.py
            +-- ui_extension_example.py
            +-- assets/
            |   +-- robot/
            |       +-- unitree.py
            |       +-- unitree_model/
            |           +-- Go2/usd/go2.usd
            +-- tasks/
                +-- manager_based/
                    +-- go2_demo/
                        +-- __init__.py
                        +-- aer_env.py
                        +-- go2_demo_env_cfg.py
                        +-- go2_demo_velocity.py
                        +-- agents/
                        |   +-- rsl_rl_ppo_cfg.py
                        +-- mdp/
                            +-- __init__.py
                            +-- commands.py
                            +-- curriculums.py
                            +-- rewards.py
```

重要文件说明：

| 文件 | 作用 |
| --- | --- |
| `source/go2_demo/go2_demo/tasks/manager_based/go2_demo/__init__.py` | 注册 Gym 环境，包括 `Go2-velocity-v0` |
| `go2_demo_velocity.py` | Go2 速度跟踪任务主配置，包含场景、观测、动作、命令、奖励、终止、课程学习 |
| `aer_env.py` | 自定义 AER 奖励管理器和环境类 |
| `mdp/rewards.py` | 自定义能量奖励、脚底滑移奖励等 |
| `mdp/commands.py` | 扩展 Isaac Lab 的速度命令配置，增加 `limit_ranges` |
| `mdp/curriculums.py` | 根据速度跟踪奖励调整命令速度范围 |
| `assets/robot/unitree.py` | Unitree Go2 USD 模型和电机参数 |
| `agents/rsl_rl_ppo_cfg.py` | RSL-RL PPO 训练超参数 |
| `scripts/rsl_rl/train.py` | 训练入口 |
| `scripts/rsl_rl/play.py` | 读取 checkpoint 播放和导出策略 |

## 4. 当前注册任务

当前 `__init__.py` 中注册了两个任务：

```text
Template-Go2-Demo-v0
Go2-velocity-v0
```

真正用于 Go2 速度跟踪/AER 复现的是：

```text
Go2-velocity-v0
```

该任务入口为：

```text
entry_point = go2_demo.tasks.manager_based.go2_demo.aer_env:AERManagerBasedRLEnv
env_cfg_entry_point = go2_demo_velocity:GO2RobotDemoEnv
rsl_rl_cfg_entry_point = agents.rsl_rl_ppo_cfg:PPORunnerCfg
```

注意：当前任务实际通过 `source/go2_demo/go2_demo/tasks/manager_based/go2_demo/__init__.py` 中的 Gym 注册入口加载。

## 5. Go2 机器人配置

机器人配置位于：

```text
source/go2_demo/go2_demo/assets/robot/unitree.py
```

当前使用的模型是本地 Unitree Go2 USD：

```text
source/go2_demo/go2_demo/assets/robot/unitree_model/Go2/usd/go2.usd
```

当前代码中 `UNITREE_MODEL_DIR` 是绝对路径：

```python
UNITREE_MODEL_DIR = "C:/Users/Administrator/Desktop/Learning/Code/RobotProject/go2_demo/source/go2_demo/go2_demo/assets/robot/unitree_model"
```

这在当前机器上可以使用，但如果换电脑或换目录，需要改成相对路径或基于 `__file__` 动态拼接。

电机配置使用 `DCMotorCfg`，控制 12 个腿部关节：

```text
joint_names_expr = [".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"]
effort_limit = 23.5
saturation_effort = 23.5
velocity_limit = 30.0
stiffness = 25.0
damping = 0.5
friction = 0.0
```

初始姿态：

```text
base position = (0.0, 0.0, 0.4)
hip joints = 0.0
front thigh joints = 0.8
rear thigh joints = 0.8
calf joints = -1.5
```

## 6. 场景与可视化

主任务配置位于：

```text
source/go2_demo/go2_demo/tasks/manager_based/go2_demo/go2_demo_velocity.py
```

当前场景为平面地形：

```text
terrain_type = "plane"
static_friction = 1.0
dynamic_friction = 1.0
terrain.debug_vis = True
```

场景中包含：

1. `/World/ground` 平面地形。
2. `{ENV_REGEX_NS}/Robot` Go2 机器人。
3. `height_scanner` 高度扫描传感器，网格大小 `1.6 x 1.0`，分辨率 `0.1`，`debug_vis=True`。
4. `contact_forces` 接触传感器，跟踪机器人所有 body 的接触力和脚部离地时间。
5. `sky_light` 天空光源。

速度命令配置中：

```text
debug_vis = True
```

因此在非 headless 训练或播放时，可以看到随机速度命令箭头。若看不到箭头，优先检查：

1. 是否使用 GUI 模式而不是 `--headless`。
2. 是否只开了少量环境，例如 `--num_envs 1` 或 `--num_envs 4`。
3. 是否正在运行 `Go2-velocity-v0`，而不是模板任务。

当前代码已经打开了平面地形、速度命令箭头和高度扫描 debug 可视化。但论文中的 rough slope / complex terrain 还没有在当前 `go2_demo_velocity.py` 中真正实现，当前只是平面。

## 7. 观测空间

当前 policy 和 critic 观测基本一致，包含：

| 观测项 | 来源 | 说明 |
| --- | --- | --- |
| `base_lin_vel` | `mdp.base_lin_vel` | 机体坐标系下的根部线速度 |
| `base_ang_vel` | `mdp.base_ang_vel` | 机体坐标系下的根部角速度 |
| `projected_gravity` | `mdp.projected_gravity` | 重力方向在机体坐标系下的投影 |
| `velocity_commands` | `mdp.generated_commands` | `base_velocity` 速度命令 |
| `joint_pos_rel` | `mdp.joint_pos_rel` | 相对默认关节位置 |
| `joint_vel_rel` | `mdp.joint_vel_rel` | 相对关节速度 |
| `joint_effort` | `mdp.joint_effort` | 关节输出力矩 |
| `last_action` | `mdp.last_action` | 上一步动作 |
| `height_scanner` | `mdp.height_scan` | 局部高度扫描 |

当前 policy 观测打开了噪声扰动：

```text
enable_corruption = True
concatenate_terms = True
```

与论文的差异：

1. 论文描述输入包含投影重力、命令速度、关节位置、关节速度、上一时刻动作，并使用过去 30 个时间步历史。
2. 当前代码额外加入了 `base_lin_vel`、`base_ang_vel`、`joint_effort` 和 `height_scanner`。
3. 当前代码没有启用 30 步历史，`history_length` 被注释掉了。

这些差异会影响与论文结果的一致性，尤其是 sim-to-real 和步态自发涌现效果。

## 8. 动作空间

当前动作使用 Isaac Lab 的关节位置控制：

```python
JointPositionAction = mdp.JointPositionActionCfg(
    asset_name="robot",
    joint_names=[".*"],
    scale=0.25,
    use_default_offset=True,
    clip={".*": (-100.0, 100.0)},
)
```

含义：

1. 策略输出所有关节的位置目标偏移。
2. `scale=0.25` 表示网络动作会缩放后叠加到默认关节姿态。
3. `use_default_offset=True` 表示默认站姿作为动作中心。

这与论文中“策略输出 12 个关节下一时刻位置命令”的描述一致。

## 9. 命令速度与课程学习

速度命令配置：

```python
base_velocity = mdp.UniformLevelVelocityCommandCfg(
    asset_name="robot",
    resampling_time_range=(10.0, 10.0),
    rel_standing_envs=0.02,
    rel_heading_envs=1.0,
    heading_command=True,
    heading_control_stiffness=0.5,
    debug_vis=True,
    ranges=Ranges(
        lin_vel_x=(0.2, 3.0),
        lin_vel_y=(-0.3, 0.3),
        ang_vel_z=(-1.0, 1.0),
        heading=(-0, 0),
    ),
    limit_ranges=Ranges(
        lin_vel_x=(0.2, 3.0),
        lin_vel_y=(-0.3, 0.3),
        ang_vel_z=(-1.0, 1.0),
        heading=(-0, 0),
    ),
)
```

当前含义：

1. 每 10 秒重采样一次速度命令。
2. 约 2% 环境会采样站立命令。
3. x 方向命令只采样正速度 `0.2` 到 `3.0 m/s`，理论上不要求机器人倒退。
4. y 方向命令范围较小，为 `-0.3` 到 `0.3 m/s`。
5. yaw 角速度范围为 `-1.0` 到 `1.0 rad/s`。
6. `heading_command=True` 且 `rel_heading_envs=1.0`，意味着所有环境都启用 heading 控制逻辑，`ang_vel_z` 可能由 heading error 转换得到。

课程学习函数位于 `mdp/curriculums.py`：

```python
lin_vel_cmd_levels(...)
```

它会读取 `track_lin_vel_xy` 的 episode 平均奖励，如果奖励超过阈值，就扩展 `lin_vel_x` 和 `lin_vel_y` 的采样范围。

当前需要注意：

```text
ranges == limit_ranges
```

因此课程学习当前几乎没有扩展空间。若要复现论文中“速度范围逐步扩大”的 curriculum，应把 `ranges` 设置为较窄初始范围，把 `limit_ranges` 设置为更大范围。例如初始 `lin_vel_x=(0.2, 0.8)`，上限 `limit_ranges.lin_vel_x=(0.2, 3.0)`。

## 10. AER 奖励实现

### 10.1 论文奖励

论文中奖励结构为：

```text
R = (R_motion + alpha_en * R_en(vx, wz)) * exp(-R_aux)
```

核心思想是：

1. `R_motion` 负责速度跟踪。
2. `R_en` 负责鼓励单位运动距离下少消耗能量。
3. `R_aux` 负责安全和稳定性约束，例如碰撞、动作频率、躯干姿态等。
4. 通过指数形式把奖励压在合理范围，避免负无穷奖励导致训练不稳定。

### 10.2 当前代码的能量奖励

当前能量奖励位于 `mdp/rewards.py`：

```python
def energy_new_actual(env, asset_cfg=SceneEntityCfg("robot"), sigma_lin=1000.0, sigma_ang=500.0, clip_lin=0.2, clip_ang=0.2):
    joint_vel = asset.data.joint_vel
    joint_torque = asset.data.applied_torque
    energy = torch.sum(torch.abs(joint_vel * joint_torque), dim=1)
    base_lin_vel_x = asset.data.root_lin_vel_b[:, 0]
    base_ang_vel_z = asset.data.root_ang_vel_b[:, 2]
    denom = (
        sigma_lin * torch.clamp(torch.abs(base_lin_vel_x), min=clip_lin)
        + sigma_ang * torch.clamp(torch.abs(base_ang_vel_z), min=clip_ang)
    )
    return torch.exp(-energy / denom)
```

对应论文：

```text
sum_i(|tau_i| * |qdot_i|)
```

对应当前代码：

```text
torch.sum(torch.abs(joint_vel * joint_torque), dim=1)
```

对应论文：

```text
sigma_en_x * |vx| + sigma_en_z * |wz|
```

对应当前代码：

```text
1000 * clamp(|root_lin_vel_b_x|, min=0.2)
+ 500 * clamp(|root_ang_vel_b_z|, min=0.2)
```

`clip_lin` 和 `clip_ang` 是当前实现额外加入的保护，避免机器人速度接近 0 时分母太小导致奖励数值异常。

### 10.3 当前代码的 AERRewardManager

当前自定义奖励管理器位于 `aer_env.py`：

```python
class AERRewardManager(RewardManager):
    positive_terms = {"track_lin_vel_xy", "track_ang_vel_z", "energy_new_actual"}
    sigma_aux = 0.2

    def compute(self, dt):
        positive_reward = ...
        negative_reward = ...
        reward = positive_reward * torch.exp(negative_reward / self.sigma_aux) * dt
```

当前组合方式可以理解为：

```text
R_current = R_positive * exp(R_negative / sigma_aux) * dt
```

由于 `R_negative` 中各项权重是负数，指数项会把不稳定行为压低。这个写法与论文中的 `exp(-R_aux)` 思路一致，但数值尺度不是完全相同，因为当前代码把辅助项本身作为负奖励累加，再除以 `sigma_aux=0.2`。

### 10.4 当前激活的奖励项

当前 `go2_demo_velocity.py` 中激活的奖励如下：

| 奖励项 | 权重 | 作用 |
| --- | ---: | --- |
| `track_lin_vel_xy` | `15.0` | 指数形式 x/y 线速度跟踪 |
| `track_ang_vel_z` | `3.0` | 指数形式 yaw 角速度跟踪 |
| `energy_new_actual` | `0.5` | AER 能量效率奖励 |
| `lin_vel_z_l2` | `-0.05` | 惩罚 z 方向根部线速度 |
| `ang_vel_xy_l2` | `-0.001` | 惩罚 roll/pitch 角速度 |
| `flat_orientation_l2` | `-5.0` | 惩罚机身倾斜 |
| `joint_torques_l2` | `-1e-4` | 惩罚大力矩 |
| `joint_vel_l2` | `-1e-4` | 惩罚关节速度 |
| `joint_acc_l2` | `-2.5e-7` | 惩罚关节加速度 |
| `action_rate_l2` | `-6.25e-3` | 惩罚动作变化过快 |
| `feet_slip` | `-0.04` | 惩罚接触时脚底 xy 滑移 |
| `joint_pos_limits` | `-10.0` | 惩罚关节接近/超过限制 |
| `undesired_contacts` | `-5.0` | 惩罚头、髋、大腿、小腿等非脚部接触 |

当前 `feet_air_time` 只在 `rewards.py` 中定义，没有在 `RewardsCfg` 中启用。当前也没有显式步态周期、相位或接触时序奖励，这一点符合论文“无需预设步态知识”的方向。

### 10.5 关于能量奖励权重

论文强调：

1. `alpha_en = 0.0` 时没有能量正则，策略可能采用高能耗步态。
2. `alpha_en = 0.5` 时能量项影响较弱。
3. `alpha_en = 1.0` 在论文实验中通常比较合适。
4. `alpha_en = 1.5` 时高速下可能牺牲速度跟踪，甚至速度掉到 0。

当前代码中速度跟踪权重是 `15.0` 和 `3.0`，能量奖励权重是 `0.5`，所以它不等价于论文归一化表达里的 `alpha_en=0.5`。如果要做论文式消融，应记录每个实验的全部奖励权重，而不是只记录 `energy_new_actual.weight`。

推荐消融顺序：

```text
energy_new_actual.weight = 0.0
energy_new_actual.weight = 0.25
energy_new_actual.weight = 0.5
energy_new_actual.weight = 1.0
```

先确认机器人能稳定向前跟踪速度，再逐步增加能量项。

## 11. 终止条件与 reset

当前终止条件：

| 条件 | 作用 |
| --- | --- |
| `time_out` | episode 到时结束 |
| `base_contact` | base 接触地面时终止 |
| `bad_orientation` | 机身倾斜超过 `0.8 rad` 时终止 |

reset 时根部姿态随机化：

```text
x = [-0.5, 0.5]
y = [-0.5, 0.5]
yaw = [-3.14, 3.14]
root velocity = 0
```

由于 yaw 初始值随机，而命令配置又启用了 `heading_command=True` 且 `heading=(-0, 0)`，训练早期可能会有明显转向行为。若只想先验证“正向速度跟踪”，可以考虑临时关闭 heading command 或固定 yaw reset。

## 12. 仿真与训练参数

环境参数：

```text
num_envs = 4096
env_spacing = 2.5
decimation = 4
sim.dt = 0.005
policy step_dt = 0.02
episode_length_s = 20.0
max episode steps = 1000
```

RSL-RL PPO 参数：

```text
num_steps_per_env = 24
max_iterations = 50000
save_interval = 100
experiment_name = "go2_demo"
actor_hidden_dims = [512, 256, 128]
critic_hidden_dims = [512, 256, 128]
activation = "elu"
learning_rate = 1e-3
gamma = 0.99
lam = 0.95
desired_kl = 0.01
entropy_coef = 0.005
```

日志默认保存到：

```text
logs/rsl_rl/go2_demo/<时间戳>_<run_name>/
```

每次训练会保存：

```text
params/env.yaml
params/agent.yaml
model_*.pt
```

## 13. 安装与环境准备

必须使用已经安装 Isaac Lab 的 Python 环境。你当前常用环境看起来是：

```text
D:\Anaconda\envs\env_isaaclab
```

推荐 PowerShell 操作：

```powershell
conda activate env_isaaclab
cd C:\Users\Administrator\Desktop\Learning\Code\RobotProject\go2_demo
python -m pip install -e .\source\go2_demo
```

如果没有通过普通 `python` 进入 Isaac Lab 环境，o应使用 Isaac Lab 提供的 Python 启动器，例如：

```powershell
C:\Users\Administrator\Desktop\Learning\Cde\isaaclab\isaaclab.bat -p -m pip install -e .\source\go2_demo
```

常见错误：

```text
ModuleNotFoundError: No module named 'go2_demo'
```

含义是当前 Python 环境没有安装这个扩展包，或者你用错了 Python。解决方法是在同一个环境里重新执行：

```powershell
python -m pip install -e .\source\go2_demo
```

然后确认：

```powershell
python -c "import go2_demo; print(go2_demo.__file__)"
```

## 14. 训练命令

### 14.1 小规模调试训练

先用少量环境确认能正常启动、机器人可视化、命令箭头和奖励没有报错：

```powershell
python .\scripts\rsl_rl\train.py --task Go2-velocity-v0 --num_envs 32 --max_iterations 100 --run_name debug
```

如果要打开 GUI 看机器人：

```powershell
python .\scripts\rsl_rl\train.py --task Go2-velocity-v0 --num_envs 4 --max_iterations 100 --run_name gui_debug
```

如果显存不足，可以继续降低：

```powershell
python .\scripts\rsl_rl\train.py --task Go2-velocity-v0 --num_envs 1 --max_iterations 20 --run_name smoke_test
```

### 14.2 正式训练

当前默认配置是 4096 个环境和 50000 次迭代：

```powershell
python .\scripts\rsl_rl\train.py --task Go2-velocity-v0 --run_name aer_go2
```

也可以显式指定：

```powershell
python .\scripts\rsl_rl\train.py --task Go2-velocity-v0 --num_envs 4096 --max_iterations 50000 --run_name aer_go2
```

### 14.3 从 checkpoint 继续训练

```powershell
python .\scripts\rsl_rl\train.py --task Go2-velocity-v0 --resume --load_run <run_folder_name> --checkpoint <checkpoint_name>.pt
```

示例：

```powershell
python .\scripts\rsl_rl\train.py --task Go2-velocity-v0 --resume --load_run 2026-06-05_10-30-00_aer_go2 --checkpoint model_1000.pt
```

## 15. 播放和测试训练结果

播放最新 checkpoint：

```powershell
python .\scripts\rsl_rl\play.py --task Go2-velocity-v0 --num_envs 1 --real-time
```

播放指定 checkpoint：

```powershell
python .\scripts\rsl_rl\play.py --task Go2-velocity-v0 --num_envs 1 --real-time --checkpoint .\logs\rsl_rl\go2_demo\<run_folder>\model_<iter>.pt
```

录制视频：

```powershell
python .\scripts\rsl_rl\play.py --task Go2-velocity-v0 --num_envs 1 --video --video_length 500 --checkpoint .\logs\rsl_rl\go2_demo\<run_folder>\model_<iter>.pt
```

`play.py` 会自动导出策略：

```text
logs/rsl_rl/go2_demo/<run_folder>/exported/policy.pt
logs/rsl_rl/go2_demo/<run_folder>/exported/policy.onnx
```

## 16. 论文复现实验建议

### 16.1 实验 A：无能量正则 baseline

目的：观察无 AER 时机器人是否能跟踪速度，以及是否出现高能耗、不自然抖动、弹跳或倒退。

建议设置：

```text
energy_new_actual.weight = 0.0
run_name = no_energy
```

可通过代码修改或 Hydra override 完成：

```powershell
python .\scripts\rsl_rl\train.py --task Go2-velocity-v0 --num_envs 4096 --max_iterations 50000 --run_name no_energy rewards.energy_new_actual.weight=0.0
```

如果当前 Isaac Lab/Hydra 版本不接受该 override，就直接在 `go2_demo_velocity.py` 中把 `energy_new_actual` 的 `weight` 改为 `0.0`。

### 16.2 实验 B：逐步增加能量正则

目的：复现论文中“能量权重需要适中”的结论。

推荐训练：

```powershell
python .\scripts\rsl_rl\train.py --task Go2-velocity-v0 --num_envs 4096 --max_iterations 50000 --run_name energy_025 rewards.energy_new_actual.weight=0.25
python .\scripts\rsl_rl\train.py --task Go2-velocity-v0 --num_envs 4096 --max_iterations 50000 --run_name energy_050 rewards.energy_new_actual.weight=0.5
python .\scripts\rsl_rl\train.py --task Go2-velocity-v0 --num_envs 4096 --max_iterations 50000 --run_name energy_100 rewards.energy_new_actual.weight=1.0
```

记录指标：

```text
平均 episode reward
track_lin_vel_xy
track_ang_vel_z
energy_new_actual
真实 vx 与 command vx 的误差
真实 wz 与 command wz 的误差
sum(|tau| * |qdot|)
脚底接触序列
```

判断标准：

1. 如果速度跟踪变差或机器人停住，能量项过强或辅助惩罚门控过强。
2. 如果能量下降不明显，能量项过弱或动作/力矩惩罚已经主导。
3. 如果机器人抖动明显，优先检查动作变化惩罚、关节 PD 参数、接触/脚滑惩罚和 reset 姿态。

### 16.3 实验 C：不同速度下步态观察

论文中重要现象是不同速度下自动切换步态。当前项目可以先在播放时观察随机命令，也可以固定速度范围进行训练或测试。

建议测试速度：

```text
0.3 m/s
0.5 m/s
1.0 m/s
1.5 m/s
2.0 m/s
2.5 m/s
```

观察内容：

```text
是否向命令箭头方向前进
机身是否稳定
四只脚接触顺序是否随速度变化
是否出现弹跳、拖脚、倒退、原地抖动
真实速度是否接近 command
```

### 16.4 实验 D：圆轨迹跟踪

论文中圆轨迹测试使用：

```text
linear velocity = 1.0 m/s
angular velocity = 0.5 rad/s
target radius = 2 m
```

当前代码还没有专门的圆轨迹评估脚本，但可以通过固定命令速度并记录 robot root pose 实现。指标为：

```text
e(t) = |d(t) - 2|
sum_t e(t)
```

其中 `d(t)` 是机器人位置到圆心的距离。

### 16.5 实验 E：地形实验

论文包含 rough slope 和 20 cm 台阶等地形测试。当前 `go2_demo_velocity.py` 使用的是 `terrain_type="plane"`，因此目前只能复现平地版本。

若要复现地形实验，需要进一步添加：

1. Isaac Lab terrain generator，例如 rough slope / random rough terrain。
2. 更完整的 terrain curriculum。
3. 高度扫描输入与地形 mesh 正确绑定。
4. 地形难度、摩擦、质量扰动等 domain randomization。

## 17. 当前代码与论文的主要差异

| 项目 | 论文 | 当前代码 |
| --- | --- | --- |
| 机器人 | Unitree Go1 | Unitree Go2 |
| 仿真框架 | IsaacGym / legged-gym 体系 | Isaac Lab Manager-Based RL |
| RL 算法 | PPO | RSL-RL PPO |
| 动作 | 12 关节位置命令 | 12 关节位置命令 |
| 能量常数 | `sigma_en_x=1000`, `sigma_en_z=500` | 相同 |
| 能量公式 | `exp(-power / generalized_distance)` | 相同思路，额外加速度 clamp |
| 输入历史 | 过去 30 步 | 当前未启用 |
| 地形 | 平地 + rough slope | 当前为平面 |
| domain randomization | 摩擦、质量、观测噪声等 | 当前主要是观测噪声，摩擦/质量随机化未完整实现 |
| 命令 curriculum | 从小范围逐步扩到大范围 | 当前 `ranges` 与 `limit_ranges` 一样，扩展效果有限 |
| 步态奖励 | 无显式步态奖励 | 无显式步态奖励 |
| 速度范围 | 论文可到更宽正负范围 | 当前 x 为 `0.2` 到 `3.0`，不含负向 x |

## 18. 关于“机器人倒退、抖动、最后不动”的排查建议

你之前遇到的问题和论文中的现象有重叠，也有可能来自实现细节。建议按以下顺序排查。

### 18.1 确认坐标系方向

当前命令 `lin_vel_x=(0.2, 3.0)` 是正向 x 速度。如果机器人总是“倒退”去跟踪，最优先检查：

1. Go2 USD 模型的 base 朝向是否与 Isaac Lab 约定的 body x 轴一致。
2. `root_lin_vel_b[:, 0]` 的正方向是否等于机器人视觉上的前方。
3. Go2 关节命名和默认姿态是否导致前后腿配置视觉上反了。

一个简单判断方法是播放时看命令箭头方向和机器人 base 坐标轴方向是否一致。

### 18.2 暂时关闭 heading_command

当前：

```text
heading_command=True
rel_heading_envs=1.0
heading=(-0, 0)
reset yaw 随机 [-3.14, 3.14]
```

这会让机器人在训练早期同时处理“向前速度”和“转到目标 heading”。如果想先验证前进速度跟踪，可以临时设为：

```text
heading_command=False
rel_heading_envs=0.0
reset yaw=(0.0, 0.0)
```

确认直线前进正常后，再恢复 heading/yaw 随机化。

### 18.3 先移除能量项，再逐步加回

论文明确指出能量权重过大可能导致不动。当前建议：

```text
energy_new_actual.weight = 0.0
```

先确认 `track_lin_vel_xy` 能让机器人动起来，再尝试：

```text
0.25 -> 0.5 -> 1.0
```

### 18.4 检查 AER 指数门控是否过强

当前：

```text
reward = positive_reward * exp(negative_reward / 0.2) * dt
```

如果负奖励项数值稍大，`exp(negative_reward / 0.2)` 会迅速接近 0，导致正向速度奖励被严重压低，策略可能学到“少动少错”。如果出现训练后期不动，可以尝试：

```text
sigma_aux = 0.5
sigma_aux = 1.0
```

或者先临时退回 Isaac Lab 默认的加和式 reward manager，确认基础速度跟踪没问题。

### 18.5 先关掉过强惩罚

当前比较强的惩罚包括：

```text
flat_orientation_l2 = -5.0
dof_pos_limits = -10.0
undesired_contacts = -5.0
```

如果机器人训练初期经常接触终止或 reward 被门控归零，可以先降低这些权重，等能稳定前进后再逐步加回。

### 18.6 检查 actuator 参数

当前 Go2 电机参数：

```text
stiffness = 25.0
damping = 0.5
effort_limit = 23.5
velocity_limit = 30.0
```

如果机器人抖动明显，可能需要调大 damping、降低 action scale、增加 action_rate 惩罚或检查 USD inertial/contact 参数。

## 19. 推荐复现路线

为了避免同时引入太多变量，推荐按下面顺序推进：

1. `num_envs=1` GUI smoke test，确认模型、地面、命令箭头、接触传感器能启动。
2. `energy_new_actual.weight=0.0`，只训练速度跟踪和基础稳定项。
3. 固定 yaw reset，关闭 heading command，确认机器人视觉上向前走。
4. 恢复 yaw 随机和 heading command。
5. 加回 `energy_new_actual.weight=0.25`。
6. 训练到足够迭代后观察步态和能耗。
7. 尝试 `0.5`、`1.0` 的能量权重做消融。
8. 修正 curriculum 初始范围，使速度命令从易到难扩大。
9. 加入 terrain generator 和 domain randomization，复现论文地形实验。

## 20. 最小可运行命令汇总

安装：

```powershell
conda activate env_isaaclab
cd C:\Users\Administrator\Desktop\Learning\Code\RobotProject\go2_demo
python -m pip install -e .\source\go2_demo
```

调试训练：

```powershell
python .\scripts\rsl_rl\train.py --task Go2-velocity-v0 --num_envs 32 --max_iterations 100 --run_name debug
```

正式训练：

```powershell
python .\scripts\rsl_rl\train.py --task Go2-velocity-v0 --num_envs 4096 --max_iterations 50000 --run_name aer_go2
```

播放：

```powershell
python .\scripts\rsl_rl\play.py --task Go2-velocity-v0 --num_envs 1 --real-time
```

播放指定模型：

```powershell
python .\scripts\rsl_rl\play.py --task Go2-velocity-v0 --num_envs 1 --real-time --checkpoint .\logs\rsl_rl\go2_demo\<run_folder>\model_<iter>.pt
```

无能量消融：

```powershell
python .\scripts\rsl_rl\train.py --task Go2-velocity-v0 --num_envs 4096 --max_iterations 50000 --run_name no_energy rewards.energy_new_actual.weight=0.0
```

小权重能量消融：

```powershell
python .\scripts\rsl_rl\train.py --task Go2-velocity-v0 --num_envs 4096 --max_iterations 50000 --run_name energy_025 rewards.energy_new_actual.weight=0.25
```

## 21. 当前仓库状态下的结论

当前项目已经具备复现论文 AER 思路的关键骨架：

1. Go2 模型已接入 Isaac Lab。
2. `Go2-velocity-v0` 任务已注册。
3. 速度命令、命令箭头 debug 可视化、高度扫描、接触传感器已配置。
4. 论文核心能量奖励 `exp(-power / generalized_distance)` 已实现。
5. 自定义 `AERRewardManager` 已实现正奖励与辅助惩罚的指数门控。
6. RSL-RL PPO 训练和播放脚本已具备。

但它还不是论文完整复现，主要缺口是：

1. Go1 原模型和论文原始训练框架没有完全一致。
2. 30 步历史观测未启用。
3. 课程学习初始范围和上限范围目前相同，难以体现逐步扩展。
4. 平地可跑，rough terrain 还未实现。
5. domain randomization 还不完整。
6. 机器人倒退/抖动/不动问题还需要从坐标系、heading command、AER 门控强度和奖励权重逐步排查。

因此，最稳妥的复现策略是先完成平地正向速度跟踪，再逐步加入 AER、curriculum、domain randomization 和 terrain，而不是一开始就追求论文全部结果。
