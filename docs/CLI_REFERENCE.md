# CLI 常用命令速查

## TensorBoard（查看训练曲线）

```bash
# 查看所有 RL 训练日志（推荐）
tensorboard --logdir IsaacLab/logs/rsl_rl --port 6006

# 只看某个任务的曲线，例如 MDPo
tensorboard --logdir IsaacLab/logs/rsl_rl/go2w_navigation_mdpo --port 6006

# 只看某一次具体 run
tensorboard --logdir IsaacLab/logs/rsl_rl/go2w_navigation_mdpo/2026-10-22_09-33-21_stage1_flat --port 6006
```

打开浏览器访问 `http://localhost:6006`。

### 端口被占用

```bash
tensorboard --logdir IsaacLab/logs/rsl_rl --port 6007
```

### 远程机器（SSH）

```bash
# 本地终端做端口转发
ssh -L 6006:localhost:6006 user@remote-host

# 然后本地浏览器打开 http://localhost:6006
```

### 后台运行 + 指定 host（允许局域网访问）

```bash
tensorboard --logdir IsaacLab/logs/rsl_rl --host 0.0.0.0 --port 6006 &
```

## 日志目录结构

```
IsaacLab/logs/rsl_rl/
├── b2w_navigation_ppo/
│   └── 2026-10-19_14-16-10/
│       └── events.out.tfevents.*
├── go2w_navigation_mdpo/
│   ├── 2026-10-21_11-10-00/
│   └── 2026-10-22_09-33-21_stage1_flat/
└── go2w_navigation_finetune/
    ├── 2026-10-21_16-17-46_from_nav_policy_new/
    └── 2026-10-21_16-46-38_warmup_ppo/
```

每个子目录对应一次训练 run，文件夹名是 `YYYY-MM-DD_HH-MM-SS` 时间戳。

## WandB（如果启用了）

```bash
# 查看 wandb 项目
wandb login
wandb offline   # 离线模式，不上传

# 本地查看 wandb 日志（无需联网）
ls IsaacLab/wandb/
```

## 训练相关

```bash
# 激活 conda 环境
conda activate sru_nav_learning

# 进入 IsaacLab 目录
cd IsaacLab

# 运行导航训练脚本（示例，具体参数看各任务 README）
python scripts/rsl_rl/train.py --task Go2W-Navigation-MDPo
```

## 其他常用

```bash
# 查看 GPU 占用
nvidia-smi

# 杀掉占用的 tensorboard 进程
pkill -f tensorboard

# 查看某端口被谁占用
lsof -i :6006
```
