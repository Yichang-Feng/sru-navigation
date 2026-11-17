# sru_ws — 狗机 `~/sru_ws/` 工作空间级文件快照

> 对应机载路径：`agile@192.168.123.111:~/sru_ws/`（局部 SRU 模式工作空间）。
> 本目录只保存**工作空间根目录**的文件，用于核对狗上实际运行的脚本与配置。

## 为什么只有这些文件

狗上的 `~/sru_ws/` 结构是：

```
~/sru_ws/
├── src/sru-robot-deployment/   ← 本仓库（sru-robot-deployment）在狗上的旧 git 克隆
├── model/                      ← ONNX 权重，不进 git
├── map/                        ← 预留地图目录，不进 git
├── build/ install/ log/        ← colcon 产物，不进 git
├── benchmark/                  ← 性能基准套件
└── start_local_sru.sh  stop_local_sru.sh  *.rviz  *_nav_override.yaml  测试脚本
```

因此这里**不镜像** `src/`（那是本仓库自己的旧克隆）、`model/`、`map/`、`build/`、`install/`、`log/`。

## 与 `../scripts/` 的关系

- `../scripts/` 是**同步源**：`scripts/sync_to_go2w_nav.sh` 会把 `scripts/start_local_sru.sh`、
  `scripts/stop_local_sru.sh`、`scripts/realsense_nav_override.yaml` 推到狗上的 `~/sru_ws/`。
- 本目录是**狗上快照**，用于核对"狗上现在到底是什么"。需要改行为时改 `../scripts/` 再同步，
  不要只改这里，否则又会和狗上不一致。
- 目前 `start_local_sru.sh` / `stop_local_sru.sh` / `realsense_nav_override.yaml` 与 `../scripts/` 一致；
  `zed_nav_override.yaml` 是局部模式专用的一份，与 `rl_nav_controller/config/zed_nav_override.yaml` 不同，
  以 `start_local_sru.sh --zed` 实际加载的那份为准。

## 启动

```bash
cd ~/sru_ws
./start_local_sru.sh          # 默认 RealSense D435/D435i
./start_local_sru.sh --zed    # 切换 ZED 2i
./stop_local_sru.sh
```

局部模式下 `start_local_sru.sh` 除启动 `super_lio map_localization.py` 外，还会广播一条
**静态 `map -> world` 恒等 TF**，所以 RViz 的 `Fixed Frame: map` 一开始就存在；
这条静态 TF 与动态 `map -> world` 会互相拉扯，只适用于局部模式（全局模式的 `start_nav.sh`
不广播它，必须先给 `/initialpose`）。
