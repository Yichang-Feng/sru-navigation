# sru_ws — 机载 `~/sru_ws/` 的规范副本

> 对应机载路径：`agile@192.168.123.111:~/sru_ws/`（局部 SRU 模式工作空间）。
> 本目录保存**工作空间根目录**的文件（启动脚本 / 相机配置 / RViz / 测试与基准脚本），是部署到狗上的权威来源。

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

## 文件定位

- 本目录是机载 `~/sru_ws/` 的**规范副本**：`start_local_sru.sh` / `stop_local_sru.sh` /
  `realsense_nav_override.yaml` 以这里为准，部署时直接同步到机载（拉取/编译/部署步骤见
  [docs/VLN_BASE_FRAME_GOALS.md](../docs/VLN_BASE_FRAME_GOALS.md) 第 5 节）。
- `zed_nav_override.yaml` 是局部模式专用的一份，与 `rl_nav_controller/config/zed_nav_override.yaml` 不同；
  本项目默认 RealSense、不涉及 ZED。

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
