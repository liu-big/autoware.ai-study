# Autoware.ai 实车自动驾驶软件栈（Ant 定制版）

> 基于 [Autoware.ai](https://github.com/Autoware-AI/autoware.ai) 的低速自动驾驶完整软件系统。  
> 在开源 Autoware 之上完成 **传感接入 → SLAM 建图 → NDT 定位 → 感知 → 规划 → 控制 → CAN 线控** 全链路实车闭环。

**维护者：** [liu-big](https://github.com/liu-big)（2715209705@qq.com）  
**仓库属性：** 私有（Private），仅所有者可访问与贡献，不添加外部 Collaborator  
**目标仓库：** https://github.com/liu-big/autoware.ai-study  
**构建方式：** colcon（ROS Melodic）  
**主语言：** C++ / Python / Launch / YAML

---

## 目录

1. [项目简介](#1-项目简介)
2. [系统架构](#2-系统架构)
3. [目录结构](#3-目录结构)
4. [技术栈与依赖](#4-技术栈与依赖)
5. [定制模块详解（src/ant）](#5-定制模块详解srcant)
6. [Autoware 开源模块（本仓库使用到的）](#6-autoware-开源模块本仓库使用到的)
7. [数据流与关键话题](#7-数据流与关键话题)
8. [配置体系（地图 / 车型）](#8-配置体系地图--车型)
9. [编译与运行](#9-编译与运行)
10. [建图与 SLAM](#10-建图与-slam)
11. [多激光雷达接入说明](#11-多激光雷达接入说明)
12. [关键参数说明](#12-关键参数说明)
13. [排障速查](#13-排障速查)
14. [许可证说明](#14-许可证说明)

---

## 1. 项目简介

本仓库是一套可在 **实车上运行** 的自动驾驶软件工作空间，核心目标是：

| 能力 | 说明 |
|------|------|
| 传感 | 激光雷达（雷神 LSLidar C16 / 速腾 RoboSense）、GNSS、IMU、超声波 |
| 建图 | LIO-SAM-6AXIS（激光+IMU+GPS 因子图）或 Autoware NDT Mapping |
| 定位 | NDT Matching + GNSS 初值/辅助 |
| 感知 | 地面滤波、欧氏聚类、轮廓跟踪、安全距离限速、YOLO、红绿灯 |
| 规划 | OpenPlanner 全局路径 + 局部 rollout 轨迹 |
| 控制 | Pure Pursuit 路径跟踪 + twist_filter |
| 执行 | CAN 盒抽象 + 多车型线控协议（五菱/中云/泛太/埃安等） |

上层算法大量复用 Autoware.ai；**工程化配置、传感驱动、安全模块、多车型 CAN、LIO-SAM 建图适配** 为 `src/ant/` 定制内容。

### 1.1 一句话定位

> 离线用 SLAM 建 PCD 地图 → 在线用 NDT 定位 → OpenPlanner 规划 → Pure Pursuit + CAN 控车。

---

## 2. 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        startingup_ros                            │
│         （地图 / 车型 / Common 配置中心 + 一键启动链）              │
└────────────────────────────┬────────────────────────────────────┘
                             │
     ┌───────────┬───────────┼───────────┬───────────┬────────────┐
     ▼           ▼           ▼           ▼           ▼            ▼
  Sensing    Mapping     Localization  Detection   Planning    Control
  雷达/GNSS   LIO-SAM      NDT Match     聚类/跟踪    OP全局/局部  Pure Pursuit
  IMU/融合    NDT Map      GNSS初值      safe_range   航点生成     twist_filter
     │           │           │           │           │            │
     └───────────┴───────────┴───────────┴───────────┴─────┬──────┘
                                                           ▼
                                                    can_connect
                                                    can_client
                                                      底盘 CAN
```

### 2.1 启动链路

```
startingup.launch
  └── startingup_navigation.launch
        ├── my_map.launch              # 点云图 + 矢量地图 + TF
        ├── my_sensing.launch          # 传感器（实车按车型配置）
        ├── my_localization.launch     # 降采样 + GNSS + NDT
        ├── my_detection.launch        # 感知 + 安全
        ├── my_mission_planning.launch # OpenPlanner
        ├── my_motion_planning.launch  # Pure Pursuit
        └── RViz
```

实车执行层（线控）另启：

```
my_auto.launch
  ├── can_client.launch   # 车型协议 + 超声波/毫米波
  └── twist_filter        # 横向动力学限幅 → vehicle_cmd
```

---

## 3. 目录结构

```
autoware.ai/
├── README.md                 # 本说明（中文）
├── make.sh                   # colcon 编译脚本（支持 CUDA）
├── select_map.sh             # 选择地图 → 拷贝到 Common
├── select_vehicle.sh         # 选择车型 → 拷贝到 Common
├── ui.sh                     # 启动 driverless_ui（可选）
├── src/
│   ├── ant/                  # ★ 本项目定制与集成包（重点）
│   ├── autoware/             # Autoware.ai 开源核心
│   ├── drivers/              # 官方/社区驱动
│   └── car_demo/             # Gazebo Prius 演示（可选）
├── build/                    # 编译中间产物（不上传）
└── install/                  # 安装空间（不上传）
```

### 3.1 `src/ant/` 定制包一览

| 包名 | 作用 |
|------|------|
| `startingup_ros` | 配置中心、地图/车型数据、启动 launch、参数 |
| `lslidar_c16` | 雷神 C16 驱动 + 解码（单/双/多雷达） |
| `rslidar_sdk` | 速腾雷达 SDK |
| `nmea_navsat_driver` | GNSS NMEA 串口/网络驱动 |
| `lio_sam_6axis` | 6 轴 IMU 版 LIO-SAM + GPS 因子建图 |
| `lidar_obstacle_detection` | 障碍检测、多雷达融合、安全距离 `safe_range` |
| `can_connect` | CAN 盒抽象（SocketCAN 等） |
| `can_client` | 多车型底盘协议、超声波、毫米波 |
| `gnss_origin_pose` | GNSS 原点/坐标相关 |
| `site_pose` / `send_sitedata` | 站点任务数据 |
| `lane_follow` | 视觉车道跟随相关 |
| `driverless_ui` | 无人驾驶操作界面（可选） |
| `ui_bridge` | UI 与 ROS 动态库桥接（可选） |
| `remote_communication` | 远程通信示例 |
| `ros-bridge` | CARLA 仿真桥（可选，实车可不启用） |
| `cv_bridge_1` / `vision_opencv` | 视觉桥接依赖 |

### 3.2 `src/autoware/` 主要子目录

| 目录 | 内容 |
|------|------|
| `core_perception` | 定位、点云预处理、聚类跟踪、视觉检测、红绿灯 |
| `core_planning` | OpenPlanner、Pure Pursuit、twist_filter、航点 |
| `messages` | Autoware 消息定义 |
| `utilities` | 标定、地图工具、launcher 等 |
| `visualization` | 车辆模型、可视化 |
| `common` / `simulation` / `documentation` | 公共库、仿真、文档 |

---

## 4. 技术栈与依赖

### 4.1 基础环境

| 项目 | 建议版本 |
|------|----------|
| OS | Ubuntu 18.04 |
| ROS | Melodic |
| 构建 | colcon |
| CUDA | 可选（NDT GPU / YOLO / 聚类加速） |
| 语言 | C++11+、Python 2/3（视节点而定） |

### 4.2 系统依赖（示例）

```bash
sudo apt-get install \
  ros-melodic-jsk-recognition-msgs \
  ros-melodic-jsk-recognition \
  ros-melodic-jsk-rviz-plugins \
  libpcap-dev
```

### 4.3 LIO-SAM 额外依赖

- **GTSAM 4.0.2**（因子图优化，版本需匹配）
- 若遇 FLANN / LZ4 头文件冲突，参考 `src/ant/lio_sam_6axis/README.md`

### 4.4 算法与中间件

| 类别 | 技术 |
|------|------|
| 中间件 | ROS Melodic、TF、message_filters |
| 点云 | PCL |
| 视觉 | OpenCV、Darknet YOLO |
| SLAM | LIO-SAM（GTSAM）、NDT Mapping |
| 定位 | NDT Matching、GNSS（nmea2tfpose / fix2tfpose） |
| 规划 | OpenPlanner（op_global / op_local） |
| 控制 | Pure Pursuit、twist_filter / twist_gate |
| 底盘 | SocketCAN、多车型 CAN 协议、模糊 PID |

---

## 5. 定制模块详解（`src/ant`）

### 5.1 `startingup_ros` — 配置与启动中枢

- **Map**：各地图的 PCD、矢量地图 CSV、GNSS 初值等  
- **Vehicle**：车型参数、TF、标定、覆盖用 launch  
- **Common**：当前生效配置（由 `select_map.sh` / `select_vehicle.sh` 拷贝生成）

环境变量：

| 变量 | 含义 |
|------|------|
| `AUTOCAR_MAP_TYPE` | 当前地图名（如 `tonglu`、`sim`） |
| `AUTOCAR_VEHICLE_TYPE` | 当前车型名（如 `sim`、实车名） |

已有地图示例：`tonglu`、`beijing`、`meixun`、`mitehua`、`sanfrancisco`、`sanxiang`、`sim`、`Town01`~`Town07` 等。

### 5.2 传感驱动

#### LSLidar C16（`lslidar_c16`）

```
UDP 原始包 → driver_node → decoder_node → PointCloud2
```

- 单雷达：`lslidar_c16.launch` → 默认 `/points_raw`，frame 常为 `velodyne`
- 多雷达：`lslidar_c16_mult.launch` / `lslidar_c16_double.launch`  
  - 不同 IP、MSOP/DIFOP 端口、namespace、独立 frame_id  
  - 静态 TF：`base_link → laser*_link`

#### 速腾（`rslidar_sdk`）

可与 LSLidar 混用（多雷达 launch 中有 RSHELIOS 注释示例）。

#### GNSS（`nmea_navsat_driver`）

- 默认串口：`/dev/gnss`
- 波特率示例：`230400`
- 输出：`/nmea_sentence` 等

### 5.3 `lidar_obstacle_detection` — 融合与安全

| 节点/功能 | 说明 |
|-----------|------|
| `lidar_fusion` | 多雷达近似时间同步 + 外参变换 + 角度裁剪 → `/points_raw` |
| `obstacle_detection` | 点云聚类、车体框、停车框可视化 |
| `safe_range` | 订阅预测障碍物/超声波/速度，发布 `safe_range_ctrl` 限速 |

安全距离示例（`config.yaml`）：前 1.0 m / 后 0.75 m / 左右 0.22 m，并按速度系数放大。

### 5.4 `lio_sam_6axis` — 激光惯性 SLAM 建图

输入：

- `/points_raw`
- `/imu_raw`
- GPS（经 navsat → `odometry/gpsz`）

流水线：

```
imageProjection → featureExtraction → imuPreintegration → mapOptimization(+GPS)
```

输出：全局 PCD、优化轨迹；地图保存到 `startingup_ros/data/Common/map/`。

启动入口：`build_pcd_mapLio.launch`。

### 5.5 `can_connect` + `can_client` — 线控底盘

```
vehicle_cmd
    → vehicle_core（按 AUTOCAR_VEHICLE_TYPE 选车型类）
    → can_box_cmd
    → can_connect（SocketCAN / 物理 CAN）
    → 底盘

can_box_raw → 车型类解析 → can_info / real_velocity
safe_range_ctrl → 车型层速度比例裁剪
```

已适配车型类示例：`wuling`、`zhongyun`、`kaiwo`、`yunle`、`fantai`、`aian`、`yuhesen`、`sim` 等。  
另含超声波、毫米波 CAN 节点。

---

## 6. Autoware 开源模块（本仓库使用到的）

### 6.1 感知 / 定位（`core_perception`）

| 包 | 用途 |
|----|------|
| `points_downsampler` | 点云体素降采样 |
| `points_preprocessor` | 地面滤波（ray_ground_filter）等 |
| `lidar_localizer` / `ndt_*` | NDT 匹配 / 建图 |
| `gnss_localizer` | nmea2tfpose / fix2tfpose |
| `lidar_euclidean_cluster_detect` | 欧氏聚类 |
| `lidar_kf_contour_track` | 轮廓跟踪 |
| `vision_darknet_detect` | YOLOv3 目标检测 |
| `trafficlight_recognizer` | 红绿灯识别 |
| `vision_lane_detect` | 车道线检测 |
| `autoware_connector` | vel_pose_connect（位姿速度汇合） |

### 6.2 规划 / 控制（`core_planning`）

| 包 | 用途 |
|----|------|
| `op_global_planner` | 全局路径 |
| `op_local_planner` | 局部轨迹生成 / 评价 / 行为选择 |
| `pure_pursuit` | 路径跟踪 |
| `twist_filter` / `twist_gate` | 指令滤波与门控 |
| `waypoint_maker` | 航点可视化等 |

---

## 7. 数据流与关键话题

### 7.1 实车主链路

```
雷达 → /points_raw (frame: velodyne)
GNSS → /nmea_sentence 或 /gps/fix
IMU  → /imu_raw

/points_raw
  → voxel_grid_filter
  → ndt_matching → /ndt_pose, /estimate_twist
  → vel_pose_connect → /current_pose, /current_velocity

/points_raw
  → ray_ground_filter → /points_no_ground
  → euclidean_cluster → kf_contour_track → /predicted_objects
  → safe_range → safe_range_ctrl

矢量地图 + 目标点 + /current_pose
  → op_global_planner → op_local_planner → 局部轨迹
  → pure_pursuit → twist_filter → /vehicle_cmd
  → can_client → 底盘
```

### 7.2 建图录包常用话题

```
/points_raw
/imu_raw
/nmea_sentence
```

对应 launch：`startingup_ros/.../record_ros_bag.launch`。

### 7.3 TF 示意

```
world → map → mobility → base_link
                              ├── velodyne   （主雷达 / 融合参考系）
                              ├── laser*_link（各物理雷达）
                              └── gps
```

`sensor_tf.launch` 示例外参：`base_link → velodyne = (1.2, 0, 2.0)`。

---

## 8. 配置体系（地图 / 车型）

### 8.1 选地图

```bash
source install/setup.bash   # 或对应 devel/setup
./select_map.sh
```

脚本会：

1. 列出 `startingup_ros/data/Map/` 下地图  
2. 拷贝选中地图到 `data/Common/`  
3. 写入 `~/.bashrc` 中 `AUTOCAR_MAP_TYPE`

### 8.2 选车型

```bash
./select_vehicle.sh
```

同理设置 `AUTOCAR_VEHICLE_TYPE`，并拷贝车型 TF、标定、launch 覆盖到 Common。

### 8.3 参数总表

主配置：`src/ant/startingup_ros/data/Common/config/config.yaml`

包含：GNSS/NDT、安全距离、聚类、车辆尺寸、OP 规划限速、Pure Pursuit、CAN 刹车增益等。

---

## 9. 编译与运行

### 9.1 编译

```bash
cd ~/autoware.ai   # 工作空间根目录
chmod +x make.sh
./make.sh          # 全量编译（默认开启 CUDA）
# 或
./make.sh can_client lio_sam_6axis   # 按包编译
```

等价于：

```bash
AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 9.2 环境

```bash
source /opt/ros/melodic/setup.bash
source install/setup.bash
```

### 9.3 运行导航栈

```bash
roslaunch startingup_ros startingup.launch
# 或
roslaunch startingup_ros startingup_navigation.launch
```

### 9.4 运行线控（实车）

```bash
roslaunch startingup_ros ...   # 先保证 Common 下 my_auto.launch 可用
# 典型入口：
roslaunch can_client can_client.launch
# 或 include Common/custom_launch/my_auto.launch
```

### 9.5 建图

```bash
# NDT Mapping
roslaunch startingup_ros ... build_pcd_map.launch
# 实际路径：
roslaunch startingup_ros  # 若已安装；或直接
# src/ant/startingup_ros/data/Common/custom_launch/map/build_pcd_map.launch

# LIO-SAM
# .../map/build_pcd_mapLio.launch
```

---

## 10. 建图与 SLAM

| 方案 | 入口 | 特点 | 适用 |
|------|------|------|------|
| **LIO-SAM-6AXIS** | `build_pcd_mapLio.launch` | 特征+IMU 预积分+GTSAM+回环+GPS 因子 | 长距离、有闭环、起伏地形 |
| **NDT Mapping** | `build_pcd_map.launch` | 逐帧 NDT 堆图，实现简单 | 短距离、平坦、低速 |
| **NDT Matching** | `ndt_matching.launch` | **不是建图**，是在线定位 | 日常自动驾驶 |

工程策略：

> **离线 SLAM 建图 → 在线 NDT 定位**，保证运行时算力与稳定性。

LIO-SAM 注意：

- 6 轴 IMU 需正确外参与 `imuConverter` 适配  
- 隧道等 GPS 不可信场景应提高 `gpsCovThreshold` 或关闭 GPS 因子，避免错误约束  

---

## 11. 多激光雷达接入说明

（整理自 `src/ant/readme.md` 实车经验）

1. `convert.cc` 统一时间戳：`outMsg.header.stamp = ros::Time::now();`  
2. 各雷达独立 `frame_id`（如 `laser1_link` / `laser2_link`）  
3. 独立 IP（如 `192.168.86.200` / `192.168.86.202`）  
4. 静态 TF：`base_link → laser*_link`  
5. 使用 `<group ns="lidar16_x">` 避免节点名冲突  
6. 参数名注意 `device_ip` / `lidar_ip`  
7. **每台 MSOP/DIFOP 端口必须不同**（如 2368/2369 与 2370/2371）  
8. 融合后统一 `/points_raw`，下游 frame 对齐 `velodyne`

---

## 12. 关键参数说明

摘自 `config.yaml`（可按实车标定修改）：

| 模块 | 参数 | 典型值 |
|------|------|--------|
| 车辆 | 宽 / 长 / 轴距 | 1.85 / 4.2 / 2.7 m |
| 规划 | maxVelocity | 3 m/s |
| 规划 | rollOutsNumber | 6 |
| 规划 | maxLocalPlanDistance | 80 m |
| 控制 | minimum_lookahead_distance | 6.2 m |
| 控制 | lookahead_ratio | 2.5 |
| 安全 | 前后左右安全距离 | 1.0 / 0.75 / 0.22 / 0.22 m |
| 定位 | use_gnss / use_imu | 1 / 1 |
| CAN | stop / normal 刹车增益 | 10 / 2.0（车型不同需重调） |

---

## 13. 排障速查

| 现象 | 优先检查 |
|------|----------|
| 无 `/points_raw` | 雷达 IP/端口、网卡、`rosnode`、防火墙、libpcap |
| 定位飘 / 失败 | 地图是否加载、初值、TF `base_link↔velodyne`、点云质量 |
| 多雷达重影 | 时间戳、外参、角度裁剪、融合同步 |
| 不跟线 | `/current_pose`、规划是否出轨迹、Pure Pursuit 前视距离 |
| 车不动 | `vehicle_cmd`、CAN 盒、车型名、急停/安全限速是否为 0 |
| LIO-SAM 炸 | GTSAM 版本、IMU 外参、GPS 协方差阈值、特征是否充足 |

常用命令：

```bash
rostopic list
rostopic hz /points_raw
rostopic echo /current_pose -n1
rosrun tf view_frames
rosnode list
```

---

## 14. 许可证说明

- Autoware.ai 及相关开源包遵循各自上游许可证（多为 Apache-2.0 / BSD 等）。  
- `src/ant/` 定制代码请按团队约定使用；本仓库为 **私有仓库**，未经授权请勿外传。  
- 第三方驱动（雷神、速腾、CARLA ros-bridge 等）遵循其原作者许可。

---

## 附录 A：根目录脚本

| 脚本 | 功能 |
|------|------|
| `make.sh` | colcon Release 编译；可传包名选择性编译 |
| `select_map.sh` | 交互选择地图并写入环境变量 |
| `select_vehicle.sh` | 交互选择车型并写入环境变量 |
| `ui.sh` | 清理 ROS log 并启动 `driverless_ui` |

## 附录 B：仓库贡献说明

- 本仓库为 **Private**，仅所有者 `2715209705@qq.com` 对应 GitHub 账号可访问。  
- 不接受外部 Collaborator；如需共享请由所有者主动添加。  

---

**文档版本：** 2026-09  
**对应工作空间：** `autoware.ai`（Ant 定制实车栈）
