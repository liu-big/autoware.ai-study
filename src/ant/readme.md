# Ant 定制包说明（`src/ant`）

> 完整项目说明见仓库根目录 [README.md](../../README.md)。  
> 本文档补充 **实车定制层** 的模块清单与多雷达接入细节。

维护者：2715209705@qq.com

---

## 1. 本目录有什么

| 包 / 目录 | 类型 | 说明 |
|-----------|------|------|
| `startingup_ros` | 核心 | 地图/车型/Common 配置、启动 launch、参数 |
| `lslidar_c16` | 驱动 | 雷神 C16 driver + decoder |
| `rslidar_sdk` | 驱动 | 速腾雷达 SDK |
| `nmea_navsat_driver` | 驱动 | GNSS NMEA |
| `lio_sam_6axis` | SLAM | 6 轴 IMU + GPS 因子 LIO-SAM 建图 |
| `lidar_obstacle_detection` | 感知 | 多雷达融合、障碍检测、`safe_range` |
| `can_connect` | 底盘 | CAN 盒收发抽象 |
| `can_client` | 底盘 | 多车型协议、超声波、毫米波 |
| `gnss_origin_pose` | 定位辅助 | GNSS 原点相关 |
| `site_pose` | 任务 | 站点位姿 / `send_sitedata` |
| `lane_follow` | 感知控制 | 车道跟随相关 |
| `driverless_ui` | 可选 UI | 操作界面 |
| `ui_bridge` | 可选 UI | C++ 动态库桥接 |
| `remote_communication` | 通信 | 远程通信示例 |
| `ros-bridge` | 可选仿真 | CARLA ROS Bridge（实车可不用） |
| `cv_bridge_1` / `vision_opencv` | 依赖 | 视觉桥接 |

---

## 2. 系统依赖

```bash
sudo apt-get install ros-melodic-jsk-recognition-msgs \
  ros-melodic-jsk-recognition \
  ros-melodic-jsk-rviz-plugins \
  libpcap-dev
```

LIO-SAM 另需 **GTSAM 4.0.2**，详见 `lio_sam_6axis/README.md`。

---

## 3. 多激光雷达数据接入（实车）

### 3.1 时间戳（`lslidar_c16_decoder/.../convert.cc`）

```cpp
outMsg.header.stamp = ros::Time::now();
```

多雷达融合时建议统一使用主机 ROS 时间，避免各雷达 GPS 时间不一致。

### 3.2 frame_id

```xml
<arg name="frame1_id" default="laser1_link" />
<arg name="frame2_id" default="laser2_link" />
```

### 3.3 传感器 IP

```xml
<arg name="device1_ip" default="192.168.86.200" />
<arg name="device2_ip" default="192.168.86.202" />
```

### 3.4 静态 TF

```xml
<node pkg="tf" type="static_transform_publisher" name="lidar1_link"
      args="1.15 0.625 -0.73 0 0 0 base_link laser1_link 100" />
<node pkg="tf" type="static_transform_publisher" name="lidar2_link"
      args="-1.15 -0.625 -0.7 0 0 0 base_link laser2_link 100" />
```

### 3.5 namespace 分组

```xml
<group ns="lidar16_1">
  <include file="$(find lslidar_c16_decoder)/launch/lslidar_c16.launch">
  ...
</group>
<group ns="lidar16_2">
  <include file="$(find lslidar_c16_decoder)/launch/lslidar_c16.launch">
  ...
</group>
```

### 3.6 IP 参数名

部分驱动版本使用 `lidar_ip` 而非 `device_ip`，按实际 launch 对齐：

```xml
<param name="lidar_ip" value="$(arg device_ip)"/>
```

### 3.7 端口必须互不冲突

```xml
<arg name="msop1_port" default="2368" />
<arg name="difop1_port" default="2369" />
<arg name="msop2_port" default="2370" />
<arg name="difop2_port" default="2371" />
```

### 3.8 障碍检测显示 frame

```xml
<!-- obstacle_detection.launch -->
<arg name="frame_id" default="laser_link" />
```

融合后建议统一到 `velodyne` / `base_link`，与 NDT、规划一致。

### 3.9 融合节点

见 `lidar_obstacle_detection/launch/mult_lidar_fusion.launch`：

- 输入：`/lidar16_1/points_raw`、`/lidar16_2/points_raw` …
- 输出：`/points_raw`
- 支持外参与角度扇区裁剪，减少重叠区重影

---

## 4. 与 Autoware 的关系

- **开源复用**：NDT、OpenPlanner、Pure Pursuit、YOLO、地面滤波、聚类跟踪等（`src/autoware/`）
- **本目录职责**：驱动、配置中心、安全限速、多车型 CAN、LIO-SAM 适配、启动编排

实车推荐主路径：

```
传感 → LIO-SAM/NDT 建图 → NDT 定位 → 感知+safe_range → OP 规划 → Pure Pursuit → CAN
```

---

## 5. 快速入口

| 操作 | 命令 / 文件 |
|------|-------------|
| 编译 | 仓库根目录 `./make.sh` |
| 选地图 | `./select_map.sh` |
| 选车型 | `./select_vehicle.sh` |
| 导航启动 | `roslaunch startingup_ros startingup.launch` |
| 线控 | `Common/custom_launch/my_auto.launch` |
| NDT 建图 | `.../map/build_pcd_map.launch` |
| LIO-SAM 建图 | `.../map/build_pcd_mapLio.launch` |

更完整的架构图、话题表、参数表与排障清单，请阅读根目录 **README.md**。
