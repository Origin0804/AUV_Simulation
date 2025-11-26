# AUV_Simulation
AUV (Autonomous Underwater Vehicle) simulation in ROS2

## 概述 (Overview)
本项目是一个基于ROS2的AUV（自主水下航行器）模拟系统，用于考察对AUV控制与导航的理解和应用能力。用户可以在此模拟环境中设计和实现AUV控制系统，完成目标识别等任务。

This project is a ROS2-based AUV (Autonomous Underwater Vehicle) simulation system designed to help understand and apply AUV control and navigation concepts. Users can design and implement AUV control systems in this simulated environment to complete tasks such as target recognition.

## 项目结构 (Project Structure)

```
AUV_Simulation/
├── src/
│   ├── auv_interfaces/     # 自定义消息定义 (Custom message definitions)
│   │   └── msg/
│   │       ├── DVLData.msg        # DVL速度数据
│   │       ├── IMUData.msg        # IMU加速度和角度数据
│   │       ├── CVModel.msg        # 摄像头视觉模型数据
│   │       ├── MotorsData.msg     # 电机PWM数据
│   │       ├── PositionData.msg   # 位置数据（内部使用）
│   │       ├── TotalForce.msg     # 总力和力矩（内部使用）
│   │       └── ObjectsData.msg    # 物体数据（内部使用）
│   │
│   ├── env_pkg/            # 环境模拟包 (Environment simulation package)
│   │   └── env_pkg/
│   │       ├── drag_node.py       # 阻力节点
│   │       └── props_node.py      # 道具节点
│   │
│   ├── AUV_pkg/            # AUV模拟包 (AUV simulation package)
│   │   └── AUV_pkg/
│   │       ├── position_node.py   # 位置方向节点
│   │       ├── motor_node.py      # 电机节点
│   │       ├── camera_node.py     # 摄像头节点
│   │       ├── dvl_node.py        # DVL节点
│   │       ├── imu_node.py        # IMU节点
│   │       └── main_node.py       # 用户编程节点
│   │
│   └── auv_bringup/        # 启动文件包 (Launch files package)
│       └── launch/
│           └── auv_simulation.launch.py
└── README.md
```

## 话题通讯 (Topic Communication)

### 用户可订阅的话题 (User-subscribable topics)
| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `DVL_data` | `auv_interfaces/DVLData` | 提供速度信息 |
| `imu_data` | `auv_interfaces/IMUData` | 提供加速度和角度信息 |
| `CV_modle` | `auv_interfaces/CVModel` | 提供目标检测信息 |

### 用户需发布的话题 (Topics for user to publish)
| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `motors_data` | `auv_interfaces/MotorsData` | 六个电机的PWM值 (-100 到 100) |

### 内部话题 (Internal topics - not for user access)
| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `posi_data` | `auv_interfaces/PositionData` | 位置、速度、加速度、方向数据 |
| `totle_force` | `auv_interfaces/TotalForce` | X,Y,Z轴的力和Rx,Ry,Rz的扭矩 |
| `objects_data` | `auv_interfaces/ObjectsData` | 物体位置信息 |
| `drag_force` | `auv_interfaces/TotalForce` | 阻力数据 |

## 节点说明 (Node Description)

### env_pkg 环境包
1. **drag_node (阻力节点)**: 根据简单流体阻力公式计算并返回阻力
2. **props_node (道具节点)**: 模拟环境中的道具，发布道具位置信息

### AUV_pkg AUV模拟包
1. **position_node (位置方向节点)**: 订阅`totle_force`，计算AUV的位置、速度、加速度、方向，发布`posi_data`
2. **motor_node (电机节点)**: 从`motors_data`订阅电机PWM，根据推力曲线计算六个电机的推力，发布`totle_force`
3. **camera_node (摄像头节点)**: 订阅`posi_data`和`objects_data`，模拟摄像头视野，发布`CV_modle`
4. **dvl_node (DVL节点)**: 订阅`posi_data`，发布`DVL_data`速度信息
5. **imu_node (IMU节点)**: 订阅`posi_data`，发布`imu_data`加速度和角度信息
6. **main_node (主节点)**: 用户编程节点，订阅`DVL_data`、`imu_data`、`CV_modle`，发布`motors_data`

## 安装与使用 (Installation and Usage)

### 依赖 (Dependencies)
- ROS2 (Humble or later recommended)
- Python 3.8+

### 编译 (Build)
```bash
cd AUV_Simulation
colcon build
source install/setup.bash
```

### 运行 (Run)
```bash
# 启动完整模拟
ros2 launch auv_bringup auv_simulation.launch.py
```

## 用户编程指南 (User Programming Guide)

用户主要在 `main_node.py` 中的 `control_loop` 方法中编写控制逻辑。

Users should primarily write their control logic in the `control_loop` method in `main_node.py`.

### 可用数据 (Available Data)
- `self.dvl_data`: DVL速度数据 (velocity information)
- `self.imu_data`: IMU加速度和角度数据 (acceleration and orientation)
- `self.cv_data`: 摄像头检测结果 (camera detection results)

### 电机配置 (Motor Configuration)
- 电机1-4: 水平推进器 (用于X、Y方向运动和偏航)
- 电机5-6: 垂直推进器 (用于Z方向运动和俯仰/横滚)

## 许可证 (License)
MIT License
