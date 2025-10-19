---
license: apache-2.0
tags:
- LeRobot
- robotics
- dobot
- teach-drag
task_categories:
- robotics
---

# cr5_drag_teach_20251019_190319

这是一个使用标准LeRobot格式的机器人数据集，包含dobot_cr5机器人的示教轨迹数据。

## 数据集信息

- **机器人类型**: dobot_cr5
- **采集频率**: 10 Hz
- **总Episodes**: 1
- **总帧数**: 221
- **创建时间**: 2025-10-19T19:04:08.598280

## 特征说明

- **observation.state**: float32, shape=(6,), names=['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
- **observation.cartesian_position**: float32, shape=(6,), names=['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz']
- **action**: float32, shape=(7,), names=['J1_vel', 'J2_vel', 'J3_vel', 'J4_vel', 'J5_vel', 'J6_vel', 'gripper']
- **observation.images.camera_color**: video, shape=(480, 640, 3), names=['height', 'width', 'channels']
- **observation.images.camera_depth**: video, shape=(480, 640, 3), names=['height', 'width', 'channels']
- **episode_index**: int64, shape=(), names=None
- **frame_index**: int64, shape=(), names=None
- **timestamp**: float32, shape=(), names=None
- **next.done**: bool, shape=(), names=None
- **index**: int64, shape=(), names=None

## 使用方法

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset

# 加载数据集
dataset = LeRobotDataset("cr5_drag_teach_20251019_190319")

# 获取一帧数据
frame = dataset[0]
print(frame.keys())
```

## 数据集结构

该数据集遵循标准的LeRobot格式，包含以下文件：

- `meta/info.json`: 数据集基本信息
- `meta/episodes.parquet`: Episode元数据
- `meta/stats.json`: 数据统计信息
- `meta/tasks.parquet`: 任务信息
- `data/`: Parquet格式的帧数据
- `videos/`: MP4格式的视频文件

创建时间: 2025-10-19 19:04:08
