"""
兼容LeRobot格式的数据集保存器实现
由于LeRobot库存在依赖问题，使用自定义实现生成标准格式
基于convert_libero_data_to_lerobot.py的格式规范
"""

import json
import numpy as np
import pandas as pd
import os
import cv2
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, Optional


class LeRobotDatasetSaver:
    """
    兼容LeRobot格式的数据集保存器
    生成标准的机器人学习数据集格式，无需依赖LeRobot库
    参考convert_libero_data_to_lerobot.py的数据结构
    """
    
    def __init__(self, repo_id: str, fps: int = 10, robot_type: str = "dobot_cr5"):
        """
        初始化数据集保存器
        
        Args:
            repo_id (str): 数据集仓库ID，例如 "dobot/cr5_teach_demo_20241009_143000"
            fps (int): 数据采集帧率
            robot_type (str): 机器人类型
        """
        self.repo_id = repo_id
        self.fps = fps
        self.robot_type = robot_type
        
        # 创建数据集目录
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.dataset_name = f"lerobot_dataset_{timestamp}"
        self.root_path = Path(self.dataset_name)
        
        # 创建目录结构
        self._create_directory_structure()
        
        # 数据存储
        self.episodes_data = []
        self.frames_data = []
        self.current_episode_data = []
        self.current_episode_index = 0
        self.total_frames = 0
        self.current_task = None
        
        # 特征定义 - 参考convert_libero_data_to_lerobot.py
        self.features = {
            "state": {
                "dtype": "float32",
                "shape": (6,),
                "names": ["J1", "J2", "J3", "J4", "J5", "J6"],
            },
            "cartesian_position": {
                "dtype": "float32", 
                "shape": (6,),
                "names": ["X", "Y", "Z", "Rx", "Ry", "Rz"],
            },
            "action": {
                "dtype": "float32",
                "shape": (6,),
                "names": ["J1", "J2", "J3", "J4", "J5", "J6"],
            },
            "camera_color": {
                "dtype": "image",
                "shape": (480, 640, 3),
                "names": ["height", "width", "channel"],
            },
            "camera_depth": {
                "dtype": "image", 
                "shape": (480, 640, 3),
                "names": ["height", "width", "channel"],
            },
        }
        
        # 视频编写器
        self.video_writers = {}
        self.video_frame_counts = {}
        
        print(f"✓ LeRobot兼容数据集创建成功: {self.repo_id}")
        print(f"✓ 机器人类型: {self.robot_type}")
        print(f"✓ 采集频率: {self.fps} Hz")
        print(f"✓ 数据集路径: {self.root_path}")
    
    def _create_directory_structure(self):
        """创建LeRobot标准目录结构"""
        self.root_path.mkdir(parents=True, exist_ok=True)
        
        # 创建数据目录
        self.data_path = self.root_path / "data"
        self.data_path.mkdir(exist_ok=True)
        
        # 创建视频目录
        self.videos_path = self.root_path / "videos"
        self.videos_path.mkdir(exist_ok=True)
        (self.videos_path / "camera_color").mkdir(exist_ok=True)
        (self.videos_path / "camera_depth").mkdir(exist_ok=True)
        
        print(f"✓ 创建数据集目录结构: {self.root_path}")
    
    def start_episode(self, task_name: str = "teach_drag"):
        """开始一个新的episode"""
        self.current_task = task_name
        self.current_episode_data = []
        
        # 初始化视频编写器
        self._init_video_writers()
        
        print(f"✓ 开始新的episode: {task_name}")
    
    def _init_video_writers(self):
        """初始化视频编写器"""
        # 使用更兼容的fourcc编码
        fourcc = cv2.VideoWriter.fourcc(*'mp4v')
        
        # 彩色视频
        color_path = self.videos_path / "camera_color" / f"episode_{self.current_episode_index:06d}.mp4"
        self.video_writers["color"] = cv2.VideoWriter(
            str(color_path), fourcc, self.fps, (640, 480)
        )
        
        # 深度视频
        depth_path = self.videos_path / "camera_depth" / f"episode_{self.current_episode_index:06d}.mp4"
        self.video_writers["depth"] = cv2.VideoWriter(
            str(depth_path), fourcc, self.fps, (640, 480)
        )
        
        self.video_frame_counts = {"color": 0, "depth": 0}
    
    def add_frame(self, robot_data: Dict[str, Any], camera_data: Optional[Dict[str, Any]] = None):
        """添加一帧数据"""
        # 准备机器人数据，确保NumPy兼容的类型转换
        state = [float(robot_data['J1']), float(robot_data['J2']), float(robot_data['J3']),
                float(robot_data['J4']), float(robot_data['J5']), float(robot_data['J6'])]
        
        cartesian_pos = [float(robot_data['X']), float(robot_data['Y']), float(robot_data['Z']),
                        float(robot_data['Rx']), float(robot_data['Ry']), float(robot_data['Rz'])]
        
        action = state.copy()  # 使用当前状态作为动作
        
        # 准备帧数据
        frame_data = {
            "episode_index": self.current_episode_index,
            "frame_index": len(self.current_episode_data),
            "timestamp": robot_data.get('timestamp', 0),
            "task": self.current_task,
            "state": state,
            "cartesian_position": cartesian_pos,
            "action": action,
        }
        
        # 处理相机数据
        if camera_data is not None:
            self._save_camera_frame(camera_data, frame_data["frame_index"])
            frame_data["has_camera"] = True
        else:
            # 保存空白帧
            self._save_empty_camera_frame(frame_data["frame_index"])
            frame_data["has_camera"] = False
        
        self.current_episode_data.append(frame_data)
        self.total_frames += 1
    
    def _save_camera_frame(self, camera_data: Dict[str, Any], frame_index: int):
        """保存相机帧到视频"""
        # 处理彩色图像
        if "color_image" in camera_data:
            color_img = camera_data["color_image"]
            # 确保数组是NumPy数组并且是正确的类型
            if not isinstance(color_img, np.ndarray):
                color_img = np.array(color_img)
            
            # 确保数据类型正确
            if color_img.dtype != np.uint8:
                color_img = color_img.astype(np.uint8)
            
            if color_img.shape != (480, 640, 3):
                color_img = cv2.resize(color_img, (640, 480))
            
            # 转换BGR到RGB (OpenCV使用BGR)
            if len(color_img.shape) == 3 and color_img.shape[2] == 3:
                color_bgr = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)
            else:
                color_bgr = color_img
            
            self.video_writers["color"].write(color_bgr)
            self.video_frame_counts["color"] += 1
        
        # 处理深度图像
        if "depth_image" in camera_data:
            depth_img = camera_data["depth_image"]
            # 确保数组是NumPy数组
            if not isinstance(depth_img, np.ndarray):
                depth_img = np.array(depth_img)
            
            if len(depth_img.shape) == 2:
                # 将深度图转换为可视化的彩色图
                # 安全的除法处理
                max_val = depth_img.max()
                if max_val > 0:
                    depth_normalized = ((depth_img / max_val) * 255).astype(np.uint8)
                else:
                    depth_normalized = np.zeros_like(depth_img, dtype=np.uint8)
                depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_PLASMA)
            else:
                depth_colored = depth_img.astype(np.uint8) if depth_img.dtype != np.uint8 else depth_img
            
            if depth_colored.shape[:2] != (480, 640):
                depth_colored = cv2.resize(depth_colored, (640, 480))
            
            self.video_writers["depth"].write(depth_colored)
            self.video_frame_counts["depth"] += 1
    
    def _save_empty_camera_frame(self, frame_index: int):
        """保存空白相机帧"""
        # 空白彩色帧
        empty_color = np.zeros((480, 640, 3), dtype=np.uint8)
        self.video_writers["color"].write(empty_color)
        self.video_frame_counts["color"] += 1
        
        # 空白深度帧
        empty_depth = np.zeros((480, 640, 3), dtype=np.uint8)
        self.video_writers["depth"].write(empty_depth)
        self.video_frame_counts["depth"] += 1
    
    def save_episode(self):
        """保存当前episode"""
        if not self.current_episode_data:
            print("⚠ 当前episode没有数据，跳过保存")
            return
        
        # 关闭视频编写器
        for writer in self.video_writers.values():
            if writer:
                writer.release()
        
        # 保存episode数据
        episode_info = {
            "episode_index": self.current_episode_index,
            "task": self.current_task,
            "frame_count": len(self.current_episode_data),
            "duration": self.current_episode_data[-1]["timestamp"] if self.current_episode_data else 0,
        }
        self.episodes_data.append(episode_info)
        
        # 将当前episode的帧数据添加到总数据中
        self.frames_data.extend(self.current_episode_data)
        
        print(f"✓ Episode {self.current_episode_index} 已保存，包含 {len(self.current_episode_data)} 帧")
        
        # 准备下一个episode
        self.current_episode_index += 1
        self.current_episode_data = []
        self.video_writers = {}
    
    def finalize_dataset(self) -> str:
        """完成数据集保存"""
        # 确保最后一个episode被保存
        if self.current_episode_data:
            self.save_episode()
        
        # 保存数据集元数据
        self._save_metadata()
        
        # 保存帧数据为CSV/Parquet格式
        self._save_frame_data()
        
        print(f"✓ LeRobot兼容数据集保存完成")
        print(f"✓ 数据集路径: {self.root_path}")
        print(f"✓ 总Episodes: {len(self.episodes_data)}")
        print(f"✓ 总帧数: {self.total_frames}")
        
        return str(self.root_path)
    
    def _save_metadata(self):
        """保存数据集元数据"""
        metadata = {
            "repo_id": self.repo_id,
            "robot_type": self.robot_type,
            "fps": self.fps,
            "total_episodes": len(self.episodes_data),
            "total_frames": self.total_frames,
            "features": self.features,
            "created_at": datetime.now().isoformat(),
            "episodes": self.episodes_data
        }
        
        # 保存为JSON
        with open(self.root_path / "metadata.json", 'w', encoding='utf-8') as f:
            json.dump(metadata, f, indent=2, ensure_ascii=False)
        
        print(f"✓ 元数据已保存: metadata.json")
    
    def _save_frame_data(self):
        """保存帧数据"""
        if not self.frames_data:
            return
        
        # 转换为DataFrame，确保数据类型正确
        try:
            df = pd.DataFrame(self.frames_data)
            
            # 确保数值列的类型正确
            numeric_columns = ['episode_index', 'frame_index', 'timestamp']
            for col in numeric_columns:
                if col in df.columns:
                    df[col] = pd.to_numeric(df[col], errors='coerce')
            
            # 保存为CSV
            df.to_csv(self.root_path / "frames_data.csv", index=False)
            
            # 尝试保存为Parquet格式（如果可能）
            try:
                # 使用更兼容的Parquet引擎
                df.to_parquet(self.root_path / "frames_data.parquet", 
                             index=False, 
                             engine='pyarrow')
                print(f"✓ 帧数据已保存: frames_data.parquet")
            except ImportError:
                print(f"✓ 帧数据已保存: frames_data.csv (PyArrow不可用)")
            except Exception as e:
                print(f"⚠ Parquet保存失败: {e}")
                print(f"✓ 帧数据已保存: frames_data.csv")
                
        except Exception as e:
            print(f"⚠ 数据保存出错: {e}")
            # 紧急保存原始数据
            import json
            with open(self.root_path / "frames_data_raw.json", 'w') as f:
                json.dump(self.frames_data, f, indent=2)
            print(f"✓ 原始数据已保存: frames_data_raw.json")


# 简化的工厂函数
def create_lerobot_saver(task_name: str = "teach_drag", fps: int = 10) -> LeRobotDatasetSaver:
    """
    创建LeRobot数据集保存器的便捷函数
    
    Args:
        task_name (str): 任务名称
        fps (int): 采集频率
        
    Returns:
        LeRobotDatasetSaver: 数据集保存器实例
    """
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    repo_id = f"dobot/cr5_{task_name}_{timestamp}"
    
    saver = LeRobotDatasetSaver(
        repo_id=repo_id,
        fps=fps,
        robot_type="dobot_cr5"
    )
    
    saver.start_episode(task_name)
    return saver