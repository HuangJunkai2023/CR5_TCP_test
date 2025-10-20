#!/usr/bin/env python3
"""
使用 LeRobot 官方库进行 CR5 机械臂数据采集
Official LeRobot library for CR5 robot data collection
"""

import time
import numpy as np
from pathlib import Path
from datetime import datetime
from lerobot.record import LeRobotDataset
import pyrealsense2 as rs
import cv2
import sys
import threading

# 添加 CR5 API 路径
cr5_path = Path(__file__).parent / "files"
sys.path.append(str(cr5_path))
from dobot_api import DobotApiDashboard, DobotApiFeedBack

# 尝试导入夹爪控制
try:
    from gripper import Robotiq2F85
    GRIPPER_AVAILABLE = True
except ImportError:
    GRIPPER_AVAILABLE = False
    print("⚠️  警告: 无法导入 gripper 模块，夹爪功能将不可用")

# 尝试导入键盘监听（Windows 特定）
try:
    import msvcrt
    KEYBOARD_AVAILABLE = True
except ImportError:
    KEYBOARD_AVAILABLE = False
    print("⚠️  警告: 非 Windows 系统，键盘控制将使用替代方案")

# ====================
# 配置参数
# ====================

ROBOT_IP = "192.168.5.1"
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
FPS = 10  # 10 Hz 采集频率
TASK_NAME = "grasp_cube"  # 任务描述

# 夹爪配置
GRIPPER_PORT = "COM5"  # 夹爪串口号（根据实际情况修改）
GRIPPER_THRESHOLD_MM = 50.0  # 夹爪开口阈值（毫米）

# 全局变量
gripper = None
gripper_state = 0.0  # 0.0=打开, 1.0=关闭
gripper_lock = threading.Lock()
keyboard_thread_running = False

# ====================
# 夹爪控制类
# ====================

class GripperController:
    """夹爪控制器，支持 Robotiq 2F-85"""
    
    def __init__(self, port=GRIPPER_PORT):
        global gripper, gripper_state
        self.gripper = None
        
        if not GRIPPER_AVAILABLE:
            print("⚠️  夹爪模块不可用，将使用模拟状态")
            return
        
        try:
            print(f"🤖 初始化夹爪 @ {port}...")
            self.gripper = Robotiq2F85(port)
            self.gripper.activate_gripper()
            time.sleep(1)
            gripper = self.gripper
            gripper_state = 0.0
            print("✅ 夹爪已激活")
        except Exception as e:
            print(f"⚠️  夹爪初始化失败: {e}")
            self.gripper = None
    
    def open(self):
        """打开夹爪"""
        global gripper_state
        if self.gripper:
            try:
                # 注意：Robotiq API 可能反向，open_gripper 关闭，close_gripper 打开
                self.gripper.close_gripper(speed=100, force=170)
                with gripper_lock:
                    gripper_state = 0.0
                print("🔓 夹爪打开")
            except Exception as e:
                print(f"⚠️  打开夹爪失败: {e}")
    
    def close(self):
        """关闭夹爪"""
        global gripper_state
        if self.gripper:
            try:
                # 注意：Robotiq API 可能反向
                self.gripper.open_gripper(speed=255, force=200, wait=0.1)
                with gripper_lock:
                    gripper_state = 1.0
                print("🔒 夹爪关闭")
            except Exception as e:
                print(f"⚠️  关闭夹爪失败: {e}")
    
    def get_state(self):
        """获取当前夹爪状态 (0.0=打开, 1.0=关闭)"""
        with gripper_lock:
            return gripper_state
    
    def cleanup(self):
        """清理资源"""
        if self.gripper:
            try:
                self.gripper.close()
                print("✅ 夹爪已断开")
            except:
                pass

# ====================
# 键盘监听线程
# ====================

def keyboard_listener(gripper_controller):
    """键盘监听线程：监听夹爪控制按键"""
    global keyboard_thread_running
    
    print("\n" + "="*60)
    print("⌨️  键盘控制说明:")
    print("  按 'O' 键 → 打开夹爪 (Open)")
    print("  按 'C' 键 → 关闭夹爪 (Close)")
    print("  按 'Q' 键 → 结束当前 episode")
    print("="*60 + "\n")
    
    keyboard_thread_running = True
    
    if not KEYBOARD_AVAILABLE:
        print("⚠️  键盘监听不可用（非 Windows 系统）")
        return
    
    while keyboard_thread_running:
        try:
            if msvcrt.kbhit():
                key_bytes = msvcrt.getch()
                try:
                    key = key_bytes.decode('utf-8').upper()
                    
                    if key == 'O':  # 打开夹爪
                        gripper_controller.open()
                    elif key == 'C':  # 关闭夹爪
                        gripper_controller.close()
                        
                except UnicodeDecodeError:
                    pass
            
            time.sleep(0.05)
        except Exception as e:
            print(f"⚠️  键盘监听异常: {e}")
            break

# ====================
# CR5 机械臂控制
# ====================

class CR5Robot:
    def __init__(self, ip="192.168.5.1"):
        self.ip = ip
        self.dashboard = DobotApiDashboard(ip, 29999)
        self.feed = DobotApiFeedBack(ip, 30004)
        
    def connect(self):
        print(f"🔌 连接 CR5 @ {self.ip}...")
        self.dashboard.connect()
        self.feed.connect()
        self.dashboard.EnableRobot()
        time.sleep(0.5)
        self.dashboard.DragTeachSwitch(1)  # 启用拖拽
        print("✅ 已连接 (拖拽模式)")
        
    def disconnect(self):
        self.dashboard.DragTeachSwitch(0)
        self.dashboard.DisableRobot()
        self.dashboard.close()
        self.feed.close()
        
    def get_state(self):
        """返回: (joint_positions, joint_velocities, gripper_position)"""
        feed_data = self.feed.get_feed()
        if not feed_data:
            return None, None, None
        
        # 关节位置（弧度）
        joints_deg = feed_data.get('ActualQd', [0]*6)[:6]
        joint_positions = np.deg2rad(joints_deg).astype(np.float32)
        
        # 关节速度（rad/s）
        velocities_deg = feed_data.get('ActualQvd', [0]*6)[:6]
        joint_velocities = np.deg2rad(velocities_deg).astype(np.float32)
        
        # 夹爪（0=打开, 1=关闭）
        do_status = feed_data.get('ActualDO', [0])
        gripper_closed = do_status[0] if len(do_status) > 0 else 0
        gripper_position = float(gripper_closed)
        
        return joint_positions, joint_velocities, gripper_position

# ====================
# RealSense 相机
# ====================

class RealSenseCamera:
    def __init__(self, width=640, height=480, fps=30):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.config = config
        
    def start(self):
        print("📷 启动相机...")
        self.pipeline.start(self.config)
        for _ in range(30):  # 预热
            self.pipeline.wait_for_frames()
        print("✅ 相机就绪")
        
    def stop(self):
        self.pipeline.stop()
        
    def get_frame(self):
        """返回 RGB 图像 (H, W, 3)"""
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        image_bgr = np.asanyarray(color_frame.get_data())
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        return image_rgb

# ====================
# 主采集函数
# ====================

def record_episode(dataset, robot, camera, gripper_controller, episode_idx, task_name):
    """
    记录一个 episode
    
    Args:
        dataset: LeRobotDataset 实例
        robot: CR5Robot 实例
        camera: RealSenseCamera 实例
        gripper_controller: GripperController 实例
        episode_idx: Episode 编号
        task_name: 任务名称
    """
    print(f"\n{'='*60}")
    print(f"📹 Episode {episode_idx} - {task_name}")
    print(f"{'='*60}")
    print("   按 's' 开始记录")
    print("   记录期间: 'O'=打开夹爪, 'C'=关闭夹爪, 'q'=结束")
    
    # 预览并等待开始
    cv2.namedWindow('CR5 Data Collection')
    while True:
        image_rgb = camera.get_frame()
        if image_rgb is not None:
            display = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
            
            # 显示夹爪状态
            gripper_status = gripper_controller.get_state()
            gripper_text = "🔒 关闭" if gripper_status > 0.5 else "🔓 打开"
            
            cv2.putText(display, "Press 's' to START", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(display, f"Gripper: {gripper_text}", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.imshow('CR5 Data Collection', display)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            break
        elif key == ord('q'):
            cv2.destroyAllWindows()
            return False
    
    print("🎬 开始记录...")
    
    # 创建 episode buffer
    dataset.create_episode_buffer()
    
    frame_count = 0
    start_time = time.time()
    
    while True:
        loop_start = time.time()
        
        # 获取机械臂状态
        joint_pos, joint_vel, _ = robot.get_state()  # 忽略机械臂的夹爪状态
        if joint_pos is None:
            print("⚠️  无法获取机械臂状态")
            time.sleep(0.01)
            continue
        
        # 获取真实夹爪状态
        gripper_state_value = gripper_controller.get_state()
        
        # 获取图像
        image_rgb = camera.get_frame()
        if image_rgb is None:
            print("⚠️  无法获取图像")
            time.sleep(0.01)
            continue
        
        # 构建帧数据
        frame_data = {
            "observation.state": joint_pos,  # (6,) 关节位置
            "observation.images.top": image_rgb,  # (H, W, 3) RGB
            "action": np.concatenate([joint_vel, [gripper_state_value]]),  # (7,) 速度+夹爪
        }
        
        # 添加到数据集
        timestamp = time.time() - start_time
        dataset.add_frame(frame_data, task=task_name, timestamp=timestamp)
        
        frame_count += 1
        
        # 显示
        display = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        elapsed = time.time() - loop_start
        actual_fps = 1.0 / elapsed if elapsed > 0 else 0
        
        # 夹爪状态显示
        gripper_text = "🔒 CLOSED" if gripper_state_value > 0.5 else "🔓 OPEN"
        gripper_color = (0, 0, 255) if gripper_state_value > 0.5 else (0, 255, 0)
        
        cv2.putText(display, f"Frame: {frame_count}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(display, f"FPS: {actual_fps:.1f}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(display, f"Gripper: {gripper_text}", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, gripper_color, 2)
        cv2.putText(display, "O=Open | C=Close | Q=Stop", (10, 120), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow('CR5 Data Collection', display)
        
        # 检查退出
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('o') or key == ord('O'):
            gripper_controller.open()
        elif key == ord('c') or key == ord('C'):
            gripper_controller.close()
        
        # 精确 10Hz
        elapsed = time.time() - loop_start
        sleep_time = max(0, 1.0/FPS - elapsed)
        if sleep_time > 0:
            time.sleep(sleep_time)
    
    cv2.destroyAllWindows()
    
    # 保存 episode
    print(f"💾 保存 Episode {episode_idx} ({frame_count} 帧)...")
    dataset.save_episode()
    
    print(f"✅ Episode {episode_idx} 已保存")
    return True

# ====================
# 主程序
# ====================

def main():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    dataset_name = f"cr5_{TASK_NAME}_{timestamp}"
    
    print("="*60)
    print("🤖 CR5 LeRobot 官方数据采集")
    print("="*60)
    print(f"📊 数据集: {dataset_name}")
    print(f"📷 分辨率: {CAMERA_WIDTH}x{CAMERA_HEIGHT}")
    print(f"⏱️  频率: {FPS} Hz")
    print(f"🎯 任务: {TASK_NAME}")
    print("="*60)
    
    # 初始化
    robot = CR5Robot(ROBOT_IP)
    camera = RealSenseCamera(CAMERA_WIDTH, CAMERA_HEIGHT)
    gripper_controller = GripperController(GRIPPER_PORT)
    
    # 启动键盘监听线程
    keyboard_thread = None
    if KEYBOARD_AVAILABLE and gripper_controller.gripper:
        keyboard_thread = threading.Thread(
            target=keyboard_listener, 
            args=(gripper_controller,),
            daemon=True
        )
        keyboard_thread.start()
    
    try:
        robot.connect()
        camera.start()
        
        # 定义 LeRobot 特征
        features = {
            "observation.state": {
                "dtype": "float32",
                "shape": (6,),
                "names": [f"joint_{i}" for i in range(6)],
            },
            "observation.images.top": {
                "dtype": "video",
                "shape": (CAMERA_HEIGHT, CAMERA_WIDTH, 3),
                "names": ["height", "width", "channel"],
            },
            "action": {
                "dtype": "float32",
                "shape": (7,),
                "names": [f"joint_{i}_vel" for i in range(6)] + ["gripper"],
            },
        }
        
        # 创建数据集
        print("\n💾 创建 LeRobot 数据集...")
        dataset = LeRobotDataset.create(
            repo_id=dataset_name,
            fps=FPS,
            features=features,
            root="./lerobot_data",
            robot_type="cr5",
            use_videos=True,
        )
        print(f"✅ 数据集已创建: ./lerobot_data/{dataset_name}")
        
        # 记录 episodes
        episode_idx = 0
        while True:
            success = record_episode(dataset, robot, camera, gripper_controller, episode_idx, TASK_NAME)
            if not success:
                break
            episode_idx += 1
            
            # 询问是否继续
            print(f"\n已完成 {episode_idx} 个 episodes")
            response = input("继续记录下一个 episode? (y/n): ").lower()
            if response != 'y':
                break
        
        # 完成
        print("\n" + "="*60)
        print(f"✅ 采集完成！总共 {episode_idx} 个 episodes")
        print(f"📂 数据集路径: ./lerobot_data/{dataset_name}")
        print("\n🚀 下一步训练:")
        print(f"    python scripts/train_pytorch.py \\")
        print(f"      --pretrained-checkpoint pi0_droid \\")
        print(f"      --dataset-repo-id {dataset_name}")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n⚠️  用户中断")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 停止键盘线程
        global keyboard_thread_running
        keyboard_thread_running = False
        if keyboard_thread:
            keyboard_thread.join(timeout=1)
        
        # 清理资源
        camera.stop()
        robot.disconnect()
        gripper_controller.cleanup()

if __name__ == "__main__":
    main()
