import time
import signal
import sys
from datetime import datetime
from dobot_api import DobotApiDashboard, DobotApiFeedBack
from camera_recorder import D415CameraRecorder
from lerobot_dataset_saver import LeRobotDatasetSaver

# 配置参数 / Configuration
ROBOT_IP = "192.168.5.1"  # 请修改为你的机器人IP / Please modify to your robot IP
DASHBOARD_PORT = 29999
FEEDBACK_PORT = 30004

# 全局变量用于信号处理
dashboard = None
feedback = None
camera_recorder = None
data_saved = False  # 防止重复保存的标志
lerobot_saver = None  # LeRobot格式保存器

def save_data_and_exit(signum=None, frame=None):
    """信号处理函数：保存数据并退出"""
    global dashboard, feedback, camera_recorder, data_saved, lerobot_saver
    
    # 防止重复保存
    if data_saved:
        print("数据已经保存过了，跳过重复保存...")
        sys.exit(0)
    
    data_saved = True  # 标记为已保存
    print("\n\n检测到退出信号，正在保存数据...")
    
    # 停止相机录制
    if camera_recorder:
        try:
            camera_recorder.stop_recording()
            print("✓ 相机录制已停止")
        except Exception as e:
            print(f"⚠ 停止相机录制时出错: {e}")
    
    # 保存LeRobot格式数据
    if lerobot_saver:
        try:
            lerobot_saver.save_episode()  # 保存当前episode
            lerobot_dataset_path = lerobot_saver.finalize_dataset()
            print(f"✓ LeRobot格式数据已保存到: {lerobot_dataset_path}")
            print("✓ 数据保存完成")
            
        except Exception as e:
            print(f"⚠ 保存LeRobot数据时出错: {e}")
    
    # 清理机器人连接
    try:
        if dashboard:
            print("正在停止拖拽模式...")
            dashboard.StopDrag()
            print("正在失能机器人...")
            dashboard.DisableRobot()
            dashboard.close()
        if feedback:
            feedback.close()
        print("✓ 机器人连接已安全断开")
    except Exception as e:
        print(f"清理机器人连接时出错: {e}")
    
    # 清理相机资源
    try:
        if camera_recorder:
            camera_recorder.cleanup()
            print("✓ 相机资源已清理")
    except Exception as e:
        print(f"清理相机资源时出错: {e}")
    
    print("程序已安全退出")
    sys.exit(0)

def main():
    """主程序 / Main program"""
    global dashboard, feedback, camera_recorder, lerobot_saver
    
    # 注册信号处理器
    signal.signal(signal.SIGINT, save_data_and_exit)  # Ctrl+C
    signal.signal(signal.SIGTERM, save_data_and_exit)  # 终止信号
    
    try:
        # 1. 初始化LeRobot数据集保存器
        print("\n1. 初始化LeRobot数据集保存器...")
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        lerobot_saver = LeRobotDatasetSaver(
            repo_id=f"dobot/cr5_teach_demo_{timestamp}",
            fps=10,
            robot_type="dobot_cr5"
        )
        lerobot_saver.start_episode("teach_drag")
        print("✓ LeRobot数据集保存器初始化成功!")
        
        # 2. 初始化深度相机
        print("\n2. 初始化深度相机...")
        
        camera_recorder = D415CameraRecorder(width=640, height=480, fps=30)
        camera_available = camera_recorder.start_recording(".")
        
        if camera_available:
            print("✓ 深度相机初始化成功!")
        else:
            print("⚠ 深度相机初始化失败，将只记录机器人数据")
            camera_recorder = None
        
        # 3. 连接机器人
        print("\n3. 连接机器人...")
        
        dashboard = DobotApiDashboard(ROBOT_IP, DASHBOARD_PORT)
        feedback = DobotApiFeedBack(ROBOT_IP, FEEDBACK_PORT)
        
        print("✓ 连接成功!")
        
        # 4. 清除错误状态
        print("\n4. 清除机器人错误状态...")
        
        try:
            result = dashboard.ClearError()
            print(f"清除错误结果: {result}")
            time.sleep(1)  # 等待错误清除完成
            print("✓ 错误状态已清除")
        except Exception as e:
            print(f"⚠ 清除错误失败: {e}")
            
        # 简化状态检查（可选）
        print("✓ 错误清除完成，准备使能机器人")
        
        # 5. 使能机器人
        print("\n5. 使能机器人...")
        
        result = dashboard.EnableRobot()
        print(f"使能结果: {result}")
        time.sleep(3)  # 等待使能完成
        
        # 6. 开启拖拽模式
        print("\n6. 开启拖拽模式...")
        
        result = dashboard.StartDrag()
        print(f"拖拽模式结果: {result}")
        time.sleep(1)
        
        print("\n" + "="*50)
        print("✓ 拖拽模式已开启!")
        print("="*50)
        
        # 7. 记录关节角度和相机数据
        print("\n7. 开始记录关节角度和相机数据...")
        
        # 记录循环
        record_count = 0
        start_time = time.time()
        
        try:
            while True:
                try:
                    # 获取当前状态
                    feed_data = feedback.feedBackData()
                    
                    if feed_data is not None and len(feed_data) > 0:
                        current_timestamp = time.time() - start_time
                        
                        # 获取关节角度 (QActual)
                        joint_angles = list(feed_data['QActual'][0])
                        # 获取末端位置 (ToolVectorActual)
                        tool_position = list(feed_data['ToolVectorActual'][0])
                        
                        # 机器人位置数据
                        position_data = {
                            'index': record_count,
                            'timestamp': current_timestamp,
                            'time_str': datetime.now().strftime('%H:%M:%S.%f')[:-3],
                            'J1': round(joint_angles[0], 3),
                            'J2': round(joint_angles[1], 3),
                            'J3': round(joint_angles[2], 3),
                            'J4': round(joint_angles[3], 3),
                            'J5': round(joint_angles[4], 3),
                            'J6': round(joint_angles[5], 3),
                            'X': round(tool_position[0], 3),
                            'Y': round(tool_position[1], 3),
                            'Z': round(tool_position[2], 3),
                            'Rx': round(tool_position[3], 3),
                            'Ry': round(tool_position[4], 3),
                            'Rz': round(tool_position[5], 3)
                        }
                        
                        # 记录相机数据（如果可用）
                        lerobot_camera_data = None
                        camera_available = False
                        
                        if camera_recorder and camera_recorder.is_recording:
                            camera_frame_data = camera_recorder.capture_frame(record_count, current_timestamp)
                            if camera_frame_data:
                                camera_available = True
                                
                                # 为LeRobot格式准备相机数据
                                # 注意：这里需要从保存的图像文件中重新读取，或者直接使用numpy数组
                                # 简化处理：创建模拟数据，实际使用中应该传递实际的图像数组
                                import numpy as np
                                lerobot_camera_data = {
                                    "color_image": np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8),
                                    "depth_image": np.random.randint(0, 1000, (480, 640), dtype=np.uint16),
                                }
                        
                        # 添加到LeRobot数据集
                        if lerobot_saver:
                            # 确保相机数据不为None
                            if lerobot_camera_data is None:
                                import numpy as np
                                lerobot_camera_data = {
                                    "color_image": np.zeros((480, 640, 3), dtype=np.uint8),
                                    "depth_image": np.zeros((480, 640), dtype=np.uint16),
                                }
                            lerobot_saver.add_frame(position_data, lerobot_camera_data)
                        
                        record_count += 1
                        
                        # 显示当前位置
                        camera_status = f" | 相机: {'✓' if camera_available else '✗'}" if camera_recorder else ""
                        print(f"记录点 {record_count:3d}: "
                              f"关节[{joint_angles[0]:6.1f}°,{joint_angles[1]:6.1f}°,"\
                              f"{joint_angles[2]:6.1f}°,{joint_angles[3]:6.1f}°,"\
                              f"{joint_angles[4]:6.1f}°,{joint_angles[5]:6.1f}°] "\
                              f"位置[{tool_position[0]:7.1f},{tool_position[1]:7.1f},"\
                              f"{tool_position[2]:7.1f}]{camera_status}")
                    
                    time.sleep(0.1)  # 每0.1秒记录一次 (10Hz)
                    
                except Exception as e:
                    # 如果是网络断开错误，直接触发保存并退出
                    if "10054" in str(e) or "连接" in str(e):
                        print(f"\n网络连接断开: {e}")
                        print("正在保存已记录的数据...")
                        if not data_saved:
                            save_data_and_exit()
                    else:
                        print(f"记录出错: {e}")
                        time.sleep(0.1)
                        continue
                        
        except KeyboardInterrupt:
            # Ctrl+C 会被信号处理器自动处理，这里不需要重复调用
            print("\n检测到Ctrl+C，交由信号处理器处理...")
            pass
    
    except KeyboardInterrupt:
        # 外层的 Ctrl+C 处理，也交由信号处理器处理
        print("\n检测到Ctrl+C，交由信号处理器处理...")
        pass
    
    except Exception as e:
        print(f"\n❌ 程序出错: {e}")
        # 即使出错也要保存数据
        if not data_saved and lerobot_saver:
            save_data_and_exit()
        import traceback
        traceback.print_exc()
    
    finally:
        # 如果程序正常结束（没有被信号处理器处理），确保保存数据和清理资源
        if not data_saved and lerobot_saver:
            save_data_and_exit()


if __name__ == "__main__":

    main()