import time
import signal
import sys
import numpy as np
from datetime import datetime
import threading
import msvcrt  # Windows 键盘输入
from dobot_api import DobotApiDashboard, DobotApiFeedBack
from camera_recorder import D415CameraRecorder
from lerobot_dataset_saver import LeRobotDatasetSaver
from gripper import Robotiq2F85

# 注意：使用 LeRobotDatasetSaver 类而不是 create_lerobot_saver
# 因为我们已经修改了 lerobot_dataset_saver.py 支持 pi0 格式

# 配置参数 / Configuration
ROBOT_IP = "192.168.5.1"  # 请修改为你的机器人IP / Please modify to your robot IP
DASHBOARD_PORT = 29999
FEEDBACK_PORT = 30004
GRIPPER_PORT = "COM5"  # 夹爪串口号
GRIPPER_THRESHOLD_MM = 50.0  # 夹爪开口宽度阈值（毫米），<= 50mm为闭合(1)，> 50mm为打开(0)

# 全局变量用于信号处理
dashboard = None
feedback = None
camera_recorder = None
gripper = None  # 夹爪控制器
gripper_state = 0.0  # 当前夹爪状态: 0.0=打开, 1.0=闭合
gripper_lock = threading.Lock()  # 线程锁,保护gripper_state
data_saved = False  # 防止重复保存的标志
lerobot_saver = None  # LeRobot格式保存器
keyboard_thread_running = False  # 键盘线程运行标志

def save_data_and_exit(signum=None, frame=None):
    """信号处理函数：保存数据并退出"""
    global dashboard, feedback, camera_recorder, data_saved, lerobot_saver, keyboard_thread_running
    
    # 停止键盘监听线程
    keyboard_thread_running = False
    
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
    
    # 保存LeRobot格式数据（pi0 格式）
    if lerobot_saver:
        try:
            lerobot_saver.save_episode()  # 保存当前episode
            lerobot_dataset_path = lerobot_saver.finalize_dataset()
            print(f"✓ pi0 格式数据已保存到: {lerobot_dataset_path}")
            
            # 创建数据集README
            readme_path = lerobot_saver.create_dataset_card()
            print(f"✓ 数据集README已创建: {readme_path}")
            
            # 验证数据集格式
            if lerobot_saver.validate_dataset():
                print("✓ 数据集格式验证通过")
            else:
                print("⚠ 数据集格式验证失败")
            
            print("\n" + "="*60)
            print("✓ 数据保存完成！")
            print("="*60)
            print("\n✨ pi0 格式特性:")
            print("  ✓ 关节角度已转换为弧度制")
            print("  ✓ 动作已计算为关节速度（rad/s）")
            print("  ✓ 符合 pi0 训练格式")
            print("\n📊 数据统计:")
            print(f"  Episodes: {lerobot_saver.current_episode_index}")
            print(f"  总帧数: {lerobot_saver.total_frames}")
            print(f"  采集频率: {lerobot_saver.fps} Hz")
            print("\n🚀 下一步:")
            print("  如需采集更多数据，再次运行此程序")
            print("  如需开始微调，请运行:")
            print(f"    cd /home/huang/learn_arm_robot/openpi")
            print(f"    python scripts/train_pytorch.py \\")
            print(f"      --pretrained-checkpoint pi05_droid \\")
            print(f"      --dataset-repo-id {lerobot_dataset_path} \\")
            print(f"      --use-lora --lora-rank 8")
            print("="*60)
            
        except Exception as e:
            print(f"⚠ 保存LeRobot数据时出错: {e}")
    
    # 清理机器人连接
    # 分别处理每个步骤，确保失能命令一定执行
    if dashboard:
        try:
            print("正在停止拖拽模式...")
            dashboard.StopDrag()
            print("✓ 拖拽模式已停止")
        except Exception as e:
            print(f"⚠ 停止拖拽模式时出错: {e}")
        
        try:
            print("正在失能机器人...")
            dashboard.DisableRobot()
            print("✓ 机器人已失能")
        except Exception as e:
            print(f"⚠ 失能机器人时出错: {e}")
        
        try:
            dashboard.close()
            print("✓ Dashboard连接已关闭")
        except Exception as e:
            print(f"⚠ 关闭Dashboard连接时出错: {e}")
    
    if feedback:
        try:
            feedback.close()
            print("✓ Feedback连接已关闭")
        except Exception as e:
            print(f"⚠ 关闭Feedback连接时出错: {e}")
    
    # 清理相机资源
    try:
        if camera_recorder:
            camera_recorder.cleanup()
            print("✓ 相机资源已清理")
    except Exception as e:
        print(f"清理相机资源时出错: {e}")
    
    # 清理夹爪连接
    try:
        if gripper:
            gripper.close()
            print("✓ 夹爪连接已关闭")
    except Exception as e:
        print(f"清理夹爪连接时出错: {e}")
    
    print("程序已安全退出")
    sys.exit(0)

def keyboard_listener():
    """键盘监听线程：监听夹爪控制按键"""
    global gripper, gripper_state, gripper_lock, keyboard_thread_running
    
    print("\n" + "="*60)
    print("⌨️  键盘控制说明:")
    print("  按 'O' 键 → 打开夹爪 (Open)")
    print("  按 'C' 键 → 关闭夹爪 (Close)")
    print("  按 'Ctrl+C' → 结束录制并保存数据")
    print("="*60 + "\n")
    
    keyboard_thread_running = True
    
    while keyboard_thread_running:
        try:
            if msvcrt.kbhit():  # 检查是否有按键
                # 读取原始字节,避免解码问题
                key_bytes = msvcrt.getch()
                
                try:
                    # 尝试解码为字符
                    key = key_bytes.decode('utf-8').upper()
                    
                    if key == 'O':  # 打开夹爪
                        if gripper:
                            try:
                                print(f"\n🔓 [按键 'O'] 正在打开夹爪...")
                                # 对调：打开夹爪实际调用 close_gripper
                                gripper.close_gripper(speed=100, force=170)
                                with gripper_lock:
                                    gripper_state = 0.0
                                print("✓ 夹爪打开命令已发送")
                            except Exception as e:
                                print(f"⚠ 打开夹爪失败: {e}")
                                import traceback
                                traceback.print_exc()
                        else:
                            print("⚠ 夹爪未初始化")
                    
                    elif key == 'C':  # 关闭夹爪
                        if gripper:
                            try:
                                print(f"\n🔒 [按键 'C'] 正在关闭夹爪...")
                                # 对调：关闭夹爪实际调用 open_gripper
                                gripper.open_gripper(speed=255, force=200, wait=0.1)
                                with gripper_lock:
                                    gripper_state = 1.0
                                print("✓ 夹爪关闭命令已发送")
                            except Exception as e:
                                print(f"⚠ 关闭夹爪失败: {e}")
                                import traceback
                                traceback.print_exc()
                        else:
                            print("⚠ 夹爪未初始化")
                    else:
                        # 调试:显示按下的键
                        print(f"[Debug] 按下未映射的键: '{key}' (ASCII: {ord(key)})")
                
                except UnicodeDecodeError:
                    # 特殊键(方向键等)无法解码,忽略
                    pass
            
            time.sleep(0.05)  # 短暂休眠，避免CPU占用过高
            
        except Exception as e:
            # 打印其他异常,便于调试
            print(f"[键盘监听异常] {e}")
            import traceback
            traceback.print_exc()

def main():
    """主程序 / Main program"""
    global dashboard, feedback, camera_recorder, lerobot_saver, gripper, gripper_state, gripper_lock
    
    # 注册信号处理器
    signal.signal(signal.SIGINT, save_data_and_exit)  # Ctrl+C
    signal.signal(signal.SIGTERM, save_data_and_exit)  # 终止信号
    
    try:
        # 1. 初始化LeRobot数据集保存器（pi0 格式）
        print("\n1. 初始化LeRobot数据集保存器（pi0 格式）...")
        
        # 创建时间戳
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        repo_id = f"cr5_drag_teach_{timestamp}"
        
        # 使用修改后的保存器（已支持 pi0 格式）
        lerobot_saver = LeRobotDatasetSaver(
            repo_id=repo_id,
            fps=10,
            robot_type="dobot_cr5"
        )
        
        # 开始第一个 episode
        lerobot_saver.start_episode(task_name="teach_drag")
        
        print("✓ LeRobot数据集保存器初始化成功（pi0 格式）!")
        print(f"  数据集: {repo_id}")
        print(f"  特点: 自动转换角度→弧度，使用TCP实际速度")
        
        # 2. 初始化Robotiq 2F-85夹爪
        print("\n2. 初始化Robotiq 2F-85夹爪...")
        
        try:
            gripper = Robotiq2F85(port=GRIPPER_PORT, debug=False)
            gripper.activate()
            print("✓ 夹爪初始化成功!")
            print(f"  默认状态: 打开 (gripper_state = 0.0)")
            print(f"  控制方式: 按键控制 (O=打开, C=关闭)")
            
            # 初始化夹爪状态为打开
            with gripper_lock:
                gripper_state = 0.0
            
            # 启动键盘监听线程
            keyboard_thread = threading.Thread(target=keyboard_listener, daemon=True)
            keyboard_thread.start()
            print("✓ 键盘监听线程已启动")
            
        except Exception as e:
            print(f"⚠ 夹爪初始化失败: {e}")
            print("  将不记录夹爪数据（action.gripper 将为 0）")
            gripper = None
        
        # 3. 初始化深度相机
        print("\n3. 初始化深度相机...")
        
        camera_recorder = D415CameraRecorder(width=640, height=480, fps=30)
        camera_available = camera_recorder.start_recording(".")
        
        if camera_available:
            print("✓ 深度相机初始化成功!")
        else:
            print("⚠ 深度相机初始化失败，将只记录机器人数据")
            camera_recorder = None
        
        # 4. 连接机器人
        print("\n4. 连接机器人...")
        
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
                        # 获取关节速度 (QDActual) - 直接从机器人获取！
                        joint_velocities = list(feed_data['QDActual'][0])
                        
                        # 获取夹爪状态（从全局变量读取，由键盘线程控制）
                        with gripper_lock:
                            current_gripper_state = gripper_state
                        
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
                            'Rz': round(tool_position[5], 3),
                            # 添加实际关节速度（角度/秒）
                            'J1_vel': round(joint_velocities[0], 6),
                            'J2_vel': round(joint_velocities[1], 6),
                            'J3_vel': round(joint_velocities[2], 6),
                            'J4_vel': round(joint_velocities[3], 6),
                            'J5_vel': round(joint_velocities[4], 6),
                            'J6_vel': round(joint_velocities[5], 6),
                            # 添加夹爪状态（0=打开，1=闭合）- 由键盘控制
                            'gripper_state': current_gripper_state
                        }
                        
                        # 记录相机数据（如果可用）
                        lerobot_camera_data = None
                        camera_available = False
                        
                        if camera_recorder and camera_recorder.is_recording:
                            # 使用新的方法同时获取图像数据和保存路径
                            camera_frame_data = camera_recorder.capture_frame_with_data(record_count, current_timestamp)
                            if camera_frame_data:
                                camera_available = True
                                
                                # 获取实际的相机图像数据
                                color_image = camera_frame_data.get('color_image_data')
                                depth_image = camera_frame_data.get('depth_image_data')
                                
                                if color_image is not None and depth_image is not None:
                                    # 确保深度图像是正确的格式
                                    if len(depth_image.shape) == 2:
                                        # 将单通道深度图转换为3通道以适配LeRobot格式
                                        depth_image_3ch = np.stack([depth_image] * 3, axis=-1)
                                    else:
                                        depth_image_3ch = depth_image
                                    
                                    lerobot_camera_data = {
                                        "color_image": color_image,
                                        "depth_image": depth_image_3ch,
                                    }
                                else:
                                    # 备用：使用空白图像
                                    lerobot_camera_data = {
                                        "color_image": np.zeros((480, 640, 3), dtype=np.uint8),
                                        "depth_image": np.zeros((480, 640, 3), dtype=np.uint8),
                                    }
                            else:
                                # 如果capture_frame_with_data失败，回退到原始方法
                                camera_recorder.capture_frame(record_count, current_timestamp)
                        
                        # 添加到LeRobot数据集
                        if lerobot_saver:
                            # 确保相机数据不为None
                            if lerobot_camera_data is None:
                                lerobot_camera_data = {
                                    "color_image": np.zeros((480, 640, 3), dtype=np.uint8),
                                    "depth_image": np.zeros((480, 640, 3), dtype=np.uint8),
                                }
                            lerobot_saver.add_frame(position_data, lerobot_camera_data)
                        
                        record_count += 1
                        
                        # 显示当前位置和速度
                        camera_status = f" | 相机: {'✓' if camera_available else '✗'}" if camera_recorder else ""
                        gripper_status = f" | 夹爪: {'🔒 闭合' if current_gripper_state == 1.0 else '🔓 打开'}" if gripper else ""
                        
                        # 计算速度范数（判断是否在运动）
                        vel_norm = sum(abs(v) for v in joint_velocities)
                        moving_status = "🚀" if vel_norm > 0.5 else "⏸️"
                        
                        print(f"记录点 {record_count:3d} {moving_status}: "
                              f"关节[{joint_angles[0]:6.1f}°,{joint_angles[1]:6.1f}°,"\
                              f"{joint_angles[2]:6.1f}°,{joint_angles[3]:6.1f}°,"\
                              f"{joint_angles[4]:6.1f}°,{joint_angles[5]:6.1f}°] "
                              f"速度[{joint_velocities[0]:5.1f}°/s,{joint_velocities[1]:5.1f}°/s] "
                              f"{camera_status}{gripper_status}")
                    
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