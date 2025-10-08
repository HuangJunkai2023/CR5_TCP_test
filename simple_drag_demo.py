import time
import json
import signal
import sys
from datetime import datetime
from dobot_api import DobotApiDashboard, DobotApiFeedBack

# 配置参数 / Configuration
ROBOT_IP = "192.168.5.1"  # 请修改为你的机器人IP / Please modify to your robot IP
DASHBOARD_PORT = 29999
FEEDBACK_PORT = 30004

# 全局变量用于信号处理
recorded_positions = []
dashboard = None
feedback = None

def save_data_and_exit(signum=None, frame=None):
    """信号处理函数：保存数据并退出"""
    global recorded_positions, dashboard, feedback
    
    print("\n\n检测到退出信号，正在保存数据...")
    
    # 保存数据
    if recorded_positions:
        try:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f"teach_drag_record_{timestamp}.json"
            
            save_data = {
                'metadata': {
                    'robot_ip': ROBOT_IP,
                    'record_count': len(recorded_positions),
                    'duration': recorded_positions[-1]['timestamp'] if recorded_positions else 0,
                    'save_time': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                    'description': '示教拖拽模式记录的关节角度和末端位置数据'
                },
                'positions': recorded_positions
            }
            
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(save_data, f, indent=2, ensure_ascii=False)
            
            print(f"✓ 数据已保存到: {filename}")
            print(f"✓ 共保存 {len(recorded_positions)} 个记录点")
            
            # 显示统计信息
            if len(recorded_positions) > 0:
                start_pos = recorded_positions[0]
                end_pos = recorded_positions[-1]
                duration = end_pos['timestamp']
                avg_freq = len(recorded_positions) / duration if duration > 0 else 0
                
                print(f"\n记录统计:")
                print(f"- 记录点数: {len(recorded_positions)}")
                print(f"- 记录时长: {duration:.1f} 秒")
                print(f"- 平均频率: {avg_freq:.1f} Hz")
                print(f"- 起始位置: 关节{start_pos['joint_angles']}")
                print(f"- 结束位置: 关节{end_pos['joint_angles']}")
                
        except Exception as e:
            print(f"❌ 保存数据失败: {e}")
            # 尝试紧急保存
            try:
                emergency_filename = f"emergency_save_{int(time.time())}.json"
                with open(emergency_filename, 'w') as f:
                    json.dump({'positions': recorded_positions}, f)
                print(f"✓ 紧急保存成功: {emergency_filename}")
            except:
                print("❌ 紧急保存也失败了")
    
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
        print(f"清理连接时出错: {e}")
    
    print("程序已安全退出")
    sys.exit(0)

def main():
    """主程序 / Main program"""
    global recorded_positions, dashboard, feedback
    
    # 注册信号处理器
    signal.signal(signal.SIGINT, save_data_and_exit)  # Ctrl+C
    signal.signal(signal.SIGTERM, save_data_and_exit)  # 终止信号
    
    print("="*60)
    print("越疆机器人示教拖拽模式演示 (数据保护版)")
    print("="*60)
    print(f"机器人IP: {ROBOT_IP}")
    print("✓ 已启用数据保护：按Ctrl+C将自动保存所有数据")
    
    try:
        # 1. 连接机器人
        print("\\n1. 连接机器人...")
        
        dashboard = DobotApiDashboard(ROBOT_IP, DASHBOARD_PORT)
        feedback = DobotApiFeedBack(ROBOT_IP, FEEDBACK_PORT)
        
        print("✓ 连接成功!")
        
        # 2. 清除错误状态
        print("\n2. 清除机器人错误状态...")
        
        try:
            result = dashboard.ClearError()
            print(f"清除错误结果: {result}")
            time.sleep(1)  # 等待错误清除完成
            print("✓ 错误状态已清除")
        except Exception as e:
            print(f"⚠ 清除错误失败: {e}")
            
        # 简化状态检查（可选）
        print("✓ 错误清除完成，准备使能机器人")
        
        # 3. 使能机器人
        print("\n3. 使能机器人...")
        
        result = dashboard.EnableRobot()
        print(f"使能结果: {result}")
        time.sleep(3)  # 等待使能完成
        
        # 3. 开启拖拽模式
        print("\\n3. 开启拖拽模式...")
        
        result = dashboard.StartDrag()
        print(f"拖拽模式结果: {result}")
        time.sleep(1)
        
        print("\\n" + "="*50)
        print("✓ 拖拽模式已开启!")
        print("现在可以手动拖拽机器人了!")
        print("="*50)
        
        # 4. 记录关节角度
        print("\n4. 开始记录关节角度...")
        print("\n操作说明:")
        print("- 程序每0.1秒记录一次当前位置 (10Hz)")
        print("- 按 Ctrl+C 停止记录并自动保存数据")
        
        # 记录循环
        record_count = 0
        start_time = time.time()
        
        try:
            while True:
                try:
                    # 获取当前状态
                    feed_data = feedback.feedBackData()
                    
                    if feed_data is not None and len(feed_data) > 0:
                        # 获取关节角度 (QActual)
                        joint_angles = list(feed_data['QActual'][0])
                        # 获取末端位置 (ToolVectorActual)
                        tool_position = list(feed_data['ToolVectorActual'][0])
                        # 只保留单独字段
                        position_data = {
                            'index': record_count,
                            'timestamp': time.time() - start_time,
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
                        recorded_positions.append(position_data)
                        record_count += 1
                        # 显示当前位置
                        print(f"记录点 {record_count:3d}: "
                              f"关节[{joint_angles[0]:6.1f}°,{joint_angles[1]:6.1f}°,"\
                              f"{joint_angles[2]:6.1f}°,{joint_angles[3]:6.1f}°,"\
                              f"{joint_angles[4]:6.1f}°,{joint_angles[5]:6.1f}°] "\
                              f"位置[{tool_position[0]:7.1f},{tool_position[1]:7.1f},"\
                              f"{tool_position[2]:7.1f}]")
                    
                    time.sleep(0.1)  # 每0.1秒记录一次 (10Hz)
                    
                except Exception as e:
                    # 如果是网络断开错误，直接触发保存并退出
                    if "10054" in str(e) or "连接" in str(e):
                        print(f"\n网络连接断开: {e}")
                        print("正在保存已记录的数据...")
                        save_data_and_exit()
                    else:
                        print(f"记录出错: {e}")
                        time.sleep(0.1)
                        continue
                        
        except KeyboardInterrupt:
            # Ctrl+C 会被信号处理器自动处理
            save_data_and_exit()
    
    except KeyboardInterrupt:
        # 外层的 Ctrl+C 处理
        save_data_and_exit()
    
    except Exception as e:
        print(f"\n❌ 程序出错: {e}")
        # 即使出错也要保存数据
        if recorded_positions:
            save_data_and_exit()
        import traceback
        traceback.print_exc()
    
    finally:
        # 如果程序正常结束（没有被信号处理器处理），确保保存数据
        if recorded_positions:
            save_data_and_exit()


if __name__ == "__main__":
    print("\\n请确保:")
    print("1. 机器人已连接并处于TCP/IP模式")
    print("2. 网络连接正常")
    print("3. 机器人IP地址设置正确")
    print(f"4. 当前设置的IP地址是: {ROBOT_IP}")
    
    print("\\n如需修改IP地址，请编辑脚本开头的ROBOT_IP变量")
    
    input("\\n按回车键开始...")
    
    main()