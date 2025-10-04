#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
最简示教拖拽模式使用示例
Simplest Teach Drag Mode Usage Example

使用说明：
1. 修改IP地址为你的机器人IP
2. 运行程序
3. 按提示操作拖拽机器人
4. 程序会自动记录关节角度并保存

Usage Instructions:
1. Modify IP address to your robot IP
2. Run the program
3. Follow prompts to drag the robot
4. Program will automatically record joint angles and save them
"""

import time
import json
from datetime import datetime
from dobot_api import DobotApiDashboard, DobotApiFeedBack

# 配置参数 / Configuration
ROBOT_IP = "192.168.5.1"  # 请修改为你的机器人IP / Please modify to your robot IP
DASHBOARD_PORT = 29999
FEEDBACK_PORT = 30004

def main():
    """主程序 / Main program"""
    
    print("="*60)
    print("越疆机器人示教拖拽模式演示")
    print("="*60)
    print(f"机器人IP: {ROBOT_IP}")
    
    dashboard = None
    feedback = None
    recorded_positions = []
    
    try:
        # 1. 连接机器人
        print("\\n1. 连接机器人...")
        
        dashboard = DobotApiDashboard(ROBOT_IP, DASHBOARD_PORT)
        feedback = DobotApiFeedBack(ROBOT_IP, FEEDBACK_PORT)
        
        print("✓ 连接成功!")
        
        # 2. 使能机器人
        print("\\n2. 使能机器人...")
        
        result = dashboard.EnableRobot()
        print(f"使能结果: {result}")
        
        # 等待使能完成并检查状态
        print("\\n检查使能状态...")
        print("程序将持续检查使能状态，直到成功或用户按 Ctrl+C 终止")
        time.sleep(2)  # 等待使能开始
        
        # 持续检查机器人状态直到使能成功
        enable_success = False
        check_count = 0
        
        while not enable_success:
            try:
                check_count += 1
                feed_data = feedback.feedBackData()
                
                if feed_data is not None and len(feed_data) > 0:
                    robot_mode = feed_data['RobotMode'][0]
                    
                    # 每5次检查显示一次状态（避免刷屏）
                    if check_count % 5 == 1:
                        print(f"\\n第 {check_count} 次检查 - 机器人模式: {robot_mode}")
                        
                        # 显示模式含义
                        mode_descriptions = {
                            1: "ROBOT_MODE_INIT (初始化)",
                            2: "ROBOT_MODE_BRAKE_OPEN (抱闸打开)", 
                            4: "ROBOT_MODE_DISABLED (失能状态)",
                            5: "ROBOT_MODE_ENABLE (使能状态)",
                            7: "ROBOT_MODE_RUNNING (运行状态)",
                            9: "ROBOT_MODE_ERROR (报警状态)",
                            10: "ROBOT_MODE_PAUSE (暂停状态)"
                        }
                        
                        if robot_mode in mode_descriptions:
                            print(f"状态说明: {mode_descriptions[robot_mode]}")
                    
                    # 检查是否使能成功 (模式5=ENABLE, 模式7=RUNNING)
                    if robot_mode in [5, 7]:  # ROBOT_MODE_ENABLE or ROBOT_MODE_RUNNING
                        enable_success = True
                        print("\\n✓ 机器人使能成功!")
                        break
                        
                    elif robot_mode == 9:  # ROBOT_MODE_ERROR
                        print(f"\\n⚠️  机器人处于报警状态 (第{check_count}次检查)")
                        print("请清除报警后程序将继续检查，或按 Ctrl+C 退出")
                        
                    # 显示等待提示
                    if check_count % 10 == 0:
                        print(f"已检查 {check_count} 次，继续等待使能成功... (按 Ctrl+C 可终止)")
                
                time.sleep(1)  # 每秒检查一次
                
            except KeyboardInterrupt:
                print("\\n\\n用户手动终止使能检查")
                raise  # 重新抛出异常，让外层处理
                
            except Exception as e:
                if check_count % 5 == 1:  # 只在某些检查时显示错误
                    print(f"检查状态出错 (第{check_count}次): {e}")
                time.sleep(1)
        
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
        print("\\n4. 开始记录关节角度...")
        print("\\n操作说明:")
        print("- 拖拽机器人到不同位置")
        print("- 程序每0.1秒记录一次当前位置 (10Hz)")
        print("- 按 Ctrl+C 停止记录")
        
        # 记录循环
        record_count = 0
        start_time = time.time()
        
        while True:
            try:
                # 获取当前状态
                feed_data = feedback.feedBackData()
                
                if feed_data is not None and len(feed_data) > 0:
                    # 获取关节角度 (QActual)
                    joint_angles = list(feed_data['QActual'][0])
                    
                    # 获取末端位置 (ToolVectorActual) 
                    tool_position = list(feed_data['ToolVectorActual'][0])
                    
                    # 记录数据
                    position_data = {
                        'index': record_count,
                        'timestamp': time.time() - start_time,
                        'time_str': datetime.now().strftime('%H:%M:%S.%f')[:-3],
                        'joint_angles': [round(angle, 3) for angle in joint_angles],
                        'tool_position': [round(pos, 3) for pos in tool_position[:6]],
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
                          f"关节[{joint_angles[0]:6.1f}°,{joint_angles[1]:6.1f}°,"
                          f"{joint_angles[2]:6.1f}°,{joint_angles[3]:6.1f}°,"
                          f"{joint_angles[4]:6.1f}°,{joint_angles[5]:6.1f}°] "
                          f"位置[{tool_position[0]:7.1f},{tool_position[1]:7.1f},"
                          f"{tool_position[2]:7.1f}]")
                
                time.sleep(0.1)  # 每0.1秒记录一次 (10Hz)
                
            except KeyboardInterrupt:
                print("\\n\\n用户停止记录...")
                break
            
            except Exception as e:
                print(f"记录出错: {e}")
                time.sleep(0.1)
                continue
        
        # 5. 停止拖拽模式
        print("\\n5. 停止拖拽模式...")
        
        result = dashboard.StopDrag()
        print(f"停止拖拽结果: {result}")
        
        # 6. 保存记录的数据
        if recorded_positions:
            print("\\n6. 保存记录数据...")
            
            # 生成文件名
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f"teach_drag_record_{timestamp}.json"
            
            # 保存数据
            save_data = {
                'metadata': {
                    'robot_ip': ROBOT_IP,
                    'record_count': len(recorded_positions),
                    'duration': recorded_positions[-1]['timestamp'],
                    'start_time': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                    'description': '示教拖拽模式记录的关节角度和末端位置数据'
                },
                'positions': recorded_positions
            }
            
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(save_data, f, indent=2, ensure_ascii=False)
            
            print(f"✓ 数据已保存到: {filename}")
            
            # 显示统计信息
            print(f"\\n记录统计:")
            print(f"- 记录点数: {len(recorded_positions)}")
            print(f"- 记录时长: {recorded_positions[-1]['timestamp']:.1f} 秒")
            print(f"- 平均频率: {len(recorded_positions)/recorded_positions[-1]['timestamp']:.1f} Hz")
            
            # 显示起始和结束位置
            start_pos = recorded_positions[0]
            end_pos = recorded_positions[-1]
            
            print(f"\\n起始位置: 关节{start_pos['joint_angles']}, 末端{start_pos['tool_position'][:3]}")
            print(f"结束位置: 关节{end_pos['joint_angles']}, 末端{end_pos['tool_position'][:3]}")
            
            print(f"\\nStart position: Joints{start_pos['joint_angles']}, Tool{start_pos['tool_position'][:3]}")
            print(f"End position: Joints{end_pos['joint_angles']}, Tool{end_pos['tool_position'][:3]}")
        
    except KeyboardInterrupt:
        print("\\n用户中断程序...")
        print("User interrupted the program...")
    
    except Exception as e:
        print(f"\\n❌ 程序出错: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # 清理资源
        try:
            if dashboard:
                # 确保停止拖拽模式
                try:
                    dashboard.StopDrag()
                    print("确保拖拽模式已关闭")
                except:
                    pass
                
                # 失能机器人
                try:
                    print("\\n正在失能机器人...")
                    disable_result = dashboard.DisableRobot()
                    print(f"失能结果: {disable_result}")
                    time.sleep(1)
                    print("✓ 机器人已安全失能")
                except Exception as e:
                    print(f"失能失败: {e}")
                
                dashboard.close()
            
            if feedback:
                feedback.close()
            
            print("\\n连接已断开")
            
        except Exception as e:
            print(f"清理出错: {e}")


if __name__ == "__main__":
    print("\\n请确保:")
    print("1. 机器人已连接并处于TCP/IP模式")
    print("2. 网络连接正常")
    print("3. 机器人IP地址设置正确")
    print(f"4. 当前设置的IP地址是: {ROBOT_IP}")
    
    print("\\n如需修改IP地址，请编辑脚本开头的ROBOT_IP变量")
    
    input("\\n按回车键开始...")
    
    main()