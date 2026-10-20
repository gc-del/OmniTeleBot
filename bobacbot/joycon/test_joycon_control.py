#!/usr/bin/env python3
"""
Joy-Con 右手柄控制输出测试脚本

用于测试和调试 Joy-Con 右手柄的 get_control() 输出。
显示姿态、夹爪状态和按钮控制信号。

使用方法:
    python test_joycon_control.py
    
按 Ctrl+C 退出
"""

import sys
import time
import os
import math

# 添加父目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from joyconrobotics.joyconrobotics import JoyconRobotics


def print_header():
    """打印表头"""
    print("\n" + "="*80)
    print("Joy-Con 右手柄控制输出测试")
    print("="*80)
    print("姿态格式: [x, y, z, roll, pitch, yaw]")
    print("  - 位置 (x, y, z): 单位米 (m)")
    print("  - 姿态 (roll, pitch, yaw): 单位弧度 (rad)")
    print("夹爪状态: 0.0 (关闭) ~ 1.0 (打开)")
    print("按钮控制: 0 (无操作), 1 (下一集), -1 (重启集), 其他值见文档")
    print("="*80)
    print("\n提示: 首次使用会进行2秒校准，请将手柄平放在桌面上\n")


def format_control_data(posture, gripper_state, button_control, joycon_robot):
    """格式化控制数据"""
    lines = []
    
    # 姿态信息
    lines.append(f"\n{'─'*80}")
    lines.append(f"📍 姿态 (Posture):")
    lines.append(f"   位置: x={posture[0]:7.4f} m, y={posture[1]:7.4f} m, z={posture[2]:7.4f} m")
    lines.append(f"   姿态: roll={posture[3]:7.4f} rad, pitch={posture[4]:7.4f} rad, yaw={posture[5]:7.4f} rad")
    
    # 转换为角度显示
    roll_deg = math.degrees(posture[3])
    pitch_deg = math.degrees(posture[4])
    yaw_deg = math.degrees(posture[5])
    lines.append(f"         roll={roll_deg:7.2f}°, pitch={pitch_deg:7.2f}°, yaw={yaw_deg:7.2f}°")
    
    # 夹爪状态
    gripper_status = "🔴 关闭" if gripper_state < 0.5 else "🟢 打开"
    lines.append(f"\n🤏 夹爪 (Gripper): {gripper_state:.2f} {gripper_status}")
    
    # 按钮控制
    button_desc = {
        0: "无操作",
        1: "下一集 (A键)",
        -1: "重启集 (Y键)",
        2: "B键",
        3: "X键",
        4: "SL键",
        5: "SR键",
        6: "ZR/ZL键",
        8: "重置"
    }
    desc = button_desc.get(button_control, f"自定义 ({button_control})")
    lines.append(f"🎮 按钮控制 (Button Control): {button_control:3d} - {desc}")
    
    # 原始传感器数据
    lines.append(f"\n📊 原始传感器数据:")
    
    # 摇杆数据
    stick_vertical = joycon_robot.joycon.get_stick_right_vertical()
    stick_horizontal = joycon_robot.joycon.get_stick_right_horizontal()
    lines.append(f"   右摇杆: 垂直={stick_vertical:5d}, 水平={stick_horizontal:5d}")
    
    # 按钮状态
    buttons = {
        'R': joycon_robot.joycon.get_button_r(),
        'ZR': joycon_robot.joycon.get_button_zr(),
        'A': joycon_robot.joycon.get_button_a(),
        'B': joycon_robot.joycon.get_button_b(),
        'X': joycon_robot.joycon.get_button_x(),
        'Y': joycon_robot.joycon.get_button_y(),
        'Home': joycon_robot.joycon.get_button_home(),
        'Plus': joycon_robot.joycon.get_button_plus(),
        'R-Stick': joycon_robot.joycon.get_button_r_stick(),
        'SR': joycon_robot.joycon.get_button_right_sr(),
        'SL': joycon_robot.joycon.get_button_right_sl(),
    }
    
    active_buttons = [name for name, state in buttons.items() if state]
    if active_buttons:
        lines.append(f"   按下按钮: {', '.join(active_buttons)}")
    else:
        lines.append(f"   按下按钮: 无")
    
    lines.append(f"{'─'*80}")
    
    return "\n".join(lines)


def main():
    """主函数"""
    
    print_header()
    
    try:
        # 初始化 Joy-Con 右手柄
        print("正在连接右手柄...")
        joycon_robot = JoyconRobotics(
            device="right",
            gripper_open=1.0,
            gripper_close=0.0,
            limit_dof=True,
            all_button_return=True  # 启用所有按钮返回
        )
        print("✅ 右手柄连接成功！\n")
        
        # 主循环
        loop_count = 0
        start_time = time.time()
        
        while True:
            # 获取控制数据
            posture, gripper_state, button_control = joycon_robot.get_control()
            
            # 清除屏幕（可选，注释掉以保留历史记录）
            # os.system('cls' if os.name == 'nt' else 'clear')
            
            # 打印控制数据（包含原始传感器数据）
            output = format_control_data(posture, gripper_state, button_control, joycon_robot)
            print(output)
            
            # 显示运行统计
            loop_count += 1
            elapsed_time = time.time() - start_time
            fps = loop_count / elapsed_time if elapsed_time > 0 else 0
            print(f"\n⏱️  运行时间: {elapsed_time:.2f}s | 循环次数: {loop_count} | FPS: {fps:.1f}")
            print(f"\n按 Ctrl+C 退出...\n")
            
            # 短暂休眠，避免过高频率
            time.sleep(0.05)  # 20 Hz
            
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断，正在退出...")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 断开连接
        try:
            joycon_robot.disconnnect()
            print("✅ 手柄已断开连接")
        except:
            pass


if __name__ == "__main__":
    main()
