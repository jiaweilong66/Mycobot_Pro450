#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MyCobot450 Gazebo仿真键盘控制器
只包含Gazebo仿真控制，不涉及真实硬件
"""

import math
import rospy
import time
import sys
import select
import termios
import tty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ========================= 全局变量 =========================
pub_arm_command = None
pub_gripper_command = None
home_pose = [0, 0, 0, 0, 0, 0]

# 关节和夹爪配置 (MyCobot450)
ARM_JOINTS = [
    "joint1",
    "joint2", 
    "joint3",
    "joint4",
    "joint5",
    "joint6",
]
GRIPPER_JOINT = "gripper_controller"

# 安全角度限制 (度)
JOINT_LIMITS = [
    (-180, 180),  # joint1
    (-180, 180),  # joint2  
    (-180, 180),  # joint3
    (-180, 180),  # joint4
    (-180, 180),  # joint5
    (-180, 180),  # joint6
]

# 夹爪映射常量
GRIPPER_MIN_ANGLE = 0      
GRIPPER_MAX_ANGLE = 100    
GAZEBO_MIN_POSITION = -60.0  
GAZEBO_MAX_POSITION = 60.0   

# 控制参数
ANGLE_STEP = 5.0            # 每次按键的角度步长
FAST_STEP = 15.0            # 快速移动步长

# 当前状态
current_angles = [0, 0, 0, 0, 0, 0]
current_gripper_angle = 50  # 夹爪默认中间位置

# ========================= 安全检查 =========================
def clamp_angles(angles):
    """将角度限制在安全范围内"""
    clamped_angles = []
    for i, (angle, (min_limit, max_limit)) in enumerate(zip(angles, JOINT_LIMITS)):
        clamped_angle = max(min_limit, min(max_limit, angle))
        clamped_angles.append(clamped_angle)
    return clamped_angles

# ========================= Gazebo发布 =========================
def publish_arm_to_gazebo(angles):
    """发布机械臂轨迹到Gazebo"""
    global pub_arm_command
    
    try:
        if pub_arm_command is None:
            pub_arm_command = rospy.Publisher("/arm_controller/command", 
                                              JointTrajectory, queue_size=1)
            time.sleep(0.1)  # 等待发布者初始化
        
        arm_traj = JointTrajectory()
        arm_traj.header.stamp = rospy.Time.now()
        arm_traj.joint_names = ARM_JOINTS
        
        pt = JointTrajectoryPoint()
        pt.positions = [math.radians(a) for a in angles]
        pt.velocities = [0.0] * len(ARM_JOINTS)
        pt.accelerations = [0.0] * len(ARM_JOINTS)
        pt.time_from_start = rospy.Duration(0.5)
        arm_traj.points.append(pt)
        
        pub_arm_command.publish(arm_traj)
        rospy.logdebug(f"📤 Gazebo机械臂: {[round(a, 1) for a in angles]}°")
        
    except Exception as e:
        rospy.logwarn(f"Gazebo机械臂发布失败: {e}")

def publish_gripper_to_gazebo(gripper_angle):
    """发布夹爪轨迹到Gazebo"""
    global pub_gripper_command
    
    try:
        if pub_gripper_command is None:
            pub_gripper_command = rospy.Publisher("/gripper_controller/command",
                                                  JointTrajectory, queue_size=1)
            time.sleep(0.1)  # 等待发布者初始化
        
        # 夹爪角度映射：0-100° -> -60°到60°
        mapped_gripper = ((gripper_angle - GRIPPER_MIN_ANGLE) /
                         (GRIPPER_MAX_ANGLE - GRIPPER_MIN_ANGLE)) * \
                        (GAZEBO_MAX_POSITION - GAZEBO_MIN_POSITION) + GAZEBO_MIN_POSITION
        
        # 限制范围
        mapped_gripper = max(GAZEBO_MIN_POSITION, min(GAZEBO_MAX_POSITION, mapped_gripper))
        
        gripper_traj = JointTrajectory()
        gripper_traj.header.stamp = rospy.Time.now()
        gripper_traj.joint_names = [GRIPPER_JOINT]
        
        gp = JointTrajectoryPoint()
        gp.positions = [math.radians(mapped_gripper)]
        gp.velocities = [0.0]
        gp.accelerations = [0.0]
        gp.time_from_start = rospy.Duration(0.5)
        gripper_traj.points.append(gp)
        
        pub_gripper_command.publish(gripper_traj)
        rospy.logdebug(f"📤 Gazebo夹爪: {gripper_angle}° -> {mapped_gripper:.1f}°")
        
    except Exception as e:
        rospy.logwarn(f"Gazebo夹爪发布失败: {e}")

# ========================= 键盘输入 =========================
class RawTerminal:
    """原始终端模式上下文管理器"""
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.prev = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        termios.tcsetattr(self.fd, termios.TCSANOW, self.prev)

def get_key_non_blocking():
    """非阻塞获取按键"""
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None

# ========================= 键盘控制逻辑 =========================
def print_help():
    """打印帮助信息"""
    help_text = f"""
╔══════════════════════════════════════════════════════════╗
║     MyCobot450 Gazebo仿真键盘控制器                      ║
╚══════════════════════════════════════════════════════════╝

🔧 关节控制 (普通步长: {ANGLE_STEP}°, 快速步长: {FAST_STEP}°):
  ┌─────────────────────────────────────────────────┐
  │ w/s: joint1 +/-     W/S: joint1 +/-  (快速)    │
  │ e/d: joint2 +/-     E/D: joint2 +/-  (快速)    │
  │ r/f: joint3 +/-     R/F: joint3 +/-  (快速)    │
  │ t/g: joint4 +/-     T/G: joint4 +/-  (快速)    │
  │ y/h: joint5 +/-     Y/H: joint5 +/-  (快速)    │
  │ u/j: joint6 +/-     U/J: joint6 +/-  (快速)    │
  └─────────────────────────────────────────────────┘

🤏 夹爪控制:
  ┌─────────────────────────────────────────────────┐
  │ o: 夹爪完全打开 (100°)                          │
  │ p: 夹爪完全关闭 (0°)                            │
  │ [: 夹爪开启 +10°                                │
  │ ]: 夹爪关闭 -10°                                │
  └─────────────────────────────────────────────────┘

🏠 特殊命令:
  ┌─────────────────────────────────────────────────┐
  │ 1: 回到初始位置 (所有关节0°)                    │
  │ 2: 显示当前角度                                 │
  │ h: 显示此帮助信息                               │
  └─────────────────────────────────────────────────┘

❌ 退出:
  q: 退出程序

⚠️  安全提示:
  • 角度限制: ±180°
  • 程序会自动限制超出范围的角度
  • 仅控制Gazebo仿真，不涉及真实硬件
"""
    print(help_text)

def teleop_keyboard():
    """键盘控制主循环"""
    global current_angles, current_gripper_angle
    
    print_help()
    
    print(f"\n当前状态:")
    print(f"  关节角度: {[round(a, 1) for a in current_angles]}°")
    print(f"  夹爪角度: {current_gripper_angle}°\n")
    
    with RawTerminal():
        while not rospy.is_shutdown():
            key = get_key_non_blocking()
            
            if key is None:
                time.sleep(0.01)  # 短暂休眠减少CPU占用
                continue
            
            # 退出程序
            if key == 'q':
                print("\n👋 退出程序...")
                break
            
            # 显示帮助
            if key == 'h':
                print_help()
                continue
            
            # 回到初始位置
            if key == '1':
                current_angles = home_pose.copy()
                publish_arm_to_gazebo(current_angles)
                print(f"🏠 回到初始位置: {[round(a, 1) for a in current_angles]}°")
                continue
            
            # 显示当前角度
            if key == '2':
                print(f"📍 当前角度:")
                for i, angle in enumerate(current_angles):
                    print(f"   关节{i+1}: {angle:7.1f}°")
                print(f"   夹爪:   {current_gripper_angle:7.1f}°")
                continue
            
            # 夹爪控制
            if key == 'o':
                current_gripper_angle = 100
                publish_gripper_to_gazebo(current_gripper_angle)
                print(f"🤏 夹爪打开: {current_gripper_angle}°")
                continue
            elif key == 'p':
                current_gripper_angle = 0
                publish_gripper_to_gazebo(current_gripper_angle)
                print(f"🤏 夹爪关闭: {current_gripper_angle}°")
                continue
            elif key == '[':
                current_gripper_angle = min(100, current_gripper_angle + 10)
                publish_gripper_to_gazebo(current_gripper_angle)
                print(f"🤏 夹爪开启: {current_gripper_angle}°")
                continue
            elif key == ']':
                current_gripper_angle = max(0, current_gripper_angle - 10)
                publish_gripper_to_gazebo(current_gripper_angle)
                print(f"🤏 夹爪关闭: {current_gripper_angle}°")
                continue
            
            # 关节运动映射 (普通步长)
            normal_mapping = {
                'w': (0, +ANGLE_STEP), 's': (0, -ANGLE_STEP),
                'e': (1, +ANGLE_STEP), 'd': (1, -ANGLE_STEP),
                'r': (2, +ANGLE_STEP), 'f': (2, -ANGLE_STEP),
                't': (3, +ANGLE_STEP), 'g': (3, -ANGLE_STEP),
                'y': (4, +ANGLE_STEP), 'h': (4, -ANGLE_STEP),
                'u': (5, +ANGLE_STEP), 'j': (5, -ANGLE_STEP),
            }
            
            # 关节运动映射 (快速步长)
            fast_mapping = {
                'W': (0, +FAST_STEP), 'S': (0, -FAST_STEP),
                'E': (1, +FAST_STEP), 'D': (1, -FAST_STEP),
                'R': (2, +FAST_STEP), 'F': (2, -FAST_STEP),
                'T': (3, +FAST_STEP), 'G': (3, -FAST_STEP),
                'Y': (4, +FAST_STEP), 'H': (4, -FAST_STEP),
                'U': (5, +FAST_STEP), 'J': (5, -FAST_STEP),
            }
            
            # 检查普通步长映射
            if key in normal_mapping:
                idx, step = normal_mapping[key]
                current_angles[idx] += step
                current_angles = clamp_angles(current_angles)
                publish_arm_to_gazebo(current_angles)
                print(f"🔧 关节{idx+1}: {current_angles[idx]:7.1f}° (步长: {step:+.1f}°)")
                continue
            
            # 检查快速步长映射
            if key in fast_mapping:
                idx, step = fast_mapping[key]
                current_angles[idx] += step
                current_angles = clamp_angles(current_angles)
                publish_arm_to_gazebo(current_angles)
                print(f"🚀 关节{idx+1}: {current_angles[idx]:7.1f}° (快速: {step:+.1f}°)")
                continue

# ========================= 主函数 =========================
def main():
    rospy.init_node("mycobot450_gazebo_keyboard", anonymous=True)
    
    print("╔══════════════════════════════════════════════════════════╗")
    print("║     MyCobot450 Gazebo仿真键盘控制器                      ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print()
    print("🚀 正在初始化...")
    print(f"✅ ROS节点已启动")
    print(f"🎮 Gazebo控制: 已启用")
    print(f"🤖 机器型号: MyCobot450")
    print(f"📊 关节数量: {len(ARM_JOINTS)}")
    print()
    
    # 等待一下让发布者初始化
    time.sleep(0.5)
    
    # 发送初始位置
    publish_arm_to_gazebo(current_angles)
    publish_gripper_to_gazebo(current_gripper_angle)
    
    print("✨ 准备就绪! 按 'h' 查看帮助，按 'q' 退出\n")
    
    try:
        teleop_keyboard()
    except KeyboardInterrupt:
        print("\n🛑 接收到中断信号，正在关闭...")
    except Exception as e:
        rospy.logerr(f"❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("👋 程序已退出")

if __name__ == "__main__":
    main()
