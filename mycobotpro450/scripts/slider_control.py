#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
slider_control_450.py 
Gazebo仿真滑块控制脚本 - MyCobot450版本
功能：滑块 -> Gazebo 控制器
支持频率控制优化性能，减少卡顿
"""
import time
import math
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# 全局变量
pub_arm = None
pub_gripper = None

# 优化参数
ANGLE_THRESHOLD = 3.0           # 角度变化阈值(度)
GRIPPER_THRESHOLD = 5.0         # 夹爪角度变化阈值(度)  
MAX_COMMAND_RATE = 10.0         # 最大命令频率(Hz)

# 安全角度限制 (度)
JOINT_LIMITS = [
    (-180, 180),  # joint1
    (-180, 180),  # joint2
    (-180, 180),  # joint3
    (-180, 180),  # joint4
    (-180, 180),  # joint5
    (-180, 180),  # joint6
]

GRIPPER_LIMITS = (-62, 62)  # 夹爪角度限制

# 夹爪配置
GAZEBO_MIN_POSITION = -60.0  
GAZEBO_MAX_POSITION = 60.0   

# 状态记录
last_angles = None
last_gripper_angle = None
last_command_time = 0

# 超限警告控制
last_warning_time = {}  # 每个关节的最后警告时间
WARNING_INTERVAL = 3.0  # 警告间隔(秒)

# 统计信息
stats = {
    'total_messages': 0,
    'commands_sent': 0,
    'commands_skipped': 0,
    'limit_violations': 0,
}

# 期望的关节名称
ARM_JOINTS = [
    "joint1",
    "joint2", 
    "joint3",
    "joint4",
    "joint5",
    "joint6",
]
GRIPPER_JOINT = "gripper_controller"

def check_angle_limits(angles, gripper_angle):
    """检查角度是否在安全范围内"""
    global last_warning_time, stats
    
    current_time = time.time()
    violations = []
    
    # 检查关节角度限制
    for i, (angle, (min_limit, max_limit)) in enumerate(zip(angles, JOINT_LIMITS)):
        if angle < min_limit or angle > max_limit:
            joint_key = f"joint{i+1}"
            
            # 控制警告频率
            if joint_key not in last_warning_time or current_time - last_warning_time[joint_key] > WARNING_INTERVAL:
                violations.append(f"关节{i+1}: {angle:.1f}° (限制: {min_limit}°~{max_limit}°)")
                last_warning_time[joint_key] = current_time
                stats['limit_violations'] += 1
    
    # 检查夹爪角度限制
    min_grip, max_grip = GRIPPER_LIMITS
    if gripper_angle < min_grip or gripper_angle > max_grip:
        gripper_key = "gripper"
        
        if gripper_key not in last_warning_time or current_time - last_warning_time[gripper_key] > WARNING_INTERVAL:
            violations.append(f"夹爪: {gripper_angle:.1f}° (限制: {min_grip}°~{max_grip}°)")
            last_warning_time[gripper_key] = current_time
            stats['limit_violations'] += 1
    
    # 打印警告信息
    if violations:
        rospy.logwarn(f"[slider_control] ⚠️  角度超限:")
        for violation in violations:
            rospy.logwarn(f"[slider_control]    {violation}")
    
    return len(violations) == 0

def clamp_angles(angles, gripper_angle):
    """将角度限制在安全范围内"""
    # 限制关节角度
    clamped_angles = []
    for i, (angle, (min_limit, max_limit)) in enumerate(zip(angles, JOINT_LIMITS)):
        clamped_angle = max(min_limit, min(max_limit, angle))
        clamped_angles.append(clamped_angle)
    
    # 限制夹爪角度
    min_grip, max_grip = GRIPPER_LIMITS
    clamped_gripper = max(min_grip, min(max_grip, gripper_angle))
    
    return clamped_angles, clamped_gripper

def calculate_angle_difference(angles1, angles2):
    """计算角度差异"""
    if angles1 is None or angles2 is None:
        return float('inf')
    return sum(abs(a - b) for a, b in zip(angles1, angles2))

def should_send_command(new_angles, new_gripper_angle):
    """判断是否应该发送命令"""
    global last_angles, last_gripper_angle, last_command_time
    
    current_time = time.time()
    
    # 频率限制
    if current_time - last_command_time < 1.0 / MAX_COMMAND_RATE:
        return False, "频率限制"
    
    # 角度变化检查
    angle_diff = calculate_angle_difference(new_angles, last_angles)
    gripper_diff = abs(new_gripper_angle - last_gripper_angle) if last_gripper_angle is not None else float('inf')
    
    if angle_diff < ANGLE_THRESHOLD and gripper_diff < GRIPPER_THRESHOLD:
        return False, f"角度变化太小 (臂:{angle_diff:.1f}°, 夹爪:{gripper_diff:.1f}°)"
    
    return True, "允许发送"

def callback(msg: JointState):
    """优化的回调函数"""
    global stats, last_angles, last_gripper_angle, last_command_time
    
    stats['total_messages'] += 1
    
    # 快速解析关节数据
    arm_deg = [0.0] * len(ARM_JOINTS)
    grip_deg = 0.0
    
    name_to_deg = {name: math.degrees(pos) for name, pos in zip(msg.name, msg.position)}
    
    # 提取臂关节角度
    for i, joint_name in enumerate(ARM_JOINTS):
        if joint_name in name_to_deg:
            arm_deg[i] = round(name_to_deg[joint_name], 1)
    
    # 提取夹爪角度
    if GRIPPER_JOINT in name_to_deg:
        grip_deg = round(name_to_deg[GRIPPER_JOINT], 1)
    
    # 检查角度限制
    check_angle_limits(arm_deg, grip_deg)
    
    # 判断是否应该发送命令
    should_send, reason = should_send_command(arm_deg, grip_deg)
    
    if should_send:
        # 对角度进行限制
        clamped_arm, clamped_grip = clamp_angles(arm_deg, grip_deg)
        publish_to_gazebo(clamped_arm, clamped_grip)
        
        # 更新状态
        last_angles = clamped_arm.copy()
        last_gripper_angle = clamped_grip
        last_command_time = time.time()
        stats['commands_sent'] += 1
    else:
        stats['commands_skipped'] += 1
        rospy.logdebug(f"[slider_control] 跳过命令: {reason}")

def publish_to_gazebo(arm_deg, grip_deg):
    """发布到Gazebo"""
    global pub_arm, pub_gripper
    
    try:
        # 臂关节轨迹
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [math.radians(d) for d in arm_deg]
        pt.time_from_start = rospy.Duration(0.2)
        traj.points = [pt]
        pub_arm.publish(traj)
        
        # 夹爪轨迹
        traj_g = JointTrajectory()
        traj_g.header.stamp = rospy.Time.now()
        traj_g.joint_names = [GRIPPER_JOINT]
        ptg = JointTrajectoryPoint()
        ptg.positions = [math.radians(grip_deg)]
        ptg.time_from_start = rospy.Duration(0.2)
        traj_g.points = [ptg]
        pub_gripper.publish(traj_g)
        
        rospy.logdebug(f"[Gazebo] 发布: 臂{[round(a,1) for a in arm_deg]}, 夹爪{grip_deg:.1f}°")
        
    except Exception as e:
        rospy.logwarn(f"[Gazebo] 发布失败: {e}")

def print_stats():
    """打印统计信息"""
    if stats['total_messages'] > 0:
        efficiency = (stats['commands_sent'] / stats['total_messages']) * 100
        rospy.loginfo(f"[slider_control] 📊 统计: 消息:{stats['total_messages']}, "
                      f"发送:{stats['commands_sent']}, 跳过:{stats['commands_skipped']}, "
                      f"超限:{stats['limit_violations']}, "
                      f"效率:{efficiency:.1f}%")

def main():
    global pub_arm, pub_gripper
    
    rospy.init_node("slider_control_450", anonymous=True)
    
    print("\n" + "="*60)
    print("🎮 MyCobot450 Gazebo仿真滑块控制器")
    print("="*60)
    
    rospy.loginfo(f"[slider_control] 🎯 控制模式: Gazebo仿真")
    rospy.loginfo(f"[slider_control] ⚙️  配置: 角度阈值={ANGLE_THRESHOLD}°, "
                  f"最大频率={MAX_COMMAND_RATE}Hz")
    rospy.loginfo(f"[slider_control] 🛡️  安全限制: 关节±180°, 夹爪{GRIPPER_LIMITS[0]}°~{GRIPPER_LIMITS[1]}°")
    
    # 初始化发布器
    pub_arm = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)
    pub_gripper = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=1)
    rospy.loginfo("[slider_control] 📡 ROS发布器初始化完成")
    
    # 订阅关节状态
    rospy.Subscriber("/joint_states", JointState, callback, queue_size=1)
    rospy.loginfo("[slider_control] 📥 已订阅 /joint_states 话题")
    
    rospy.loginfo(f"[slider_control] 🚀 Gazebo控制器启动成功，等待滑块输入...")
    rospy.loginfo("[slider_control] 💡 Tips:")
    rospy.loginfo("[slider_control]    - 角度超限时会自动限制并显示警告")
    rospy.loginfo("[slider_control]    - 使用频率控制减少延迟和卡顿")
    rospy.loginfo("[slider_control]    - 按 Ctrl+C 安全退出")
    
    # 定期打印统计信息
    import threading
    def stats_timer():
        while not rospy.is_shutdown():
            time.sleep(15.0)  # 每15秒打印一次
            print_stats()
    
    stats_thread = threading.Thread(target=stats_timer, daemon=True)
    stats_thread.start()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[slider_control] 🛑 收到中断信号，正在关闭...")
    finally:
        print_stats()  # 最终统计
        rospy.loginfo("[slider_control] 👋 程序已安全退出")

if __name__ == "__main__":
    main()
