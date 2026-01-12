#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import math
import threading
import queue
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pymycobot import Pro450Client

try:
    import moveit_commander
    from moveit_msgs.msg import RobotState, PlanningScene
    from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False

mc = None
mode = 2
pub_arm = None
pub_gripper = None
current_end_effector_coords = None
coords_lock = threading.Lock()
is_stopped = False
stop_lock = threading.Lock()
robot_commander = None
move_group = None
planning_scene_interface = None
state_validity_service = None
MOVEIT_COLLISION_CHECK = False

ANGLE_THRESHOLD = 3.0
GRIPPER_THRESHOLD = 5.0
MAX_COMMAND_RATE = 10.0
COMMAND_QUEUE_SIZE = 5
PRO450_IP = "192.168.0.232"
PRO450_PORT = 4500

JOINT_LIMITS = [(-162, 162), (-125, 125), (-154, 154), (-162, 162), (-162, 162), (-165, 165)]
GRIPPER_LIMITS = (0, 57.3)
GRIPPER_ID = 14
GAZEBO_MIN_POSITION = 0
GAZEBO_MAX_POSITION = 57.3
PRO450_GRIPPER_MIN = 0
PRO450_GRIPPER_MAX = 100

last_angles = None
last_gripper_angle = None
last_command_time = 0
command_queue = queue.Queue(maxsize=COMMAND_QUEUE_SIZE)
is_executing = False
last_warning_time = {}
WARNING_INTERVAL = 3.0

stats = {'total_messages': 0, 'commands_sent': 0, 'commands_skipped': 0, 'limit_violations': 0, 'errors': 0}

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
GRIPPER_JOINT = "gripper_controller"

COLLISION_CHECK_ENABLED = True
last_collision_warning_time = 0
COLLISION_WARNING_INTERVAL = 2.0
MOVEIT_GROUP_NAME = "arm"
PRO450_JOINT_LIMITS = [(-162, 162), (-125, 125), (-154, 154), (-162, 162), (-162, 162), (-165, 165)]

GRIPPER_LENGTH = 0.17
MIN_END_HEIGHT = 0.28
MIN_END_HEIGHT_WARNING = 0.35
BASE_RADIUS = 0.12
MIN_SAFE_HEIGHT_MM = 170

def estimate_end_effector_height(j2, j3, j4):
    j2_rad = math.radians(j2)
    j3_rad = math.radians(j3)
    j4_rad = math.radians(j4)
    L1, L2, L3, L4, L5 = 0.048, 0.18, 0.1735, 0.08, GRIPPER_LENGTH
    angle2 = j2_rad
    angle3 = angle2 + j3_rad
    angle4 = angle3 + j4_rad
    height = 0.155 + L1
    height += L2 * math.cos(angle2)
    height += L3 * math.cos(angle3)
    height += L4 * math.cos(angle4)
    height += L5 * math.cos(angle4)
    return height

def estimate_end_effector_distance_to_base(j2, j3, j4):
    j2_rad, j3_rad, j4_rad = math.radians(j2), math.radians(j3), math.radians(j4)
    L2, L3, L4, L5 = 0.18, 0.1735, 0.08, GRIPPER_LENGTH
    angle2 = j2_rad
    angle3 = angle2 + j3_rad
    angle4 = angle3 + j4_rad
    distance = abs(L2 * math.sin(angle2)) + abs(L3 * math.sin(angle3)) + abs(L4 * math.sin(angle4)) + abs(L5 * math.sin(angle4))
    return distance

def check_angle_limits(angles, gripper_angle):
    global last_warning_time, stats
    current_time = time.time()
    violations = []
    for i, (angle, (min_limit, max_limit)) in enumerate(zip(angles, JOINT_LIMITS)):
        if angle < min_limit or angle > max_limit:
            joint_key = f"joint{i+1}"
            if joint_key not in last_warning_time or current_time - last_warning_time[joint_key] > WARNING_INTERVAL:
                violations.append(f"关节{i+1}: {angle:.1f}° (限制: {min_limit}°~{max_limit}°)")
                last_warning_time[joint_key] = current_time
                stats['limit_violations'] += 1
    min_grip, max_grip = GRIPPER_LIMITS
    if gripper_angle < min_grip or gripper_angle > max_grip:
        gripper_key = "gripper"
        if gripper_key not in last_warning_time or current_time - last_warning_time[gripper_key] > WARNING_INTERVAL:
            violations.append(f"夹爪: {gripper_angle:.1f}° (限制: {min_grip}°~{max_grip}°)")
            last_warning_time[gripper_key] = current_time
            stats['limit_violations'] += 1
    if violations:
        rospy.logwarn(f"[slider_control] ⚠️  角度超限:")
        for violation in violations:
            rospy.logwarn(f"[slider_control]    {violation}")
    return len(violations) == 0

def initialize_moveit_collision_checker():
    global robot_commander, move_group, planning_scene_interface, state_validity_service, MOVEIT_COLLISION_CHECK
    if not MOVEIT_AVAILABLE:
        rospy.logwarn("[碰撞检测] MoveIt不可用，使用基础碰撞检测")
        return False
    try:
        rospy.loginfo("[碰撞检测] 正在初始化MoveIt碰撞检测...")
        moveit_commander.roscpp_initialize([])
        robot_commander = moveit_commander.RobotCommander()
        move_group = moveit_commander.MoveGroupCommander(MOVEIT_GROUP_NAME)
        planning_scene_interface = moveit_commander.PlanningSceneInterface()
        try:
            rospy.wait_for_service('/check_state_validity', timeout=5.0)
            state_validity_service = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
        except rospy.ROSException:
            state_validity_service = None
        add_ground_collision_object()
        MOVEIT_COLLISION_CHECK = True
        return True
    except Exception as e:
        rospy.logwarn(f"[碰撞检测] MoveIt初始化失败: {e}")
        MOVEIT_COLLISION_CHECK = False
        return False

def add_ground_collision_object():
    global planning_scene_interface
    if planning_scene_interface is None:
        return
    try:
        from geometry_msgs.msg import PoseStamped
        ground_pose = PoseStamped()
        ground_pose.header.frame_id = "world"
        ground_pose.pose.position.z = -0.01
        ground_pose.pose.orientation.w = 1.0
        planning_scene_interface.add_box("ground_plane", ground_pose, size=(3.0, 3.0, 0.02))
    except Exception as e:
        rospy.logwarn(f"[碰撞检测] 添加地面碰撞对象失败: {e}")

def check_moveit_collision(angles):
    global move_group, state_validity_service
    if not MOVEIT_COLLISION_CHECK or move_group is None:
        return True, ""
    try:
        joint_positions = [math.radians(a) for a in angles]
        if state_validity_service is not None:
            robot_state = RobotState()
            robot_state.joint_state.name = ARM_JOINTS
            robot_state.joint_state.position = joint_positions
            request = GetStateValidityRequest()
            request.robot_state = robot_state
            request.group_name = MOVEIT_GROUP_NAME
            response = state_validity_service(request)
            if not response.valid:
                return False, "MoveIt检测到碰撞"
        return True, ""
    except Exception as e:
        return True, ""

def check_gripper_ground_collision(angles):
    j1, j2, j3, j4, j5, j6 = angles
    end_height = estimate_end_effector_height(j2, j3, j4)
    if end_height < MIN_END_HEIGHT:
        return False, f"夹爪过低: {end_height*100:.0f}cm"
    return True, ""

def check_collision(angles):
    global last_collision_warning_time
    if not COLLISION_CHECK_ENABLED:
        return True, ""
    j1, j2, j3, j4, j5, j6 = angles
    warnings = []
    if MOVEIT_COLLISION_CHECK:
        moveit_safe, moveit_info = check_moveit_collision(angles)
        if not moveit_safe:
            warnings.append(f"🚨 MoveIt检测: {moveit_info}")
    gripper_safe, gripper_info = check_gripper_ground_collision(angles)
    if not gripper_safe:
        warnings.append(f"🚨 {gripper_info}")
    end_height = estimate_end_effector_height(j2, j3, j4)
    if end_height < MIN_END_HEIGHT:
        warnings.append(f"🚨 末端过低: {end_height*100:.0f}cm")
    if j2 > 45 and j3 > 20:
        warnings.append(f"⚠️ 向前过度伸展: j2={j2:.0f}°, j3={j3:.0f}°")
    if j2 < -45 and j3 > 100:
        warnings.append(f"⚠️ 可能自碰撞: j2={j2:.0f}°, j3={j3:.0f}°")
    if j2 > 60 and j3 < -45:
        base_distance = estimate_end_effector_distance_to_base(j2, j3, j4)
        if base_distance < BASE_RADIUS + 0.05:
            warnings.append(f"🚨 夹爪可能撞底座")
    if abs(j2) > 80 and abs(j3) > 100:
        warnings.append(f"🚨 极端姿态: j2={j2:.0f}°, j3={j3:.0f}°")
    if j2 > 30 and j3 < -30 and j4 < -60:
        warnings.append(f"🚨 夹爪指向地面")
    if j2 < -60 and j3 < -80:
        warnings.append(f"🚨 向后下方折叠")
    if warnings:
        current_time = time.time()
        if current_time - last_collision_warning_time > COLLISION_WARNING_INTERVAL:
            last_collision_warning_time = current_time
            for w in warnings:
                rospy.logwarn(f"[碰撞检测] {w}")
        return False, warnings[0]
    return True, ""

def adjust_for_collision(angles):
    j1, j2, j3, j4, j5, j6 = angles[:]
    for i, (min_lim, max_lim) in enumerate(PRO450_JOINT_LIMITS):
        if angles[i] < min_lim:
            angles[i] = min_lim
        elif angles[i] > max_lim:
            angles[i] = max_lim
    j1, j2, j3, j4, j5, j6 = angles
    end_height = estimate_end_effector_height(j2, j3, j4)
    if end_height < MIN_END_HEIGHT:
        if j3 < 0:
            j3 = min(j3 + 30, 0)
        if estimate_end_effector_height(j2, j3, j4) < MIN_END_HEIGHT:
            j2 = max(j2 - 20, -60)
        if estimate_end_effector_height(j2, j3, j4) < MIN_END_HEIGHT and j4 < 0:
            j4 = min(j4 + 30, 60)
    if j2 > 45 and j3 > 20:
        j3 = min(j3, 20 - (j2 - 45) * 0.8)
    if j2 > 20 and j3 < -70:
        j3 = max(j3, -70)
    if j2 < -45 and j3 > 100:
        j3 = min(j3, 100)
    if j2 > 60 and j3 < -45:
        j3 = max(j3, -45)
    if abs(j4) > 100:
        j4 = max(-100, min(100, j4))
    if j2 > 30 and j3 < -30 and j4 < -60:
        j4 = max(j4, -60)
    if abs(j2) > 80 and abs(j3) > 100:
        j3 = max(-100, min(100, j3))
    if j2 < -60 and j3 < -80:
        j3 = max(j3, -80)
    return [j1, j2, j3, j4, j5, j6]

def clamp_angles(angles, gripper_angle):
    clamped_angles = []
    for i, (angle, (min_limit, max_limit)) in enumerate(zip(angles, JOINT_LIMITS)):
        clamped_angles.append(max(min_limit, min(max_limit, angle)))
    is_safe, warning = check_collision(clamped_angles)
    if not is_safe:
        clamped_angles = adjust_for_collision(clamped_angles)
    min_grip, max_grip = GRIPPER_LIMITS
    clamped_gripper = max(min_grip, min(max_grip, gripper_angle))
    return clamped_angles, clamped_gripper

def calculate_angle_difference(angles1, angles2):
    if angles1 is None or angles2 is None:
        return float('inf')
    return sum(abs(a - b) for a, b in zip(angles1, angles2))

def should_send_command(new_angles, new_gripper_angle):
    global last_angles, last_gripper_angle, last_command_time
    current_time = time.time()
    if current_time - last_command_time < 1.0 / MAX_COMMAND_RATE:
        return False, "频率限制"
    angle_diff = calculate_angle_difference(new_angles, last_angles)
    gripper_diff = abs(new_gripper_angle - last_gripper_angle) if last_gripper_angle is not None else float('inf')
    if angle_diff < ANGLE_THRESHOLD and gripper_diff < GRIPPER_THRESHOLD:
        return False, f"角度变化太小"
    return True, "允许发送"

class RobotCommand:
    def __init__(self, cmd_type, data, timestamp=None):
        self.type = cmd_type
        self.data = data
        self.timestamp = timestamp or time.time()

def is_pro450_connected():
    global mc
    try:
        if mc is None:
            return False
        mc.get_angles()
        return True
    except:
        return False

def add_command_to_queue(command):
    try:
        command_queue.put_nowait(command)
        return True
    except queue.Full:
        try:
            command_queue.get_nowait()
            command_queue.put_nowait(command)
            return True
        except queue.Empty:
            return False

def initialize_gripper():
    global mc
    try:
        time.sleep(2)
        version = mc.get_pro_gripper(1, GRIPPER_ID)
        if version == -1:
            return False
        return True
    except:
        return False

def map_gripper_angle_to_pro450(gazebo_angle):
    gazebo_angle = max(GAZEBO_MIN_POSITION, min(GAZEBO_MAX_POSITION, gazebo_angle))
    mapped_angle = ((gazebo_angle - GAZEBO_MIN_POSITION) / (GAZEBO_MAX_POSITION - GAZEBO_MIN_POSITION)) * (PRO450_GRIPPER_MAX - PRO450_GRIPPER_MIN) + PRO450_GRIPPER_MIN
    return int(round(max(PRO450_GRIPPER_MIN, min(PRO450_GRIPPER_MAX, mapped_angle))))

def set_gripper_angle_pro450(gazebo_angle):
    global mc
    mapped_angle = map_gripper_angle_to_pro450(gazebo_angle)
    try:
        mc.set_pro_gripper_angle(mapped_angle, GRIPPER_ID)
        return True
    except TypeError:
        if mapped_angle >= 90:
            mc.set_gripper_state(1, 80)
        elif mapped_angle <= 10:
            mc.set_gripper_state(0, 80)
        return True
    except:
        return False


def command_executor():
    global is_executing, last_angles, last_gripper_angle, last_command_time, stats, is_stopped
    while not rospy.is_shutdown():
        try:
            try:
                first_cmd = command_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            latest_angles_cmd = None
            latest_gripper_cmd = None
            if first_cmd.type == 'angles':
                latest_angles_cmd = first_cmd
            elif first_cmd.type == 'gripper':
                latest_gripper_cmd = first_cmd
            while True:
                try:
                    cmd = command_queue.get_nowait()
                    if cmd.type == 'angles':
                        latest_angles_cmd = cmd
                    elif cmd.type == 'gripper':
                        latest_gripper_cmd = cmd
                except queue.Empty:
                    break
            if not is_pro450_connected():
                stats['errors'] += 1
                continue
            is_executing = True
            try:
                if latest_angles_cmd is not None:
                    target_angles = latest_angles_cmd.data
                    j2 = target_angles[1] if len(target_angles) > 1 else 0
                    j3 = target_angles[2] if len(target_angles) > 2 else 0
                    j4 = target_angles[3] if len(target_angles) > 3 else 0
                    target_height_m = estimate_end_effector_height(j2, j3, j4)
                    target_height_mm = target_height_m * 1000
                    if target_height_mm < MIN_SAFE_HEIGHT_MM:
                        rospy.logwarn(f"[slider_control] 🚨 目标高度过低: {target_height_mm:.0f}mm < {MIN_SAFE_HEIGHT_MM}mm，拒绝执行")
                        stats['commands_skipped'] += 1
                        continue
                    else:
                        with stop_lock:
                            if is_stopped:
                                is_stopped = False
                                rospy.loginfo(f"[slider_control] ✅ 目标高度安全 ({target_height_mm:.0f}mm)，恢复执行")
                        rospy.loginfo(f"[slider_control] 发送角度到Pro450: {[round(a,1) for a in latest_angles_cmd.data]}")
                        mc.send_angles(latest_angles_cmd.data, 10)
                        last_angles = latest_angles_cmd.data.copy()
                        stats['commands_sent'] += 1
                if latest_gripper_cmd is not None:
                    gazebo_angle = latest_gripper_cmd.data
                    mapped_angle = map_gripper_angle_to_pro450(gazebo_angle)
                    if set_gripper_angle_pro450(gazebo_angle):
                        last_gripper_angle = gazebo_angle
                        stats['commands_sent'] += 1
                    else:
                        stats['errors'] += 1
                last_command_time = time.time()
            except Exception as e:
                rospy.logerr(f"[slider_control] 命令执行失败: {e}")
                stats['errors'] += 1
            finally:
                is_executing = False
        except Exception as e:
            rospy.logerr(f"[slider_control] 命令执行器错误: {e}")
            is_executing = False

def coords_callback(msg):
    global current_end_effector_coords, coords_lock
    with coords_lock:
        current_end_effector_coords = msg

def monitor_height():
    global current_end_effector_coords, coords_lock, mc, mode, pub_arm, is_stopped, stop_lock
    last_warning_time_local = 0
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        try:
            with coords_lock:
                if current_end_effector_coords is not None:
                    end_height = current_end_effector_coords.z
                    if end_height < MIN_SAFE_HEIGHT_MM:
                        current_time = time.time()
                        if current_time - last_warning_time_local > 1.0:
                            rospy.logwarn(f"[高度监控] 🚨 当前末端高度过低: {end_height:.0f}mm，请调整滑块到安全位置")
                            last_warning_time_local = current_time
                        with stop_lock:
                            if not is_stopped:
                                is_stopped = True
                                if mode == 2:
                                    try:
                                        mc.stop()
                                        rospy.logwarn(f"[高度监控] ✋ 已停止当前运动")
                                    except Exception as e:
                                        rospy.logerr(f"[高度监控] 停止机械臂失败: {e}")
                    else:
                        with stop_lock:
                            if is_stopped:
                                is_stopped = False
                                rospy.loginfo(f"[高度监控] ✅ 当前高度恢复安全 ({end_height:.0f}mm)")
        except Exception as e:
            rospy.logerr_throttle(5, f"[高度监控] 监控错误: {e}")
        rate.sleep()

def callback(msg):
    global stats
    stats['total_messages'] += 1
    arm_deg = [0.0] * len(ARM_JOINTS)
    grip_deg = 0.0
    name_to_deg = {name: math.degrees(pos) for name, pos in zip(msg.name, msg.position)}
    for i, joint_name in enumerate(ARM_JOINTS):
        if joint_name in name_to_deg:
            arm_deg[i] = round(name_to_deg[joint_name], 1)
    if GRIPPER_JOINT in name_to_deg:
        grip_deg = round(name_to_deg[GRIPPER_JOINT], 1)
    check_angle_limits(arm_deg, grip_deg)
    if mode == 1:
        should_send, reason = should_send_command(arm_deg, grip_deg)
        if should_send:
            clamped_arm, clamped_grip = clamp_angles(arm_deg, grip_deg)
            publish_to_gazebo(clamped_arm, clamped_grip)
        else:
            stats['commands_skipped'] += 1
    elif mode == 2:
        should_send, reason = should_send_command(arm_deg, grip_deg)
        if should_send:
            clamped_arm, clamped_grip = clamp_angles(arm_deg, grip_deg)
            arm_command = RobotCommand('angles', clamped_arm)
            if add_command_to_queue(arm_command):
                gripper_diff = abs(clamped_grip - last_gripper_angle) if last_gripper_angle is not None else float('inf')
                if gripper_diff >= GRIPPER_THRESHOLD:
                    add_command_to_queue(RobotCommand('gripper', clamped_grip))
            else:
                stats['commands_skipped'] += 1
        else:
            stats['commands_skipped'] += 1

def publish_to_gazebo(arm_deg, grip_deg):
    global pub_arm, pub_gripper, last_angles, last_gripper_angle, last_command_time, stats
    try:
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [math.radians(d) for d in arm_deg]
        pt.time_from_start = rospy.Duration(0.2)
        traj.points = [pt]
        pub_arm.publish(traj)
        traj_g = JointTrajectory()
        traj_g.header.stamp = rospy.Time.now()
        traj_g.joint_names = [GRIPPER_JOINT]
        ptg = JointTrajectoryPoint()
        ptg.positions = [math.radians(grip_deg)]
        ptg.time_from_start = rospy.Duration(0.2)
        traj_g.points = [ptg]
        pub_gripper.publish(traj_g)
        last_angles = arm_deg.copy()
        last_gripper_angle = grip_deg
        last_command_time = time.time()
        stats['commands_sent'] += 1
    except Exception as e:
        stats['errors'] += 1

def initialize_pro450():
    global mc
    try:
        rospy.loginfo(f"[slider_control] 正在连接 Pro450 @ {PRO450_IP}:{PRO450_PORT}...")
        mc = Pro450Client(PRO450_IP, PRO450_PORT)
        time.sleep(1.0)
        mc.power_on()
        time.sleep(1.0)
        mc.set_servo_calibration(6)
        time.sleep(0.5)
        current_angles = mc.get_angles()
        rospy.loginfo(f"[slider_control] ✅ Pro450连接成功! 当前角度: {current_angles}")
        gripper_ok = initialize_gripper()
        if gripper_ok:
            set_gripper_angle_pro450(0.0)
        mc.release_all_servos()
        time.sleep(0.5)
        return True
    except Exception as e:
        rospy.logerr(f"[slider_control] ❌ Pro450初始化失败: {e}")
        return False

def print_stats():
    if stats['total_messages'] > 0:
        efficiency = (stats['commands_sent'] / stats['total_messages']) * 100
        rospy.loginfo(f"[slider_control] 📊 统计: 消息:{stats['total_messages']}, 发送:{stats['commands_sent']}, 跳过:{stats['commands_skipped']}, 错误:{stats['errors']}, 效率:{efficiency:.1f}%")

def main():
    global mc, mode, pub_arm, pub_gripper, MOVEIT_COLLISION_CHECK
    rospy.init_node("slider_control_450", anonymous=True)
    print("\n" + "="*60)
    print(" MyCobot Pro 450 滑块控制器")
    print("="*60)
    print("选择控制模式:")
    print("  1: 滑块 → Gazebo仿真")
    print("  2: 滑块 → 真实 MyCobot Pro 450")
    print("="*60)
    inp = input("请输入 1 或 2 (默认 2): ").strip()
    mode = 1 if inp == "1" else 2
    mode_name = "Gazebo仿真" if mode == 1 else "真实 Pro 450"
    rospy.loginfo(f"[slider_control] 控制模式: {mode_name}")
    if COLLISION_CHECK_ENABLED:
        initialize_moveit_collision_checker()
    if mode == 1:
        pub_arm = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)
        pub_gripper = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=1)
    if mode == 2:
        if not initialize_pro450():
            rospy.logerr("[slider_control] ❌ Pro450初始化失败，退出")
            return
        executor_thread = threading.Thread(target=command_executor, daemon=True)
        executor_thread.start()
        monitor_thread = threading.Thread(target=monitor_height, daemon=True)
        monitor_thread.start()
    rospy.Subscriber("/joint_states", JointState, callback, queue_size=1)
    from geometry_msgs.msg import Point
    rospy.Subscriber("/pro450/end_effector_coords", Point, coords_callback, queue_size=1)
    rospy.loginfo(f"[slider_control] {mode_name}控制器启动成功，等待滑块输入...")
    def stats_timer():
        while not rospy.is_shutdown():
            time.sleep(15.0)
            print_stats()
    stats_thread = threading.Thread(target=stats_timer, daemon=True)
    stats_thread.start()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        print_stats()
        if mc is not None:
            try:
                mc.release_all_servos()
            except:
                pass

if __name__ == "__main__":
    main()
