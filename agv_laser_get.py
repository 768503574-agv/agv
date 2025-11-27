#!/usr/bin/env python3
"""
基于仙知RoboKit NetProtocol v1.2.1的AGV激光数据获取节点
功能：1. 连接AGV状态端口（19204） 2. 发送激光请求（API 1009） 3. 解析数据并发布ROS LaserScan消息
依赖：rbkNetProtoEnums（系统路径已配置）、ROS Noetic
"""
from rbkNetProtoEnums import packMsg, unpackHead, robot_status_laser_req
import json
import socket
import rospy
from sensor_msgs.msg import LaserScan
import math
from typing import Optional, List
import time

# -------------------------- 配置参数（根据AGV实际情况修改） --------------------------
AGV_IP = "192.168.1.121"       # AGV的IP地址
API_PORT_STATE = 19204         # 状态API端口（文档1-4-38）
TIMEOUT = 8.0                  # TCP超时时间（秒）
LASER_STEP = 1                 # 激光采样步长（建议非零值，默认0表示不采样）
LASER_FOV = 2 * math.pi        # 激光视场角（360°，单位rad）
LASER_RANGE_MAX = 5.0          # 激光最大探测距离（米）
LASER_RANGE_MIN = 0.1          # 激光最小探测距离（米）
ROS_PUB_FREQ = 10              # ROS消息发布频率（Hz）
LASER_FRAME_ID = "laser"       # 激光雷达坐标系（需与TF配置一致）

# -------------------------- 协议常量（来自文档1） --------------------------
MAX_DATA_LEN = 10 * 1024 * 1024  # 最大数据区长度（10M，文档1-8-90）
ROBOT_STATUS_LASER_RESP = 11009  # 激光响应API编号（1009+10000，文档1-4-107）

def validate_laser_params() -> bool:
    """校验激光参数合法性（符合文档1要求）"""
    if not (0 <= LASER_STEP <= 100):
        rospy.logerr(f"❌ 激光步长非法（{LASER_STEP}），允许范围0-100")
        return False
    if not (0.01 <= LASER_RANGE_MIN < LASER_RANGE_MAX <= 20.0):
        rospy.logerr(f"❌ 激光距离范围非法（min={LASER_RANGE_MIN}, max={LASER_RANGE_MAX}）")
        return False
    if not (1 <= ROS_PUB_FREQ <= 50):
        rospy.logerr(f"❌ 发布频率非法（{ROS_PUB_FREQ}Hz），允许范围1-50Hz")
        return False
    return True

def validate_configuration() -> bool:
    """校验配置参数的合法性"""
    if not AGV_IP or not AGV_IP.startswith("192.168"):
        rospy.logerr(f"❌ 无效的AGV IP地址：{AGV_IP}")
        return False
    if not (1 <= API_PORT_STATE <= 65535):
        rospy.logerr(f"❌ 无效的端口号：{API_PORT_STATE}")
        return False
    return True

def parse_alarm_codes(alarm_json: dict) -> None:
    """解析告警码（文档1-9-90）"""
    fatals = alarm_json.get("fatals", [])
    errors = alarm_json.get("errors", [])
    warnings = alarm_json.get("warnings", [])
    
    if fatals:
        rospy.logerr(f"🔴 Fatal告警：{fatals}（需紧急处理）")
    if errors:
        rospy.logerr(f"🟠 Error告警：{errors}（功能异常）")
    if warnings:
        rospy.logwarn(f"🟡 Warning告警：{warnings}（建议检查）")

def receive_full_data(sock: socket.socket, expected_len: int) -> Optional[bytes]:
    """确保接收完整的数据区（处理TCP分包）"""
    received = b""
    remaining = expected_len
    while remaining > 0:
        # 每次接收8KB缓冲区（提高效率）
        chunk = sock.recv(min(remaining, 8192))
        if not chunk:
            rospy.logerr("❌ TCP连接中断（AGV主动断开或网络异常）")
            return None
        received += chunk
        remaining -= len(chunk)
    # 校验接收长度
    if len(received) != expected_len:
        rospy.logerr(f"❌ 数据区长度不匹配（接收{len(received)}B/预期{expected_len}B）")
        return None
    return received

def connect_to_agv() -> Optional[socket.socket]:
    """尝试连接AGV，最多重试3次"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(TIMEOUT)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    
    for attempt in range(3):
        try:
            sock.connect((AGV_IP, API_PORT_STATE))
            rospy.loginfo(f"✅ 连接成功：{AGV_IP}:{API_PORT_STATE}")
            return sock
        except socket.error as e:
            rospy.logerr(f"❌ 第{attempt + 1}次连接失败：{e}")
            if attempt < 2:
                rospy.loginfo(f"⚠️ 等待3秒后重试...")
                time.sleep(3)
            else:
                rospy.logerr("❌ 尝试连接失败超过3次，退出程序。")
                return None
    return None

def get_laser_from_agv() -> Optional[List[List[float]]]:
    """
    核心函数：连接AGV并获取激光数据（完全依赖rbkNetProtoEnums解析头部）
    返回：激光点数组（[[x1,y1], [x2,y2], ...]）或None（失败）
    """
    try:
        # 1. 校验配置参数和激光参数
        if not validate_configuration() or not validate_laser_params():
            return None
        
        # 2. 创建TCP连接
        sock = connect_to_agv()
        if sock is None:
            return None
        
        # 3. 构造激光请求参数
        req_params = {"step": LASER_STEP}
        req_api = robot_status_laser_req
        rospy.loginfo(f"📤 发送激光请求：API={req_api}（1009），步长={LASER_STEP}")
        
        # 4. 打包请求报文
        request = packMsg(1, req_api, req_params)
        if not request:
            rospy.logerr("❌ 请求报文打包失败（检查rbkNetProtoEnums库是否完整）")
            return None
        sock.send(request)
        
        # 5. 接收响应头部
        header = sock.recv(16)
        rospy.loginfo(f"接收到的头部数据：{header.hex()}")  # 打印头部数据（以十六进制形式显示）

        if len(header) != 16:
            rospy.logerr(f"❌ 响应头部不完整（实际{len(header)}B/需16B）")
            return None
        
        # 6. 用库函数解析头部
        try:
            json_data_len, resp_type = unpackHead(header)  # 修改为两个返回值
            rospy.loginfo(f"📥 头部解析：数据区长度={json_data_len}B，响应类型={resp_type}")
        except ValueError as e:
            rospy.logerr(f"❌ 解包错误：{e}")
            return None
        
        # 7. 校验数据区长度
        if json_data_len > MAX_DATA_LEN:
            rospy.logerr(f"❌ 数据区过大（{json_data_len}B），超过10M限制")
            return None
        if json_data_len < 0:
            rospy.logerr(f"❌ 数据区长度为负（{json_data_len}B），头部解析异常")
            return None
        
        # 8. 校验响应类型（必须是激光响应）
        if resp_type != ROBOT_STATUS_LASER_RESP:
            rospy.logerr(f"❌ 响应类型不匹配（实际{resp_type}/预期{ROBOT_STATUS_LASER_RESP}）")
            return None
        
        # 9. 接收并解析数据区
        resp_data = receive_full_data(sock, json_data_len)
        if not resp_data:
            return None
        
        # 解析JSON数据
        try:
            laser_json = json.loads(resp_data.decode("utf-8"))
            rospy.loginfo(f"返回的激光数据：{laser_json}")  # 打印返回的激光数据
        except json.JSONDecodeError as e:
            rospy.logerr(f"❌ JSON解析失败：{e}")
            rospy.logerr(f"错误数据片段（前200字节）：{resp_data[:200].hex()}")
            return None
        
        # 10. 提取并校验激光点数据
        laser_beams = laser_json.get("lasers", [])
        if not isinstance(laser_beams, list) or len(laser_beams) == 0:
            rospy.logwarn("⚠️ AGV返回激光点为空（检查激光雷达是否被遮挡/未启动）")
            return None
        
        valid_beams = []
        for idx, laser in enumerate(laser_beams):
            for beam in laser.get('beams', []):
                # 检查'angle'和'dist'字段，并且验证有效性
                if 'angle' in beam and 'dist' in beam and beam.get('valid', False):
                    # 进一步过滤掉距离值过大或过小的点（如超过最大范围或小于最小范围）
                    if LASER_RANGE_MIN <= beam['dist'] <= LASER_RANGE_MAX:
                        valid_beams.append([beam['angle'], beam['dist']])
                    else:
                        rospy.logwarn(f"⚠️ 激光点数据超出范围（angle={beam['angle']}，dist={beam['dist']}）")
                else:
                    rospy.logwarn(f"⚠️ 无效的激光点（缺少'angle'或'dist'字段）：{beam}")
        
        if not valid_beams:
            rospy.logwarn("⚠️ AGV返回的激光点均无效或为空")
            return None
        
        rospy.loginfo(f"🎉 成功获取有效激光点：{len(valid_beams)}")
        return valid_beams

    except Exception as e:
        rospy.logerr(f"❌ 获取激光数据时发生错误：{e}")
        return None
    finally:
        sock.close()
        rospy.loginfo("🔌 TCP连接已关闭")

def publish_laser_data(laser_data: List[List[float]]):
    """将激光数据转换为LaserScan消息并发布"""
    laser_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
    scan = LaserScan()

    # 设置消息头部（时间戳 + 坐标系）
    scan.header.stamp = rospy.Time.now()
    scan.header.frame_id = LASER_FRAME_ID

    # 计算激光数据的角度增量
    point_count = len(laser_data)
    scan.angle_min = -LASER_FOV / 2
    scan.angle_max = LASER_FOV / 2
    scan.angle_increment = LASER_FOV / point_count if point_count > 0 else 0.0
    scan.time_increment = 0.0
    scan.scan_time = 1.0 / ROS_PUB_FREQ

    # 激光范围和强度数据
    scan.range_min = LASER_RANGE_MIN
    scan.range_max = LASER_RANGE_MAX
    scan.ranges = [math.sqrt(x**2 + y**2) for x, y in laser_data]
    scan.intensities = [1.0] * len(scan.ranges)  # 默认强度为1.0

    # 发布数据
    laser_pub.publish(scan)
    rospy.loginfo(f"🎉 已发布激光数据：{len(scan.ranges)}个点")

if __name__ == '__main__':
    rospy.init_node('agv_laser_node', anonymous=True)  # 初始化ROS节点
    rospy.loginfo("节点已启动，开始获取激光数据...")

    # 获取激光数据
    laser_data = get_laser_from_agv()
    
    if laser_data:
        # 发布激光数据
        publish_laser_data(laser_data)
    else:
        rospy.logerr("❌ 激光数据获取失败")

