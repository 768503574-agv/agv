import rospy
import json
import socket
import struct
from geometry_msgs.msg import Twist  # 关键：确保导入Twist消息
import keyboard
import time
from rbkNetProtoEnums import *  # 导入官方枚举（包含API编号）
import rbkNetProtoEnums  # 官方协议工具

# -------------------------- 1. 基础配置（对齐官方demo） --------------------------
AGV_IP = "192.168.1.121"
CONTROL_PORT = 19205
LINEAR_SPEED = 0.2
ANGULAR_SPEED = 0.8
vx, vy, w = 0.0, 0.0, 0.0
so = None

# -------------------------- 2. 协议处理（直接使用官方工具） --------------------------
def send_control_command(api_type, params, retry=1):
    """使用官方packMsg打包，遵循官方协议格式"""
    global so
    if not so:
        rospy.logerr("❌ TCP连接未建立")
        return False

    # 生成官方格式的请求报文
    request_msg = rbkNetProtoEnums.packMsg(1, api_type, params)  # 参考demo的packMsg调用
    if not request_msg:
        rospy.logerr("❌ 报文打包失败")
        return False

    for i in range(retry + 1):
        try:
            so.send(request_msg)
            rospy.logdebug(f"📤 发送指令：API={api_type}，数据区={params}")

            # 接收响应（先收16字节头部）
            so.settimeout(5.0)
            header = so.recv(16)
            if len(header) != 16:
                rospy.logwarn(f"⚠️ 响应头部不完整（{len(header)}B）")
                continue

            # 使用官方方式解析头部（获取数据长度和响应序号）
            json_data_len, back_req_num = rbkNetProtoEnums.unpackHead(header)
            rospy.logdebug(f"📥 响应头部：数据区长度={json_data_len}，序号={back_req_num}")

            # 接收数据区（限制最大长度）
            if json_data_len > 10 * 1024 * 1024:
                rospy.logwarn(f"⚠️ 响应数据区过大（{json_data_len}B>10MB）")
                continue

            resp_data = so.recv(json_data_len).decode('utf-8') if json_data_len > 0 else "{}"
            resp_json = json.loads(resp_data)

            # 检查返回状态（官方通常用ret_code标识成功）
            if resp_json.get("ret_code", 0) == 0:
                rospy.loginfo(f"✅ 指令执行成功（API={api_type}，速度：vx={vx}，w={w}）")
                return True
            else:
                rospy.logwarn(f"⚠️ 指令执行失败：{resp_json.get('err_msg', '未知错误')}")

        except socket.timeout:
            rospy.logwarn(f"⚠️ 第{i+1}次尝试超时")
        except socket.error as e:
            rospy.logerr(f"❌ 第{i+1}次发送失败：{str(e)}")
            # 尝试重连
            if i < retry:
                so.close()
                so = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                so.connect((AGV_IP, CONTROL_PORT))
                rospy.loginfo("🔌 重新连接AGV成功")
        except json.JSONDecodeError:
            rospy.logwarn(f"⚠️ 响应数据非JSON：{resp_data}")
        except Exception as e:
            rospy.logerr(f"❌ 处理响应失败：{str(e)}")

        if i < retry:
            time.sleep(0.5)

    rospy.logerr(f"❌ 指令发送失败（已重试{retry}次）")
    return False

# -------------------------- 3. 其他逻辑保持不变（仅修改协议相关部分） --------------------------
def update_speed_by_keyboard():
    global vx, vy, w
    if keyboard.is_pressed('space'):
        vx, vy, w = 0.0, 0.0, 0.0
        rospy.logwarn("⚠️ 紧急停止")
        return
    vx = LINEAR_SPEED if keyboard.is_pressed('w') else (-LINEAR_SPEED if keyboard.is_pressed('s') else 0.0)
    w = ANGULAR_SPEED if keyboard.is_pressed('a') else (-ANGULAR_SPEED if keyboard.is_pressed('d') else 0.0)

def agv_keyboard_control():
    global so
    rospy.init_node('agv_control_node', anonymous=True)
    cmd_pub = rospy.Publisher('/agv/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(2)

    try:
        rospy.loginfo(f"🔌 连接AGV：{AGV_IP}:{CONTROL_PORT}")
        so = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        so.connect((AGV_IP, CONTROL_PORT))
        rospy.loginfo("✅ 连接成功！")

        while not rospy.is_shutdown():
            update_speed_by_keyboard()
            # 发布ROS消息
            ros_cmd = Twist()
            ros_cmd.linear.x = vx
            ros_cmd.angular.z = w
            cmd_pub.publish(ros_cmd)
            # 发送运动指令（使用官方API编号）
            send_control_command(robot_control_motion_req, {"vx": vx, "vy": vy, "w": w}, retry=1)
            rate.sleep()

    except socket.error as e:
        rospy.logerr(f"❌ 连接错误：{e}，请检查IP/端口")
    except Exception as e:
        rospy.logerr(f"❌ 程序异常：{e}")
    finally:
        rospy.loginfo("📌 发送停止指令...")
        if so:
            send_control_command(robot_control_stop_req, {}, retry=0)  # 官方停止指令
            so.close()
        rospy.loginfo("📌 程序退出")

if __name__ == "__main__":
    try:
        agv_keyboard_control()
    except rospy.ROSInterruptException:
        pass
