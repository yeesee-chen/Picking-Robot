#!/usr/bin/env python3
import rospy
import serial
import struct
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, String
from sys import exit
import time
import os
from serial.tools import list_ports  # 新增导入

# 配置目标设备ID
TARGET_VID = 0x1a86  # 十六进制格式
TARGET_PID = 0x7523

# 协议配置保持不变
BAUD_RATE = 115200
LINEAR_FRONT_HEADER = bytes.fromhex('FF 01 01')
LINEAR_BACK_HEADER = bytes.fromhex('FF 01 00')
ANGULAR_HEADER_RIGHT = bytes.fromhex('02 01')
ANGULAR_HEADER_LEFT = bytes.fromhex('02 00')
LAST = bytes.fromhex('FE')

def init_serial():
    """智能检测并打开目标设备串口"""
    # 方法1：使用serial工具检测
    ports = list_ports.comports()
    for port in ports:
        if port.vid == TARGET_VID and port.pid == TARGET_PID:
            try:
                ser = serial.Serial(port.device, BAUD_RATE)
                print(f"成功打开目标设备: {port.device}")
                return ser
            except serial.SerialException as e:
                print(f"无法打开 {port.device}: {str(e)}")
                continue

    # 方法2：通过sys文件系统检测（备用）
    for dev in os.listdir('/dev'):
        if dev.startswith('ttyUSB') or dev.startswith('ttyACM'):
            dev_path = os.path.join('/dev', dev)
            try:
                sys_path = os.path.realpath(f'/sys/class/tty/{dev}/device/../..')
                with open(f"{sys_path}/idVendor") as f:
                    vid = int(f.read().strip(), 16)
                with open(f"{sys_path}/idProduct") as f:
                    pid = int(f.read().strip(), 16)
                if vid == TARGET_VID and pid == TARGET_PID:
                    try:
                        ser = serial.Serial(dev_path, BAUD_RATE)
                        print(f"成功打开目标设备: {dev_path}")
                        return ser
                    except serial.SerialException as e:
                        print(f"无法打开 {dev_path}: {str(e)}")
            except Exception:
                continue
   
    print("未找到目标设备！请检查：\n"
                 "1. 设备已连接\n"
                 "2. 驱动已安装（CH340需要专用驱动）\n"
                 "3. 设备权限正确（尝试：sudo chmod 666 /dev/ttyUSB*）")
    exit(1)

ser = init_serial()  # 初始化串口连接

linear = 0;
angular = 0;
weather_arrive = 0;
x = 0;
y = 0;
angle = 0;
arm_data = None
arm_count = 0
whether_send_arm = 0

def Encode_arm_data(data):
    return data.encode('gbk')

def Encode_speed_data(data1, data2):
    chinese_str = f"速度:{data1};角速度:{data2};"
    encoded_data = chinese_str.encode('gbk')
    return encoded_data

def cmd_vel_callback(msg):
    global linear,angular
    linear = int(msg.linear.x * 8)
    angular = int(msg.angular.z * -5)
    # linear = 0
    # angular = 0

def arrive_state_callback(msg):
    global weather_arrive
    weather_arrive = msg.data

def arm_callback(msg):
    global arm_data,whether_send_arm
    whether_send_arm = 1
    arm_data = msg.data

if __name__ == "__main__":
    rospy.init_node("serial_transmitter", log_level=rospy.INFO)
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
    rospy.Subscriber("/weather_arrive", Int8, arrive_state_callback)
    rospy.Subscriber("/robot_arm_point", String, arm_callback)
    Rate = rospy.Rate(40)
    while not rospy.is_shutdown():
        
        # if weather_arrive != None:
            # if arm_data == f"语音:17;" AND count == 0:
            #     data = Encode_arm_data(arm_data)
            #     ser.write(data)
            #     count = 1
            # if weather_arrive == 0:
        data = Encode_speed_data(linear,angular)
        ser.write(data)
        if whether_send_arm == 1:
            data = Encode_arm_data(arm_data)
            ser.write(data)
            arm_count += 1
            whether_send_arm = 0

        back = ser.read_all()
        try:
            rospy.loginfo(f"接收: {back.decode('utf-8')}，arm:{arm_count}")
        except UnicodeDecodeError:
            try:
                rospy.loginfo(f"接收(GBK): {back.decode('gbk', errors='replace')}，arm:{arm_count}")
            except:
                rospy.loginfo(f"接收(原始): {binascii.hexlify(back)}")
        Rate.sleep()
        # packet_angular = pack_data(angular, 2)
        # ser.write(packet_angular)
        
        # packet_linear = pack_data(linear, 1)
        # ser.write(packet_linear)
        
    # while True:

    #     linear = int(1 * 3)
    #     angular = int(0 * 5)

    #     packet_angular = pack_data(angular, 2)
    #     ser.write(packet_angular)
    #     # ser.write(bytes.fromhex('FF'))  # 保持额外FF发送
        
    #     packet_linear = pack_data(linear, 1)
    #     ser.write(packet_linear)
        
    #     back = ser.read()
    #     print(back)
    #     time.sleep(0.1)
    # rospy.spin()
    