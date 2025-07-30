#!/usr/bin/env python3
import rospy
import serial
import struct
from geometry_msgs.msg import Twist
import time
import os
from serial.tools import list_ports
import binascii
from sys import exit

BAUD_RATE = 115200

# 配置目标设备ID
TARGET_VID = 0x1a86  # 十六进制格式
TARGET_PID = 0x7523

def init_serial():
    """智能检测并打开目标设备串口"""
    # 方法1：使用serial工具检测
    ports = list_ports.comports()
    for port in ports:
        if port.vid == TARGET_VID and port.pid == TARGET_PID:
            try:
                ser = serial.Serial(port.device, BAUD_RATE, timeout=0.1)  # 添加超时设置
                rospy.loginfo(f"成功打开目标设备: {port.device}")
                return ser
            except serial.SerialException as e:
                rospy.logerr(f"无法打开 {port.device}: {str(e)}")
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
                        ser = serial.Serial(dev_path, BAUD_RATE, timeout=0.1)
                        rospy.loginfo(f"成功打开目标设备: {dev_path}")
                        return ser
                    except serial.SerialException as e:
                        rospy.logerr(f"无法打开 {dev_path}: {str(e)}")
            except Exception as e:
                rospy.logdebug(f"访问 {dev_path} 失败: {str(e)}")
                continue

    rospy.logerr("未找到目标设备！请检查：\n"
                 "1. 设备已连接\n"
                 "2. 驱动已安装（CH340需要专用驱动）\n"
                 "3. 设备权限正确（尝试：sudo chmod 666 /dev/ttyUSB*）")
    exit(1)

def get_arm_input():
    """获取单行输入的机械臂参数，支持q键退出"""
    while True:
        user_input = input("请输入机械臂参数 (格式: x y angle) 或按q退出: ")
        
        # 检查退出命令
        if user_input.lower() == 'q':
            return None, None, None
            
        values = user_input.split()
        
        # 验证输入
        if len(values) != 3:
            print("错误：需要输入三个参数，用空格分隔")
            continue
            
        try:
            x = float(values[0])
            y = float(values[1])
            angle = float(values[2])
            return x, y, angle
        except ValueError:
            print("输入错误：所有参数必须是数字")

def main():
    ser = init_serial()
    print("===== 机械臂控制程序 =====")
    print("请在一行内输入x、y和角度值（用空格分隔）")
    print("输入q并enter后退出程序\n")
    
    while True:
        # 获取用户输入
        result = get_arm_input()
        
        # 处理退出
        if result == (None, None, None):
            break
            
        x, y, angle = result
        # 创建格式化字符串
        arm_status = f"机械臂:{x},{y},{angle}"
        ser.write(arm_status.encode('gbk'))
        
        # 显示结果
        print("\n===== 机械臂状态更新 =====")
        print(arm_status)
        print("==========================\n")

    print("程序已退出！")

if __name__ == "__main__":
    main()
