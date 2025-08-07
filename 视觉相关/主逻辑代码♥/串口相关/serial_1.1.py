#!/usr/bin/env python3
import serial
import struct
import time
import os
from serial.tools import list_ports
import threading

BAUD_RATE = 115200
TARGET_VID = 0x1a86
TARGET_PID = 0x7523

# 全局变量用于控制线程
running = True
current_arm_cmd = None
lock = threading.Lock()


def init_serial():
    """初始化并打开目标串口设备"""
    # 方法1：使用pyserial的自动检测
    for port in list_ports.comports():
        if port.vid == TARGET_VID and port.pid == TARGET_PID:
            try:
                ser = serial.Serial(port.device, BAUD_RATE, timeout=0.1)
                print(f"成功打开目标设备: {port.device}")
                return ser
            except serial.SerialException as e:
                print(f"无法打开 {port.device}: {str(e)}")
                continue

    # 方法2：手动检查/dev目录
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
                        print(f"成功打开目标设备: {dev_path}")
                        return ser
                    except serial.SerialException as e:
                        print(f"无法打开 {dev_path}: {str(e)}")
            except Exception as e:
                print(f"访问 {dev_path} 失败: {str(e)}")
                continue

    print("未找到目标设备！请检查：\n"
          "1. 设备已连接\n"
          "2. 驱动已安装（CH340需要专用驱动）\n"
          "3. 设备权限正确（尝试：sudo chmod 666 /dev/ttyUSB*）")
    exit(1)


def get_arm_input():
    """获取单行输入的机械臂参数，支持q键退出"""
    while True:
        user_input = input("请输入机械臂参数 (格式: x y angle) 或按q退出: ")

        if user_input.lower() == 'q':
            return None

        values = user_input.split()

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


def velocity_sender(ser):
    """持续发送速度指令的线程函数"""
    global running, current_arm_cmd, lock

    while running:
        # 准备要发送的指令
        vel_cmd = "速度:0;角速度:0;"

        # 获取当前的机械臂指令（线程安全）
        with lock:
            arm_cmd = current_arm_cmd

        # 组合指令
        full_cmd = vel_cmd
        if arm_cmd:
            full_cmd += arm_cmd

        # 发送指令
        try:
            ser.write(full_cmd.encode('gbk'))
            # 读取回传数据
            back = ser.read_all()
            if back:
                try:
                    print(f"设备响应: {back.decode('gbk')}")
                except:
                    print(f"设备响应: {back}")

        except serial.SerialException as e:
            print(f"串口发送错误: {str(e)}")
            break

        # 控制发送频率（10Hz）
        time.sleep(0.1)

    print("速度发送线程已停止")


def main():
    global running, current_arm_cmd

    ser = init_serial()
    print("===== 机械臂控制程序 =====")
    print("请在一行内输入x、y和角度值（用空格分隔）")
    print("输入q并enter后退出程序\n")

    # 启动速度发送线程
    velocity_thread = threading.Thread(target=velocity_sender, args=(ser,))
    velocity_thread.daemon = True
    velocity_thread.start()

    # 设置初始机械臂状态
    time.sleep(2)
    current_arm_cmd = f"观测位:3;"
    time.sleep(2)
    # current_arm_cmd = f"爪子:1;"

    try:
        while running:
            # 获取用户输入
            result = get_arm_input()

            # 处理退出
            if result is None:
                running = False
                break

            x, y, angle = result

            # 创建格式化字符串（线程安全更新）
            arm_status = f"机械臂:{x},{y},{angle};"
            with lock:
                current_arm_cmd = arm_status

            # 显示结果
            print("\n===== 机械臂状态更新 =====")
            print(arm_status)
            print("==========================\n")
    except KeyboardInterrupt:
        running = False
        print("\n检测到Ctrl+C，正在停止程序...")

    # 等待发送线程结束
    velocity_thread.join(timeout=1.0)
    print("程序已退出！")


if __name__ == "__main__":
    main()