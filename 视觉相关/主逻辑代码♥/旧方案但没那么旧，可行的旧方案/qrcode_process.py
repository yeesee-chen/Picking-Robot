#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

try:
    from pyzbar.pyzbar import decode

    PYZBAR_AVAILABLE = True
except ImportError:
    PYZBAR_AVAILABLE = False
    print("警告: 未安装pyzbar库，无法解码二维码。请使用 pip install pyzbar 安装")


class QRCodeScanner:
    def __init__(self):
        # 创建CV Bridge用于ROS图像消息和OpenCV图像之间的转换
        self.bridge = CvBridge()

        # 创建发布者
        self.qr_pub = rospy.Publisher("/qr_message", String, queue_size=10)

        # 创建订阅者 - 订阅图像话题
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.last_qr_data = None  # 避免重复发送相同的二维码

        rospy.loginfo("QR码扫描器已启动，等待图像...")
        rospy.loginfo("订阅话题: /camera/color/image_raw")
        rospy.loginfo("发布话题: /qr_message")

    def image_callback(self, msg):
        """
        图像话题回调函数
        """
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 检测二维码
            qr_data = self.detect_qr_code(cv_image)

            # 如果检测到二维码且与上次结果不同，则发布
            if qr_data and qr_data != self.last_qr_data:
                self.qr_pub.publish(String(qr_data))
                rospy.loginfo(f"检测到二维码: {qr_data}")
                rospy.loginfo(f"已发布到话题: /qr_message")
                self.last_qr_data = qr_data

            # 可选：显示图像用于调试
            if qr_data:
                cv2.putText(cv_image, f"QR: {qr_data}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # 显示图像窗口（可选，用于调试）
            cv2.imshow("QR Scanner", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"图像处理错误: {e}")

    def detect_qr_code(self, frame):
        """
        检测并解码图像中的二维码
        返回解码后的字符串，如果没有检测到则返回None
        """
        if not PYZBAR_AVAILABLE:
            return None

        # 转换为灰度图（可选，pyzbar也能处理彩色图像）
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 解码二维码
        decoded_objects = decode(gray)

        if decoded_objects:
            # 取第一个二维码的数据
            qr_data = decoded_objects[0].data.decode('utf-8')
            return qr_data

        return None


def main():
    # 初始化ROS节点
    rospy.init_node("qr_scanner")

    # 创建QR扫描器实例
    scanner = QRCodeScanner()

    # 保持节点运行
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("正在关闭...")
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()