#!/usr/bin/env python3
"""
YOLOv5水果检测ROS节点 - 点位输出版 - 修改版
"""

import rospy
import cv2
import torch
import numpy as np
import traceback
import os

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String, Int32
from custom_msgs.msg import FruitStatusArray, MatureFruit, MatureFruitArray, MaturePoint, MaturePoints


class YoloNode:
    def __init__(self):
        rospy.init_node('yolo_fruit_detection_node', anonymous=True)
        rospy.loginfo("正在初始化YOLOv5水果检测节点...")

        # 初始化参数
        self._init_parameters()

        # 初始化组件
        self.bridge = CvBridge()
        self._init_model()
        self._init_publishers()
        self._init_subscribers()

        # 当前区域状态 - 默认设置为1
        self.current_region = 1
        rospy.loginfo("YOLOv5水果检测节点初始化完成")

    def _init_parameters(self):
        """初始化ROS参数"""
        # 模型相关参数
        self.yolov5_path = rospy.get_param('~yolov5_path', '/home/nvidia/chu_ws/src/yolo_realsense_ros/yolov5')
        self.weight_path = rospy.get_param('~weight_path', '/home/nvidia/chu_ws/src/yolo_realsense_ros/weights/best.pt')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)

        # 成熟度判断参数
        self.saturation_threshold = rospy.get_param('~saturation_threshold', 40)
        self.value_threshold = rospy.get_param('~value_threshold', 40)

        # 图像尺寸参数
        self.expected_width = rospy.get_param('~image_width', 640)
        self.expected_height = rospy.get_param('~image_height', 480)

        # 水果类别和成熟条件映射
        self.fruit_maturity_mapping = {
            0: {'name': 'jiao', 'mature_hue_ranges': [(0, 10), (160, 180)]},
            1: {'name': 'xihongshi', 'mature_hue_ranges': [(0, 10), (160, 180)]},
            2: {'name': 'li', 'mature_hue_ranges': [(11, 35)]},
            3: {'name': 'nangua', 'mature_hue_ranges': [(10, 30)]},
            4: {'name': 'pingguo', 'mature_hue_ranges': [(0, 10), (160, 180)]},
            5: {'name': 'yangcong', 'mature_hue_ranges': [(125, 180)]}
        }

        # 点位映射配置 - 12个矩形区域
        # 图像尺寸: 640x480
        # 分为2行4列，共8个区域
        self.point_rects = [
            # (x1, y1, x2, y2)
            # 点位方案需要测点位的大概位置，就是12目标树上的需要位置
            (545, 190, 555, 230),  # 区域1
            (465, 215, 495, 240),  # 区域2
            (395, 240, 430, 260),  # 区域3
            (250, 280, 280, 310),  # 区域4
            (195, 280, 225, 310),  # 区域5
            (130, 265, 155, 295),  # 区域6
            (120, 430, 140, 450),  # 区域7
            (180, 415, 200, 445),  # 区域8
            (220, 415, 240, 455),  # 区域9
            (405, 395, 425, 420),  # 区域10
            (445, 390, 465, 410),  # 区域11
            (515, 385, 535, 415)  # 区域12
        ]

    def _init_model(self):
        """初始化YOLOv5模型"""
        try:
            rospy.loginfo(f"正在加载YOLOv5模型: {self.weight_path}")
            self.model = torch.hub.load(self.yolov5_path, 'custom', self.weight_path, source='local')
            self.model.conf = self.confidence_threshold
            rospy.loginfo(f"模型加载成功! 类别: {self.model.names}")

        except Exception as e:
            rospy.logerr(f"模型加载失败: {str(e)}")
            rospy.signal_shutdown("模型加载失败，请检查路径和参数")

    def _init_publishers(self):
        """初始化ROS发布者"""
        # 水果状态信息
        self.fruit_status_pub = rospy.Publisher('/yolov5/fruit_status', FruitStatusArray, queue_size=10)

        # 点位发布
        self.point_pub = rospy.Publisher('/yolov5/point', MaturePoints, queue_size=10)

        # 可视化图像
        self.image_vis_pub = rospy.Publisher('/yolov5/vis', Image, queue_size=10)

    def _init_subscribers(self):
        """初始化ROS订阅者"""
        # 区域信息订阅
        rospy.Subscriber('/ggwp', Int32, self._region_callback)

        # 彩色图像订阅
        rospy.Subscriber("/camera/color/image_raw", Image, self._image_callback)

    def _region_callback(self, msg):
        """区域信息回调函数"""
        valid_regions = [1, 2, 3, 4, 5, 6]
        # 有效区域: 1(左地面),2(右地面),3(右果树),4(左果树),5(左地面），6(右地面)

        if msg.data in valid_regions:
            self.current_region = msg.data
            rospy.loginfo(f"当前区域更新为: {self.current_region}")
        else:
            rospy.logwarn(f"收到无效区域: {msg.data}，保持当前区域: {self.current_region}")

    def _image_callback(self, color_msg):
        """彩色图像回调函数"""
        try:
            # 转换图像格式
            bgr_img = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)

            # 验证图像尺寸
            bgr_img = self._validate_and_resize_image(bgr_img)
            if bgr_img is None:
                return

            # 同时调整RGB图像尺寸
            rgb_img = cv2.resize(rgb_img, (self.expected_width, self.expected_height))

            # 执行检测和发布
            self._detect_and_publish(rgb_img, bgr_img, color_msg.header)

        except Exception as e:
            rospy.logerr(f"图像处理异常: {str(e)}")

    def _validate_and_resize_image(self, bgr_img):
        """验证并调整图像尺寸"""
        if bgr_img.shape[:2] != (self.expected_height, self.expected_width):
            bgr_img = cv2.resize(bgr_img, (self.expected_width, self.expected_height))
        return bgr_img

    def _detect_and_publish(self, rgb_img, bgr_img, header):
        """执行目标检测并发布结果"""
        try:
            # YOLOv5推理
            results = self.model(rgb_img)
            detections = results.xyxy[0].cpu().numpy()

            # 初始化发布数据
            class_names = []
            is_mature_list = []
            mature_points = []  # 存储成熟果实点位
            vis_img = bgr_img.copy()

            # 遍历所有检测目标
            for det in detections:
                x1, y1, x2, y2, conf, cls_id = det
                cls_id = int(cls_id)
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                # 计算边界框中心点
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                # 获取类别名称
                if cls_id in self.fruit_maturity_mapping:
                    class_name = self.fruit_maturity_mapping[cls_id]['name']
                else:
                    class_name = f"unknown_{cls_id}"

                # 成熟度判断
                is_mature = self._check_maturity(bgr_img, (x1, y1, x2, y2), cls_id)

                # 记录水果状态
                class_names.append(class_name)
                is_mature_list.append(is_mature)

                # 果树区域且成熟果实: 判断点位
                # 只有3.4区域才需要判断点位，其他区域就是固定的
                if self.current_region in [3, 4] and is_mature:
                    point_id = self._get_point_location(cx, cy)
                    if point_id > 0:  # 有效点位
                        point_value = self.current_region * 100 + point_id
                        mature_points.append(point_value)
                        rospy.loginfo(f"成熟果实点位: {point_value}")

                # 可视化绘制
                color = self._get_region_color()
                cv2.rectangle(vis_img, (x1, y1), (x2, y2), color, 2)

                # 添加标签 (类别 + 成熟度)
                label = f"{class_name} {'M' if is_mature else 'I'}"
                cv2.putText(vis_img, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # 发布水果状态信息 (所有区域)
            status_msg = FruitStatusArray()
            status_msg.class_names = class_names
            status_msg.is_mature_list = is_mature_list
            self.fruit_status_pub.publish(status_msg)

            # 果树区域: 发布成熟果实点位
            if self.current_region in [3, 4]:
                points_msg = MaturePoints()
                points_msg.count = len(mature_points)

                # 添加点位
                for point_val in mature_points:
                    point_msg = MaturePoint()
                    point_msg.point = point_val
                    points_msg.mature_points.append(point_msg)

                self.point_pub.publish(points_msg)
                rospy.loginfo(f"发布点位: {mature_points}, 数量: {len(mature_points)}")

            # 发布可视化图像
            vis_msg = self.bridge.cv2_to_imgmsg(vis_img, "bgr8")
            vis_msg.header = header
            self.image_vis_pub.publish(vis_msg)

        except Exception as e:
            rospy.logerr(f"检测发布异常: {str(e)}")
            traceback.print_exc()

    def _get_region_color(self):
        """根据当前区域获取可视化颜色"""
        # 如果未设置区域，使用默认区域1的颜色
        region_colors = {
            1: (0, 255, 0),  # 左地面: 绿色
            2: (0, 255, 0),  # 右地面: 绿色
            3: (255, 0, 0),  # 右果树: 蓝色
            4: (0, 0, 255)  # 左果树: 红色
        }

        return region_colors.get(self.current_region, (200, 200, 200))  # 默认灰色

    def _get_point_location(self, cx, cy):
        """根据2D坐标返回点位编号 (1-8)"""
        # 遍历8个区域
        for idx, rect in enumerate(self.point_rects):
            x1, y1, x2, y2 = rect
            if x1 <= cx <= x2 and y1 <= cy <= y2:
                return idx + 1  # 点位编号从1开始
        return 0  # 不在任何区域

    def _check_maturity(self, bgr_img, bbox, class_id):
        """判断水果成熟度"""
        try:
            x1, y1, x2, y2 = bbox

            # 提取ROI区域
            roi = bgr_img[y1:y2, x1:x2]
            if roi.size == 0:
                return False

            # 转换到HSV色彩空间
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            # 过滤低饱和度和低亮度像素
            h_channel = hsv_roi[:, :, 0]
            s_channel = hsv_roi[:, :, 1]
            v_channel = hsv_roi[:, :, 2]

            # 创建有效像素掩码
            valid_mask = (s_channel > self.saturation_threshold) & (v_channel > self.value_threshold)
            valid_hues = h_channel[valid_mask]

            if len(valid_hues) == 0:
                return False

            # 计算色调中值
            median_hue = np.median(valid_hues)

            # 检查是否匹配成熟条件
            if class_id not in self.fruit_maturity_mapping:
                return False

            mature_ranges = self.fruit_maturity_mapping[class_id]['mature_hue_ranges']

            for hue_min, hue_max in mature_ranges:
                if hue_min <= median_hue <= hue_max:
                    return True

            return False

        except Exception as e:
            rospy.logerr(f"成熟度判断失败: {str(e)}")
            return False


def main():
    """主函数"""
    try:
        node = YoloNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被用户中断")
    finally:
        rospy.loginfo("YOLOv5水果检测节点已关闭")


if __name__ == '__main__':
    main()