#!/usr/bin/env python3
"""
YOLOv5水果检测ROS节点 - 2D坐标左右转动版
功能：
1. 根据目标中心点位（2d坐标），判断x坐标是否在中间位置，
如果在，那么返回"停止"，如果偏左，那么返回"左"如果偏右，返回"右"
2. 只发布标签和判断坐标位置信息，将这两个信息分开发布到/fruit_class_ripeness和/fruit_move_msg
"""

import rospy
import cv2
import torch
import numpy as np
import traceback
import os
import time
from collections import deque

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header, Int32, String, Bool


class OptimizedYoloNode:
    def __init__(self):
        rospy.init_node('optimized_yolo_fruit_detection_node', anonymous=True)
        rospy.loginfo("正在初始化YOLOv5水果检测节点 - 2D坐标版...")

        # 初始化参数
        self._init_parameters()

        # 初始化组件
        self.bridge = CvBridge()
        self._init_model()
        self._init_publishers()
        self._init_subscribers()

        # 小车位置
        self.current_region = 1

        # 已删除稳定性检查相关参数

        # 加载字体
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        rospy.loginfo("YOLOv5水果检测节点初始化完成 - 2D坐标版")

    def _init_parameters(self):
        """初始化ROS参数"""
        # 模型相关参数
        self.yolov5_path = rospy.get_param('~yolov5_path', '/home/nvidia/ego-planner/src/yolo_realsense_ros/yolov5')
        self.weight_path = rospy.get_param('~weight_path',
                                           '/home/nvidia/ego-planner/src/yolo_realsense_ros/weights/best.pt')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)

        # 图像尺寸参数
        self.expected_width = rospy.get_param('~image_width', 640)
        self.expected_height = rospy.get_param('~image_height', 480)

        # 可视化控制参数
        self.enable_visualization = rospy.get_param('~enable_visualization', True)

        # 已删除稳定性和发布控制参数

        # 2D坐标位置判断参数
        self.center_tolerance = rospy.get_param('~center_tolerance', 50)  # 中心区域容忍度（像素）
        self.image_center_x = self.expected_width // 2  # 图像中心x坐标

        # 水果类别与标签id映射
        self.fruit_maturity_mapping = {
            0: {'name': 'onion_0'},
            1: {'name': 'onion_1'},
            2: {'name': 'pumpkin_0'},
            3: {'name': 'pumpkin_1'},
            4: {'name': 'tomato_0'},
            5: {'name': 'tomato_1'},
            6: {'name': 'pepper_0'},
            7: {'name': 'pepper_1'},
            8: {'name': 'apple_0'},
            9: {'name': 'apple_1'},
            10: {'name': 'pear_1'},
            11: {'name': 'pear_0'}
        }

    def _init_model(self):
        """初始化YOLOv5模型"""
        try:
            rospy.loginfo(f"正在加载YOLOv5模型: {self.weight_path}")
            self.model = torch.hub.load(self.yolov5_path, 'custom', self.weight_path, source='local')
            self.model.conf = self.confidence_threshold

            if hasattr(self.model, 'names') and self.model.names:
                rospy.loginfo(f"模型加载成功! 类别: {self.model.names}")
            else:
                rospy.logwarn("模型加载成功，但未获取到类别名称")

        except Exception as e:
            rospy.logerr(f"模型加载失败: {str(e)}")
            rospy.logerr(traceback.format_exc())
            rospy.signal_shutdown("模型加载失败，请检查路径和参数")

    def _init_publishers(self):
        """初始化ROS发布者"""
        # 类别和成熟度信息发布者
        self.class_ripeness_pub = rospy.Publisher('/fruit_class_ripeness', String, queue_size=10)

        # 移动指令发布者
        self.move_msg_pub = rospy.Publisher('/fruit_move_msg', String, queue_size=10)

        # 可视化图像发布者
        self.image_vis_pub = rospy.Publisher('/yolov5/vis', Image, queue_size=10)

    def _init_subscribers(self):
        """初始化ROS订阅者"""
        # 小车位置订阅
        rospy.Subscriber('/ggwp', Int32, self._position_callback)

        # 图像流订阅 - 修改为单独的彩色图像订阅
        rospy.Subscriber("/camera/color/image_raw", Image, self._image_callback)

    def _position_callback(self, msg):
        """小车位置回调函数"""
        position = msg.data
        if position in range(1, 7):
            self.current_region = position
            rospy.loginfo(f"更新小车位置: 位置 {position}")
        else:
            rospy.logwarn(f"收到无效位置: {position}，保持当前位置 {self.current_region}")

    def _image_callback(self, color_msg):
        """图像回调函数"""
        try:
            # 转换图像格式
            try:
                bgr_img = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
                rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
            except Exception as e:
                rospy.logerr(f"图像转换失败: {str(e)}")
                return

            # 验证并调整图像尺寸
            bgr_img = self._validate_and_resize_image(bgr_img)
            if bgr_img is None:
                return
            rgb_img = cv2.resize(rgb_img, (self.expected_width, self.expected_height))

            # 执行检测和发布
            self._detect_and_publish(rgb_img, bgr_img, color_msg.header)

        except Exception as e:
            rospy.logerr(f"图像处理异常: {str(e)}")
            rospy.logerr(traceback.format_exc())

    def _validate_and_resize_image(self, bgr_img):
        """验证并调整图像尺寸"""
        try:
            if bgr_img.shape[:2] != (self.expected_height, self.expected_width):
                bgr_img = cv2.resize(bgr_img, (self.expected_width, self.expected_height))
            return bgr_img
        except Exception as e:
            rospy.logerr(f"图像尺寸调整失败: {str(e)}")
            return None

    def _detect_and_publish(self, rgb_img, bgr_img, header):
        """执行目标检测并发布结果"""
        try:
            # YOLOv5推理
            results = self.model(rgb_img)
            detections = results.xyxy[0].cpu().numpy()

            if len(detections) == 0:
                # 没有检测到目标
                if self.enable_visualization:
                    self._publish_visualization(bgr_img, [], header, "No fruits detected")
                return

            # 找到最左侧的水果（x坐标最小）
            leftmost_detection = self._find_leftmost_fruit(detections)
            if leftmost_detection is None:
                return

            # 解析检测结果
            x1, y1, x2, y2, conf, cls_id = leftmost_detection
            cls_id = int(cls_id)

            # 计算边界框中心点
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            # 获取类别名称
            class_name = self.fruit_maturity_mapping[cls_id]['name']

            # 判断移动方向
            move_direction = self._determine_move_direction(cx)

            # 创建检测结果
            detection_result = {
                'center_x': cx,
                'center_y': cy,
                'class_ripeness': class_name,
                'move_direction': move_direction,
                'bbox': (x1, y1, x2, y2),
                'confidence': conf
            }

            # 发布结果（总是发布类别信息，只有_0标签才发布移动指令）
            self._publish_detection_result(detection_result)

            # 发布可视化图像
            if self.enable_visualization:
                self._publish_visualization(bgr_img, [leftmost_detection], header, "Detecting", detection_result)

        except Exception as e:
            rospy.logerr(f"检测发布异常: {str(e)}")
            rospy.logerr(traceback.format_exc())

    def _find_leftmost_fruit(self, detections):
        """找到最左侧的水果（x坐标最小）"""
        if len(detections) == 0:
            return None
        # 按x1坐标排序，选择最左侧的
        leftmost_idx = np.argmin(detections[:, 0])
        return detections[leftmost_idx]

    def _determine_move_direction(self, center_x):
        """根据目标中心点x坐标判断移动方向"""
        # 计算与图像中心的偏移
        offset = center_x - self.image_center_x

        if abs(offset) <= self.center_tolerance:
            return "停止"
        elif offset < -self.center_tolerance:
            return "左"
        else:
            return "右"

    def _publish_detection_result(self, detection_result):
        """发布检测结果"""
        # 总是发布类别和成熟度信息
        class_ripeness_msg = String()
        class_ripeness_msg.data = detection_result['class_ripeness']
        self.class_ripeness_pub.publish(class_ripeness_msg)

        # 只有标签以_0结尾的才发布移动指令
        if detection_result['class_ripeness'].endswith('_0'):
            move_msg = String()
            move_msg.data = detection_result['move_direction']
            self.move_msg_pub.publish(move_msg)
            rospy.loginfo(
                f"发布移动指令: {detection_result['move_direction']} (类别: {detection_result['class_ripeness']})")
        else:
            rospy.loginfo(f"检测到 {detection_result['class_ripeness']}，不发布移动指令")

    def _publish_visualization(self, bgr_img, detections, header, status, detection_result=None):
        """发布可视化图像"""
        try:
            vis_img = bgr_img.copy()

            # 绘制图像中心线和容忍区域
            center_x = self.image_center_x
            # 绘制中心线
            cv2.line(vis_img, (center_x, 0), (center_x, vis_img.shape[0]), (255, 255, 0), 2)
            # 绘制容忍区域
            left_bound = center_x - self.center_tolerance
            right_bound = center_x + self.center_tolerance
            cv2.line(vis_img, (left_bound, 0), (left_bound, vis_img.shape[0]), (0, 255, 255), 1)
            cv2.line(vis_img, (right_bound, 0), (right_bound, vis_img.shape[0]), (0, 255, 255), 1)

            # 绘制检测结果
            if len(detections) > 0 and detection_result:
                det = detections[0]  # 只有一个检测结果（最左侧）
                x1, y1, x2, y2, conf, cls_id = det

                # 根据是否会发布移动指令来选择颜色
                is_moveable = detection_result['class_ripeness'].endswith('_0')
                color = (0, 255, 0) if is_moveable else (128, 128, 128)  # 绿色表示会发布移动指令，灰色表示不会

                # 绘制边界框
                cv2.rectangle(vis_img, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)

                # 绘制中心点
                cx, cy = detection_result['center_x'], detection_result['center_y']
                cv2.circle(vis_img, (cx, cy), 5, (0, 0, 255), -1)  # 红色圆点

                # 添加标签
                label = f"{detection_result['class_ripeness']} ({conf:.2f})"
                text_y = int(y1) - 10 if int(y1) > 25 else int(y2) + 20
                cv2.putText(vis_img, label, (int(x1), text_y), self.font, 0.5, color, 1)

                # 绘制移动方向信息（仅当会发布移动指令时显示）
                if is_moveable:
                    move_text = f"Direction: {detection_result['move_direction']}"
                    cv2.putText(vis_img, move_text, (10, vis_img.shape[0] - 60), self.font, 0.6, (0, 255, 0), 2)
                else:
                    no_move_text = "No movement command (not _0)"
                    cv2.putText(vis_img, no_move_text, (10, vis_img.shape[0] - 60), self.font, 0.6, (128, 128, 128), 2)

                # 绘制中心坐标信息
                coord_text = f"Center: ({cx}, {cy})"
                cv2.putText(vis_img, coord_text, (10, vis_img.shape[0] - 30), self.font, 0.5, (255, 255, 255), 1)

            # 绘制状态信息
            cv2.putText(vis_img, f"Status: {status}", (10, 30), self.font, 0.6, (0, 255, 0), 2)
            cv2.putText(vis_img, f"Position: {self.current_region}", (10, 60), self.font, 0.5, (255, 0, 0), 1)

            # 发布图像
            vis_msg = self.bridge.cv2_to_imgmsg(vis_img, "bgr8")
            vis_msg.header = header
            self.image_vis_pub.publish(vis_msg)

        except Exception as e:
            rospy.logerr(f"可视化发布失败: {str(e)}")


def main():
    """主函数"""
    try:
        node = OptimizedYoloNode()
        rospy.loginfo("YOLOv5水果检测节点正在运行 - 2D坐标版...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被用户中断")
    except Exception as e:
        rospy.logerr(f"节点运行异常: {str(e)}")
    finally:
        rospy.loginfo("YOLOv5水果检测节点已关闭 - 2D坐标版")


if __name__ == '__main__':
    main()