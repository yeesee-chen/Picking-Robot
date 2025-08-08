#!/usr/bin/env python3
"""
简化版YOLOv5水果检测ROS节点
功能：
1. 检测水果并获取检测框中心位置
2. 计算相机坐标系下的3D坐标
3. 发布到/fruit_point话题
"""

import rospy
import cv2
import torch
import numpy as np
import traceback

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import Point
from message_filters import Subscriber, ApproximateTimeSynchronizer


class SimpleYoloNode:
    def __init__(self):
        rospy.init_node('simple_yolo_fruit_detection_node', anonymous=True)
        rospy.loginfo("正在初始化简化版YOLOv5水果检测节点...")

        # 初始化参数
        self._init_parameters()

        # 初始化组件
        self.bridge = CvBridge()
        self._init_model()
        self._init_publishers()
        self._init_subscribers()

        # 相机参数
        self.fx = self.fy = self.cx = self.cy = None
        self.camera_ready = False

        # 加载字体
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        rospy.loginfo("简化版YOLOv5水果检测节点初始化完成")

    def _init_parameters(self):
        """初始化ROS参数"""
        # 模型相关参数 - 需要修改为你的实际路径
        self.yolov5_path = rospy.get_param('~yolov5_path', '/home/nvidia/ego-planner/src/yolo_realsense_ros/yolov5')
        self.weight_path = rospy.get_param('~weight_path',
                                           '/home/nvidia/ego-planner/src/yolo_realsense_ros/weights/best.pt')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)

        # 深度相关参数
        self.depth_scale = rospy.get_param('~depth_scale', 0.001)
        self.depth_filter_size = rospy.get_param('~depth_filter_size', 5)

        # 图像尺寸参数
        self.expected_width = rospy.get_param('~image_width', 640)
        self.expected_height = rospy.get_param('~image_height', 480)

        # 可视化控制参数
        self.enable_visualization = rospy.get_param('~enable_visualization', True)

        # 水果类别与标签id映射
        self.fruit_mapping = {
            0: 'onion_0',
            1: 'onion_1',
            2: 'pumpkin_0',
            3: 'pumpkin_1',
            4: 'tomato_0',
            5: 'tomato_1',
            6: 'pepper_0',
            7: 'pepper_1',
            8: 'apple_0',
            9: 'apple_1',
            10: 'pear_1',
            11: 'pear_0'
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
        # 发布相机坐标系下的3D点
        self.point_pub = rospy.Publisher('/fruit_point', Point, queue_size=10)

        # 发布水果类别信息
        self.class_pub = rospy.Publisher('/fruit_class_ripeness', String, queue_size=10)

        # 可视化图像发布者
        if self.enable_visualization:
            self.image_vis_pub = rospy.Publisher('/yolov5/vis', Image, queue_size=10)

    def _init_subscribers(self):
        """初始化ROS订阅者"""
        # 相机参数订阅
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self._camera_info_callback)

        # 图像流同步订阅
        color_sub = Subscriber("/camera/color/image_raw", Image)
        depth_sub = Subscriber("/camera/aligned_depth_to_color/image_raw", Image)

        self.ts = ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self._synced_callback)

    def _camera_info_callback(self, msg):
        """相机参数回调函数"""
        if not self.camera_ready:
            self.fx = msg.K[0]
            self.fy = msg.K[4]
            self.cx = msg.K[2]
            self.cy = msg.K[5]
            self.camera_ready = True
            rospy.loginfo(f"相机参数已就绪: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}")

    def _synced_callback(self, color_msg, depth_msg):
        """同步图像回调函数"""
        try:
            # 等待相机参数就绪
            if not self.camera_ready:
                rospy.logwarn_throttle(5, "等待相机参数...")
                return

            # 转换图像格式
            try:
                bgr_img = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
                rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
                depth_img = self.bridge.imgmsg_to_cv2(depth_msg, depth_msg.encoding)
            except Exception as e:
                rospy.logerr(f"图像转换失败: {str(e)}")
                return

            # 验证图像尺寸
            bgr_img, depth_img = self._validate_and_resize_images(bgr_img, depth_img)
            if bgr_img is None or depth_img is None:
                return
            rgb_img = cv2.resize(rgb_img, (self.expected_width, self.expected_height))

            # 执行检测和发布
            self._detect_and_publish(rgb_img, bgr_img, depth_img, color_msg.header)

        except Exception as e:
            rospy.logerr(f"图像处理异常: {str(e)}")
            rospy.logerr(traceback.format_exc())

    def _validate_and_resize_images(self, bgr_img, depth_img):
        """验证并调整图像尺寸"""
        try:
            if bgr_img.shape[:2] != (self.expected_height, self.expected_width):
                bgr_img = cv2.resize(bgr_img, (self.expected_width, self.expected_height))

            if depth_img.shape[:2] != (self.expected_height, self.expected_width):
                depth_img = cv2.resize(depth_img, (self.expected_width, self.expected_height),
                                       interpolation=cv2.INTER_NEAREST)

            return bgr_img, depth_img

        except Exception as e:
            rospy.logerr(f"图像尺寸调整失败: {str(e)}")
            return None, None

    def _detect_and_publish(self, rgb_img, bgr_img, depth_img, header):
        """执行目标检测并发布结果"""
        try:
            # YOLOv5推理
            results = self.model(rgb_img)
            detections = results.xyxy[0].cpu().numpy()

            if len(detections) == 0:
                if self.enable_visualization:
                    self._publish_visualization(bgr_img, [], header, "No fruits detected")
                return

            # 选择置信度最高的检测结果
            best_detection = max(detections, key=lambda x: x[4])
            x1, y1, x2, y2, conf, cls_id = best_detection
            cls_id = int(cls_id)

            # 计算边界框中心点
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            # 计算相机坐标系下的3D坐标
            camera_coords = self._calculate_camera_coordinates(cx, cy, depth_img)
            if camera_coords is None:
                rospy.logwarn("无法计算3D坐标")
                return

            # 获取类别名称
            class_name = self.fruit_mapping.get(cls_id, f"unknown_{cls_id}")

            # 发布结果
            self._publish_results(camera_coords, class_name)

            # 打印日志
            rospy.loginfo(
                f"检测到水果: {class_name}, 相机坐标: ({camera_coords[0]:.3f}, {camera_coords[1]:.3f}, {camera_coords[2]:.3f})")

            # 发布可视化图像
            if self.enable_visualization:
                detection_info = {
                    'bbox': (x1, y1, x2, y2),
                    'class_name': class_name,
                    'confidence': conf,
                    'coords': camera_coords
                }
                self._publish_visualization(bgr_img, [best_detection], header, f"Detected: {class_name}",
                                            detection_info)

        except Exception as e:
            rospy.logerr(f"检测发布异常: {str(e)}")
            rospy.logerr(traceback.format_exc())

    def _calculate_camera_coordinates(self, cx, cy, depth_img):
        """计算相机坐标系下的3D坐标"""
        try:
            # 提取深度区域进行滤波
            y1 = max(0, cy - self.depth_filter_size // 2)
            y2 = min(depth_img.shape[0], cy + self.depth_filter_size // 2 + 1)
            x1 = max(0, cx - self.depth_filter_size // 2)
            x2 = min(depth_img.shape[1], cx + self.depth_filter_size // 2 + 1)

            depth_region = depth_img[y1:y2, x1:x2]
            valid_depths = depth_region[depth_region > 0]

            if len(valid_depths) == 0:
                rospy.logwarn("深度区域无有效深度值")
                return None

            # 使用中位数深度值减少噪声影响
            median_depth = np.median(valid_depths)
            z = median_depth * self.depth_scale

            # 计算相机坐标系下的坐标
            x = (cx - self.cx) * z / self.fx
            y = (cy - self.cy) * z / self.fy

            return (x, y, z)

        except Exception as e:
            rospy.logerr(f"相机坐标计算失败: {str(e)}")
            return None

    def _publish_results(self, camera_coords, class_name):
        """发布检测结果"""
        # 发布3D坐标点
        point_msg = Point()
        point_msg.x = camera_coords[0]
        point_msg.y = camera_coords[1]
        point_msg.z = camera_coords[2]
        self.point_pub.publish(point_msg)

        # 发布类别信息
        class_msg = String()
        class_msg.data = class_name
        self.class_pub.publish(class_msg)

    def _publish_visualization(self, bgr_img, detections, header, status, detection_info=None):
        """发布可视化图像"""
        if not self.enable_visualization:
            return

        try:
            vis_img = bgr_img.copy()

            # 绘制检测结果
            if len(detections) > 0 and detection_info:
                x1, y1, x2, y2 = detection_info['bbox']

                # 绘制边界框
                cv2.rectangle(vis_img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

                # 绘制中心点
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                cv2.circle(vis_img, (cx, cy), 5, (0, 0, 255), -1)

                # 添加标签和置信度
                label = f"{detection_info['class_name']} {detection_info['confidence']:.2f}"
                text_y = int(y1) - 10 if int(y1) > 25 else int(y2) + 20
                cv2.putText(vis_img, label, (int(x1), text_y), self.font, 0.6, (0, 255, 0), 2)

                # 显示相机坐标
                coords = detection_info['coords']
                coord_text = f"Cam: ({coords[0]:.3f}, {coords[1]:.3f}, {coords[2]:.3f})"
                cv2.putText(vis_img, coord_text, (10, vis_img.shape[0] - 40), self.font, 0.5, (255, 255, 255), 1)

            # 显示状态信息
            cv2.putText(vis_img, status, (10, 30), self.font, 0.7, (255, 255, 255), 2)

            # 发布图像
            vis_msg = self.bridge.cv2_to_imgmsg(vis_img, "bgr8")
            vis_msg.header = header
            self.image_vis_pub.publish(vis_msg)

        except Exception as e:
            rospy.logerr(f"可视化发布失败: {str(e)}")


def main():
    """主函数"""
    try:
        node = SimpleYoloNode()
        rospy.loginfo("简化版YOLOv5水果检测节点正在运行...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被用户中断")
    except Exception as e:
        rospy.logerr(f"节点运行异常: {str(e)}")
    finally:
        rospy.loginfo("简化版YOLOv5水果检测节点已关闭")

if __name__ == '__main__':
    main()