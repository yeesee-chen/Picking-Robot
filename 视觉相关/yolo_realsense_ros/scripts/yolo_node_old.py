#!/usr/bin/env python3
"""
YOLOv5水果检测ROS节点 - 颜色空间修复版
"""

import rospy
import cv2
import torch
import numpy as np
import traceback
import os

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from std_msgs.msg import Header, Int32, String
from costom_msgs.msg import detection3d, FruitStatusArray, MatureFruit, MatureFruitArray
from message_filters import Subscriber, ApproximateTimeSynchronizer


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

        # 相机参数
        self.fx = self.fy = self.cx = self.cy = None
        self.camera_ready = False

        # 小车位置和预设观测位坐标
        self.current_position = 1  # 默认位置1
        self.observation_positions = {
            1: (0.8, 27.2, 11),  # 位置1的观测位坐标 (x_init, y_init, z_init)
            2: (-0.8, -26.2, 11),
            3: (-1.68, -3.74, 19.2),
            4: (1.68, 3.74, 19.2),
            5: (0.8, 27.2, 11),
            6: (-0.8, -26.2, 11)
        }

        # 加载字体（解决文本乱码问题）
        self.font_path = self._get_font_path()
        if self.font_path:
            self.font = cv2.FONT_HERSHEY_SIMPLEX
            rospy.loginfo(f"使用系统字体: {self.font_path}")
        else:
            self.font = cv2.FONT_HERSHEY_SIMPLEX
            rospy.logwarn("未找到中文字体，使用默认字体")

        rospy.loginfo("YOLOv5水果检测节点初始化完成")

    def _get_font_path(self):
        """获取系统字体路径"""
        font_dirs = [
            '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',  # Ubuntu
            '/usr/share/fonts/truetype/freefont/FreeSans.ttf',  # Debian
            '/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc'  # 中文字体
        ]

        for font_path in font_dirs:
            if os.path.exists(font_path):
                return font_path
        return None

    def _init_parameters(self):
        """初始化ROS参数"""
        # 模型相关参数
        # -------------------
        # 需要将模型参数路径修改
        # -------------------
        self.yolov5_path = rospy.get_param('~yolov5_path', '/home/nvidia/chu_ws/src/yolo_realsense_ros/yolov5')
        self.weight_path = rospy.get_param('~weight_path', '/home/nvidia/chu_ws/src/yolo_realsense_ros/weights/best.pt')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)

        # 深度相关参数
        self.depth_scale = rospy.get_param('~depth_scale', 0.001)
        self.depth_filter_size = rospy.get_param('~depth_filter_size', 5)

        # 成熟度判断参数
        self.saturation_threshold = rospy.get_param('~saturation_threshold', 40)
        self.value_threshold = rospy.get_param('~value_threshold', 40)

        # 图像尺寸参数
        self.expected_width = rospy.get_param('~image_width', 640)
        self.expected_height = rospy.get_param('~image_height', 480)

        # 可视化控制参数
        # 不知道为什么这个参数加入launch文件就报错，所以这个参数只在这里调整
        self.enable_visualization = rospy.get_param('~enable_visualization', True)

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
        # 原始3D检测结果（所有检测目标）
        self.detection_pub = rospy.Publisher('/yolov5/3d_detection', detection3d, queue_size=10)
        # 水果相关
        self.fruit_status_pub = rospy.Publisher('/fruit_class_ripeness', String, queue_size=1)
        self.fruit_point_pub = rospy.Publisher('/fruit_point', Point, queue_size=1)

        # 可视化图像
        self.image_vis_pub = rospy.Publisher('/yolov5/vis', Image, queue_size=10)

    def _init_subscribers(self):
        """初始化ROS订阅者"""
        # 相机参数订阅
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self._camera_info_callback)

        # 小车位置订阅
        rospy.Subscriber('/ggwp', Int32, self._position_callback)

        # 图像流同步订阅
        color_sub = Subscriber("/camera/color/image_raw", Image)
        depth_sub = Subscriber("/camera/aligned_depth_to_color/image_raw", Image)

        self.ts = ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self._synced_callback)

    def _position_callback(self, msg):
        """小车位置回调函数"""
        position = msg.data
        if position in range(1, 7):  # 确保位置在1-6之间
            self.current_position = position
            rospy.loginfo(f"更新小车位置: 位置 {position}")
        else:
            rospy.logwarn(f"收到无效位置: {position}，保持当前位置 {self.current_position}")

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
                # 原始图像为BGR格式
                bgr_img = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")

                # 转换为RGB用于YOLOv5检测
                rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)

                depth_img = self.bridge.imgmsg_to_cv2(depth_msg, depth_msg.encoding)
            except Exception as e:
                rospy.logerr(f"图像转换失败: {str(e)}")
                return

            # 验证图像尺寸
            bgr_img, depth_img = self._validate_and_resize_images(bgr_img, depth_img)
            if bgr_img is None or depth_img is None:
                return

            # 同时调整RGB图像尺寸
            rgb_img = cv2.resize(rgb_img, (self.expected_width, self.expected_height))

            # 执行检测和发布：使用RGB图像检测，BGR图像用于可视化
            self._detect_and_publish(rgb_img, bgr_img, depth_img, color_msg.header)

        except Exception as e:
            rospy.logerr(f"图像处理异常: {str(e)}")
            rospy.logerr(traceback.format_exc())

    def _validate_and_resize_images(self, bgr_img, depth_img):
        """验证并调整图像尺寸"""
        try:
            # 检查彩色图像尺寸
            if bgr_img.shape[:2] != (self.expected_height, self.expected_width):
                rospy.logwarn(
                    f"调整彩色图像尺寸: {bgr_img.shape[:2]} -> ({self.expected_height}, {self.expected_width})")
                bgr_img = cv2.resize(bgr_img, (self.expected_width, self.expected_height))

            # 检查深度图像尺寸
            if depth_img.shape[:2] != (self.expected_height, self.expected_width):
                rospy.logwarn(
                    f"调整深度图像尺寸: {depth_img.shape[:2]} -> ({self.expected_height}, {self.expected_width})")
                depth_img = cv2.resize(depth_img, (self.expected_width, self.expected_height),
                                       interpolation=cv2.INTER_NEAREST)

            return bgr_img, depth_img

        except Exception as e:
            rospy.logerr(f"图像尺寸调整失败: {str(e)}")
            rospy.logerr(traceback.format_exc())
            return None, None

    def _detect_and_publish(self, rgb_img, bgr_img, depth_img, header):
        """执行目标检测并发布结果
        Args:
            rgb_img: 用于YOLOv5检测的RGB图像
            bgr_img: 用于可视化绘制的BGR图像
            depth_img: 深度图像
            header: 消息头
        """
        # 只有全部的消息才需要消息头，其他消息不需要消息头
        try:
            # YOLOv5推理
            results = self.model(rgb_img)
            detections = results.xyxy[0].cpu().numpy()

            if len(detections) > 0:
                rospy.loginfo(f"检测到 {len(detections)} 个目标")

            # 处理检测结果
            detection_list = []
            mature_fruits_list = []
            class_names = []
            is_mature_list = []

            # 遍历所有检测目标
            for det in detections:
                x1, y1, x2, y2, conf, cls_id = det
                cls_id = int(cls_id)

                # 计算边界框中心点
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                # 计算3D坐标
                world_coords = self._calculate_3d_coordinates(cx, cy, depth_img)
                if world_coords is None:
                    continue

                x, y, z = world_coords

                # 获取类别名称
                if cls_id in self.fruit_maturity_mapping:
                    class_name = self.fruit_maturity_mapping[cls_id]['name']
                else:
                    class_name = f"unknown_{cls_id}"

                # 成熟度判断（使用BGR图像）
                is_mature = self._check_maturity(bgr_img, (int(x1), int(y1), int(x2), int(y2)), cls_id)

                # 创建检测消息
                detection_msg = detection3d()
                detection_msg.header = header
                detection_msg.class_name = class_name
                detection_msg.x = float(x)
                detection_msg.y = float(y)
                detection_msg.z = float(z)
                detection_msg.is_mature = is_mature

                detection_list.append(detection_msg)
                class_names.append(class_name)
                is_mature_list.append(is_mature)

                # 如果是成熟水果
                if is_mature:
                    # 成熟水果坐标
                    mature_fruit = MatureFruit()
                    mature_fruit.x = float(x)
                    mature_fruit.y = float(y)
                    mature_fruit.z = float(z)
                    mature_fruits_list.append(mature_fruit)

            # 发布所有检测结果
            for detection_msg in detection_list:
                self.detection_pub.publish(detection_msg)

            # 发布成熟水果检测结果
            if mature_fruits_list:
                # 所有的成熟水果坐标+个数count
                mature_array_msg = MatureFruitArray()
                mature_array_msg.mature_fruits = mature_fruits_list
                mature_array_msg.count = len(mature_fruits_list)
                self.mature_detection_pub.publish(mature_array_msg)

            # 发布水果状态信息
            if class_names or is_mature_list:
                # 类别+是否成熟
                # self._publish_fruit_status(header, class_names, is_mature_list)
                self._publish_fruit_status(class_names, is_mature_list)

            # 发布可视化图像（使用BGR图像）
            if self.enable_visualization:
                self._publish_visualization(bgr_img, detections, header)

        except Exception as e:
            rospy.logerr(f"检测发布异常: {str(e)}")
            rospy.logerr(traceback.format_exc())

    def _calculate_3d_coordinates(self, cx, cy, depth_img):
        """计算3D世界坐标并转换到机械臂基坐标系"""
        try:
            # 提取深度区域
            y1 = max(0, cy - self.depth_filter_size // 2)
            y2 = min(depth_img.shape[0], cy + self.depth_filter_size // 2 + 1)
            x1 = max(0, cx - self.depth_filter_size // 2)
            x2 = min(depth_img.shape[1], cx + self.depth_filter_size // 2 + 1)

            depth_region = depth_img[y1:y2, x1:x2]

            # 过滤无效深度值
            valid_depths = depth_region[depth_region > 0]
            if len(valid_depths) == 0:
                return None

            # 计算中值深度
            median_depth = np.median(valid_depths)
            z = median_depth * self.depth_scale
            z_cam = z

            # 计算相机坐标系下的坐标
            x_cam = (cx - self.cx) * z / self.fx
            y_cam = (cy - self.cy) * z / self.fy

            # 获取当前观测位坐标
            if self.current_position in self.observation_positions:
                x_init, y_init, z_init = self.observation_positions[self.current_position]
            else:
                rospy.logwarn(f"未知位置 {self.current_position}，使用默认位置1")
                x_init, y_init, z_init = self.observation_positions[1]

            # 加入一个当观测位为3、4时，由于相机有旋转，将相机坐标旋转之后得到新相机坐标进行覆盖
            if self.current_position == 3 or self.current_position ==4:
                # x_cam的坐标不变，以为是基于x轴旋转的
                x_cam = x_cam
                # y,z坐标全部都乘cos15
                y_cam = y_cam * 0.966
                z_cam = z_cam * 0.966

            # 相机坐标系转机械臂基坐标系
            # 注意：根据需求调整坐标轴对应关系
            # 原说明：x_base=x_cam+观测位x；y_base=z_cam+观测位y；z_base=y_cam+观测位z
            # 注：相机坐标系上负下正左负右正
            x_base = x_cam + x_init
            y_base = -(z_cam + y_init)  # 深度方向z_cam对应y_base
            z_base = y_cam + z_init  # 相机y轴对应z_base

            # 调试用
            # rho = x_cam
            # phi_deg = y_cam
            # z_base = z_cam

            # 转换为圆柱坐标系 (rho, phi, z)
            rho = np.sqrt(x_base ** 2 + y_base ** 2)
            phi = np.arctan2(y_base, x_base)  # 弧度
            phi_deg = np.degrees(phi)  # 角度


            # 返回圆柱坐标系下的坐标 (r, φ, z)
            return rho, phi_deg, z_base

        except Exception as e:
            rospy.logerr(f"3D坐标计算失败: {str(e)}")
            return None

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

    def _publish_fruit_status(self, class_names, is_mature_list):
        """发布水果状态信息"""
        try:
            status_msg = FruitStatusArray()
            status_msg.class_names = class_names
            status_msg.is_mature_list = is_mature_list
            self.fruit_status_pub.publish(status_msg)
        except Exception as e:
            rospy.logerr(f"发布水果状态失败: {str(e)}")

    def _publish_visualization(self, bgr_img, detections, header):
        """发布可视化图像"""
        try:
            vis_img = bgr_img.copy()

            if len(detections) == 0:
                status_text = "Status: No fruits detected"
                cv2.putText(vis_img, status_text, (10, 30),
                            self.font, 0.7, (0, 255, 255), 2)
            else:
                mature_count = 0
                total_count = len(detections)

                for det in detections:
                    x1, y1, x2, y2, conf, cls_id = det
                    cls_id = int(cls_id)

                    # 获取类别名称
                    if cls_id in self.fruit_maturity_mapping:
                        class_name = self.fruit_maturity_mapping[cls_id]['name']
                    else:
                        class_name = f"unknown_{cls_id}"

                    # 成熟度判断
                    is_mature = self._check_maturity(bgr_img, (int(x1), int(y1), int(x2), int(y2)), cls_id)
                    if is_mature:
                        mature_count += 1

                    # 绘制边界框
                    color = (0, 255, 0) if is_mature else (0, 0, 255)  # 绿色成熟，红色未成熟
                    cv2.rectangle(vis_img, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)

                    # 添加标签（避免使用特殊符号）
                    status_char = "M" if is_mature else "I"  # M:成熟, I:未成熟
                    label = f"{class_name} {status_char} {conf:.2f}"

                    # 计算文本位置
                    text_y = int(y1) - 10 if int(y1) > 25 else int(y2) + 20
                    cv2.putText(vis_img, label, (int(x1), text_y),
                                self.font, 0.5, color, 1)

                # 绘制状态栏（英文避免乱码）
                status_text = f"Detected: {total_count} fruits, Mature: {mature_count}"
                cv2.putText(vis_img, status_text, (10, 30),
                            self.font, 0.7, (0, 255, 255), 2)

                # 添加位置信息
                pos_text = f"Position: {self.current_position}"
                cv2.putText(vis_img, pos_text, (10, 60),
                            self.font, 0.7, (255, 0, 0), 2)

            # 发布可视化图像（BGR格式）
            vis_msg = self.bridge.cv2_to_imgmsg(vis_img, "bgr8")
            vis_msg.header = header
            self.image_vis_pub.publish(vis_msg)

        except Exception as e:
            rospy.logerr(f"可视化发布失败: {str(e)}")


def main():
    """主函数"""
    try:
        node = YoloNode()
        rospy.loginfo("YOLOv5水果检测节点正在运行...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被用户中断")
    except Exception as e:
        rospy.logerr(f"节点运行异常: {str(e)}")
    finally:
        rospy.loginfo("YOLOv5水果检测节点已关闭")


if __name__ == '__main__':
    main()