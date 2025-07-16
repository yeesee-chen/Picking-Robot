#!/usr/bin/env python3
"""
YOLOv5水果检测ROS节点 - 优化版
功能：
1. 只发布最左侧水果（x坐标最小）
2. 坐标稳定性检查，防止抖动
3. 避免重复发布相同坐标
4. 发布后暂停2秒机制
5. 只发布标签和坐标信息
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
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header, Int32, String, Bool
from geometry_msgs.msg import Point
from message_filters import Subscriber, ApproximateTimeSynchronizer


class OptimizedYoloNode:
    def __init__(self):
        rospy.init_node('optimized_yolo_fruit_detection_node', anonymous=True)
        rospy.loginfo("正在初始化优化版YOLOv5水果检测节点...")

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
        self.current_region = 1
        self.observation_positions = {
            1: (0.008, 0.272, 0.11),
            2: (-0.008, -0.262, 0.11),
            3: (-0.0168, -0.0374, 0.192),
            4: (0.0168, 0.0374, 0.192),
            5: (0.008, 0.272, 0.11),
            6: (-0.008, -0.262, 0.11)
        }

        # 稳定性检查相关参数
        self.stability_buffer = deque(maxlen=self.stability_frames)  # 存储最近几帧的检测结果
        self.last_published_coord = None  # 上次发布的坐标
        self.last_publish_time = 0  # 上次发布的时间戳
        self.is_in_pause = False  # 是否处于暂停状态

        # 加载字体
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        rospy.loginfo("优化版YOLOv5水果检测节点初始化完成")


    def _init_parameters(self):
        """初始化ROS参数"""
        # 模型相关参数
        # 参数需要修改为自定义的文件夹路径
        self.yolov5_path = rospy.get_param('~yolov5_path', '/home/nvidia/ego-planner/src/yolo_realsense_ros/yolov5')
        self.weight_path = rospy.get_param('~weight_path', '/home/nvidia/ego-planner/src/yolo_realsense_ros/weights/best.pt')
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
        self.enable_visualization = rospy.get_param('~enable_visualization', True)

        # 新增：稳定性和发布控制参数
        self.stability_frames = rospy.get_param('~stability_frames', 5)  # 需要连续稳定的帧数
        self.stability_threshold = rospy.get_param('~stability_threshold', 0.05)  # 稳定性阈值（米）
        self.duplicate_threshold = rospy.get_param('~duplicate_threshold', 0.05)  # 重复坐标判断阈值（米）
        self.pause_duration = rospy.get_param('~pause_duration', 2.0)  # 暂停时长（秒）

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
        # 新的发布者：使用标准消息类型
        # point是标准类型geometry_msgs中的一种坐标点类型
        self.point_pub = rospy.Publisher('/fruit_point', Point, queue_size=10)
        self.class_ripeness_pub = rospy.Publisher('/fruit_class_ripeness', String, queue_size=10)

        # 可视化图像发布者
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
        if position in range(1, 7):
            self.current_region = position
            rospy.loginfo(f"更新小车位置: 位置 {position}")
        else:
            rospy.logwarn(f"收到无效位置: {position}，保持当前位置 {self.current_region}")

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
            # 检查是否在暂停期
            current_time = time.time()
            if self.is_in_pause:
                if current_time - self.last_publish_time < self.pause_duration:
                    # 仍在暂停期，只做可视化，不进行检测发布
                    if self.enable_visualization:
                        bgr_img = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
                        self._publish_pause_visualization(bgr_img, color_msg.header)
                    return
                else:
                    # 暂停期结束
                    self.is_in_pause = False
                    rospy.loginfo("暂停期结束，重新开始检测")

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
                # 没有检测到目标，清空稳定性缓冲区
                self.stability_buffer.clear()
                if self.enable_visualization:
                    self._publish_visualization(bgr_img, [], header, "No fruits detected")
                return

            # 找到最左侧的水果（x坐标最小）
            leftmost_detection = self._find_leftmost_fruit(detections)
            if leftmost_detection is None:
                self.stability_buffer.clear()
                return

            # 计算最左侧水果的3D坐标
            x1, y1, x2, y2, conf, cls_id = leftmost_detection
            cls_id = int(cls_id)

            # 计算边界框中心点
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            # 计算3D坐标
            world_coords = self._calculate_3d_coordinates(cx, cy, depth_img)
            if world_coords is None:
                self.stability_buffer.clear()
                return

            # 获取类别名称和成熟度
            class_name = self.fruit_maturity_mapping[cls_id]['name']

            # 创建检测结果
            # 将结果作为一个列表
            detection_result = {
                'position': world_coords,
                'class_ripeness': class_name,
                'bbox': (x1, y1, x2, y2)
            }

            # 检查稳定性
            if self._check_stability(detection_result):
                # 坐标稳定，检查是否与上次发布的坐标重复
                if not self._is_duplicate_coordinate(world_coords):
                    # 发布结果
                    self._publish_detection_result(detection_result)

                    # 更新状态
                    self.last_published_coord = world_coords
                    self.last_publish_time = time.time()
                    self.is_in_pause = True

                    rospy.loginfo(f"发布水果检测结果: 位置{world_coords}, 类别: {class_name}")
                    rospy.loginfo(f"进入{self.pause_duration}秒暂停期...")
                else:
                    rospy.loginfo("检测到重复坐标，跳过发布")

            # 发布可视化图像
            if self.enable_visualization:
                status = "Stable - Published" if len(
                    self.stability_buffer) >= self.stability_frames else f"Stabilizing {len(self.stability_buffer)}/{self.stability_frames}"
                self._publish_visualization(bgr_img, [leftmost_detection], header, status, detection_result)

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

    def _check_stability(self, detection_result):
        """检查检测结果的稳定性"""
        # 添加到稳定性缓冲区
        self.stability_buffer.append(detection_result)
        # 如果缓冲区未满，不够稳定
        if len(self.stability_buffer) < self.stability_frames:
            return False
        # 检查最近几帧的坐标是否都在阈值范围内
        positions = [result['position'] for result in self.stability_buffer]
        # 计算坐标的标准差
        rho_values = [pos[0] for pos in positions]
        phi_values = [pos[1] for pos in positions]
        z_values = [pos[2] for pos in positions]
        rho_std = np.std(rho_values)
        phi_std = np.std(phi_values)
        z_std = np.std(z_values)
        # 如果所有坐标分量的标准差都小于阈值，则认为稳定
        is_stable = (rho_std < self.stability_threshold and
                     phi_std < self.stability_threshold and
                     z_std < self.stability_threshold)
        return is_stable

    def _is_duplicate_coordinate(self, current_coord):
        """检查是否与上次发布的坐标重复"""
        if self.last_published_coord is None:
            return False

        # 计算距离
        rho1, phi1, z1 = current_coord
        rho2, phi2, z2 = self.last_published_coord

        distance = np.sqrt((rho1 - rho2) ** 2 + (phi1 - phi2) ** 2 + (z1 - z2) ** 2)

        return distance < self.duplicate_threshold

    def _publish_detection_result(self, detection_result):
        """发布检测结果"""
        # 发布位置
        position_msg = Point()
        position_msg.x = detection_result['position'][0]  # rho
        position_msg.y = detection_result['position'][1]  # phi
        position_msg.z = detection_result['position'][2]  # z
        self.point_pub.publish(position_msg)

        # 发布类别
        class_ripeness_msg = String()
        class_ripeness_msg.data = detection_result['class_ripeness']
        self.class_ripeness_pub.publish(class_ripeness_msg)

    def _calculate_3d_coordinates(self, cx, cy, depth_img):
        """计算3D世界坐标并转换到机械臂基坐标系"""
        x_base = None
        y_base = None
        z_base = None
        try:
            # 提取深度区域
            y1 = max(0, cy - self.depth_filter_size // 2)
            y2 = min(depth_img.shape[0], cy + self.depth_filter_size // 2 + 1)
            x1 = max(0, cx - self.depth_filter_size // 2)
            x2 = min(depth_img.shape[1], cx + self.depth_filter_size // 2 + 1)

            depth_region = depth_img[y1:y2, x1:x2]
            valid_depths = depth_region[depth_region > 0]

            if len(valid_depths) == 0:
                return None

            median_depth = np.median(valid_depths)
            z = median_depth * self.depth_scale
            z_cam = z

            # 计算相机坐标系下的坐标
            x_cam = (cx - self.cx) * z / self.fx
            y_cam = (cy - self.cy) * z / self.fy

            # 获取当前观测位坐标，如果有/ggwp消息就接收，如果没有默认1
            # if self.current_region in self.observation_positions:
            #     x_init, y_init, z_init = self.observation_positions[self.current_region]
            # else:
            #     x_init, y_init, z_init = self.observation_positions[2]

            x_init, y_init, z_init = self.observation_positions[2]
            flag = 1

            # 位置1
            if flag == 1 :
                # 相机坐标系转机械臂基坐标系
                x_base = x_cam + x_init
                y_base = y_init + y_cam
                z_base = -0.018
                rho = np.sqrt(x_base ** 2 + y_base ** 2)
                rho = rho + 0.05
                phi = np.arctan2(y_base, x_base)
                phi_deg = np.degrees(phi)
                phi_deg = 180 + phi_deg - 7

            # 位置2
            if flag == 2 :
                x_base = x_init - x_cam
                y_base = y_init + y_cam
                z_base = -0.018
                rho = np.sqrt(x_base ** 2 + y_base ** 2)
                rho = rho + 0.05
                phi = np.arctan2(y_base, x_base)
                phi_deg = np.degrees(phi)
                phi_deg = phi_deg + 8.0


            # # 处理位置3、4的相机旋转
            # if self.current_region in (3, 4) :
            #     x_cam = x_cam
            #     y_cam = y_cam * 0.966
            #     z_cam = z_cam * 0.966

            # 转换为圆柱坐标系
            

            return rho, phi_deg, z_base

        except Exception as e:
            rospy.logerr(f"3D坐标计算失败: {str(e)}")
            return None

    def _publish_visualization(self, bgr_img, detections, header, status, detection_result=None):
        """发布可视化图像"""
        try:
            vis_img = bgr_img.copy()

            # 绘制检测结果
            if len(detections) > 0 and detection_result:
                det = detections[0]  # 只有一个检测结果（最左侧）
                x1, y1, x2, y2, conf, cls_id = det

                # 绘制边界框
                color = (0, 255, 0)
                cv2.rectangle(vis_img, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)

                # 添加标签
                label = f"{detection_result['class_ripeness']}"
                text_y = int(y1) - 10 if int(y1) > 25 else int(y2) + 20
                cv2.putText(vis_img, label, (int(x1), text_y), self.font, 0.5, color, 1)

                # 绘制坐标信息
                coord_text = f"Pos: ({detection_result['position'][0]:.2f}, {detection_result['position'][1]:.2f}, {detection_result['position'][2]:.2f})"
                cv2.putText(vis_img, coord_text, (10, vis_img.shape[0] - 60), self.font, 0.5, (255, 255, 255), 1)

            # 发布图像
            vis_msg = self.bridge.cv2_to_imgmsg(vis_img, "bgr8")
            vis_msg.header = header
            self.image_vis_pub.publish(vis_msg)

        except Exception as e:
            rospy.logerr(f"可视化发布失败: {str(e)}")

    def _publish_pause_visualization(self, bgr_img, header):
        """发布暂停期间的可视化图像"""
        try:
            vis_img = bgr_img.copy()

            # 计算剩余暂停时间
            remaining_time = self.pause_duration - (time.time() - self.last_publish_time)
            remaining_time = max(0, remaining_time)

            # 绘制暂停状态
            pause_text = f"PAUSED - {remaining_time:.1f}s remaining"
            cv2.putText(vis_img, pause_text, (10, 30), self.font, 0.7, (0, 0, 255), 2)
            cv2.putText(vis_img, f"Position: {self.current_region}", (10, 60), self.font, 0.7, (255, 0, 0), 2)

            # 发布图像
            vis_msg = self.bridge.cv2_to_imgmsg(vis_img, "bgr8")
            vis_msg.header = header
            self.image_vis_pub.publish(vis_msg)

        except Exception as e:
            rospy.logerr(f"暂停可视化发布失败: {str(e)}")

def main():
    """主函数"""
    try:
        node = OptimizedYoloNode()
        rospy.loginfo("优化版YOLOv5水果检测节点正在运行...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被用户中断")
    except Exception as e:
        rospy.logerr(f"节点运行异常: {str(e)}")
    finally:
        rospy.loginfo("优化版YOLOv5水果检测节点已关闭")


if __name__ == '__main__':
    main()