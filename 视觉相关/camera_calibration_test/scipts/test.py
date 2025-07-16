#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class DepthVisualizer:
    def __init__(self):
        rospy.init_node('calibration_test_node')

        self.bridge = CvBridge()
        self.window_size = rospy.get_param('~window_size', 5)

        self.sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw',
                                    Image, self.depth_callback, queue_size=1)
        self.pub = rospy.Publisher('/camera/calibration', Image, queue_size=1)

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            rospy.logerr("深度图像转换失败: {}".format(e))
            return

        h, w = depth_image.shape
        cx, cy = w // 2, h // 2
        half = self.window_size // 2
        roi = depth_image[cy-half:cy+half+1, cx-half:cx+half+1]

        valid = roi[roi > 0]
        if valid.size == 0:
            depth_mm = 0
        else:
            depth_mm = int(np.mean(valid))

        # 可视化图像（转为彩色）
        vis_img = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        vis_img = np.uint8(vis_img)
        vis_img = cv2.cvtColor(vis_img, cv2.COLOR_GRAY2BGR)

        text = "Depth: {} mm".format(depth_mm)
        cv2.putText(vis_img, text, (cx - 100, cy), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.8, (0, 255, 0), 2)
        cv2.drawMarker(vis_img, (cx, cy), (0, 0, 255), markerType=cv2.MARKER_CROSS, 
                       markerSize=10, thickness=2)

        out_msg = self.bridge.cv2_to_imgmsg(vis_img, encoding='bgr8')
        self.pub.publish(out_msg)

if __name__ == '__main__':
    try:
        DepthVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
