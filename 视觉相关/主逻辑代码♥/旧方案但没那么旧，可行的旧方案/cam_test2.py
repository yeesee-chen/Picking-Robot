import pyrealsense2 as rs
import numpy as np
import cv2
import os
from datetime import datetime

# 设置保存路径为F盘根目录
save_path = r"F:\Github\Picking-Robot\new_train_image"


# 初始化RealSense管道
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 启动相机
pipeline.start(config)

# 初始化计数器
image_counter = 1
window_name = "D435i Color Stream (S:保存, Q:退出)"

try:
    # 跳过前5帧让自动曝光稳定
    for _ in range(5):
        pipeline.wait_for_frames()

    print("相机准备就绪 - 按 'S' 保存图像, 按 'Q' 退出")

    while True:
        # 等待新帧
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # 转换为OpenCV格式
        color_image = np.asanyarray(color_frame.get_data())

        # 显示图像
        cv2.imshow(window_name, color_image)

        # 键盘输入检测 (等待1ms)
        key = cv2.waitKey(1) & 0xFF

        # 保存图像
        if key == ord('s') or key == ord('S'):
            filename = os.path.join(save_path, f"image_1_{image_counter}.png")
            cv2.imwrite(filename, color_image)
            print(f"已保存: {filename}")
            image_counter += 1

        # 退出程序
        elif key == ord('q') or key == ord('Q'):
            break

finally:
    # 清理资源
    pipeline.stop()
    cv2.destroyAllWindows()
    print(f"共保存 {image_counter - 1} 张图像到: {save_path}")
    print("程序结束")