=========总体说明=========
realsense-ros是相机要用的包，不需要更改
使用
roslaunch realsense2_camera rs_camera_chu.launch启动
--------------------------------
yolo_realsense_ros是yolo的包
launch中存放launch文件，这个版本的代码运行不需要用到；
scripts中存放的是调用yolo的代码；
weights中放的是yolo的模型，自己制作的模型都是best.pt没有改名字，现在存放的是最新的；
yolov5是yolo本体，使用的版本是yolov5-5.0不需要更改，测试有没有yolo环境可以在这个文件夹内运行python3 detect.py测试
使用
rosrun yolo_realsense_ros (scripts中的某一代码名称)启动
比如现在代码的启动：
rosrun yolo_realsense_ros yolo_node.py
===========更改和使用说明============
需要更改yolo_node.py中模型相关参数为自己的路径，注意路径不要写错，最好在属性里面复制（49到51行左右）
-------------------------------
/fruit_point 为坐标
/fruit_class_ripeness 为标签
--------------------------------
单独改代码不需要重新编译，但是必须在python3环境中编译
使用：
catkin_make --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
进行编译
--------------------------------
深度相机必须用usb3.0的线连接
----------------------------------------------------------------------------
可视化：
运行终端，输入rviz回车，在rviz中add-by topic，可以看原始图像，深度图像和运行yolo后的图像。
yolo节点的图像发布是yolov5_vis，注意每个话题都是选择image
----------------------------------------------------------------------------