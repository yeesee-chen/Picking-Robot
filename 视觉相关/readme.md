一些可能出现的问题
---------------------------------------------------------------------------
1.编译之前先进入src/yolo_realsense_ros/yolov5文件夹内运行detect文件测试环境
python3 detect.py
出现小火箭表示环境没问题
有问题查看报错根据报错搜索解决
----------------------------------------------------------------------------
2.编译必须使用这串代码，每次修改完之后要重新编译，特别是修改msg文件
catkin_make --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
----------------------------------------------------------------------------
3.新建终端必须source一下
source devel/setup.bash
----------------------------------------------------------------------------
4.有新的scripts文件，就是使用新的py代码时要在launch文件里面修改
type="yolo_node4.py"（更换为自己的py文件）
----------------------------------------------------------------------------
5.深度相机必须用usb3.0的线连接
----------------------------------------------------------------------------
6.65，66行需要更改为自己的路径，使用绝对路径
self.yolov5_path = rospy.get_param('~yolov5_path', '/home/nvidia/chu_ws/src/yolo_realsense_ros/yolov5')
self.weight_path = rospy.get_param('~weight_path', '/home/nvidia/chu_ws/src/yolo_realsense_ros/weights/best.pt')


如何使用：
1.先编译，编译成功之后source，然后运行下面代码启动摄像头：
roslaunch realsense2_camera rs_camera_chu.launch
新建终端，cd工作空间，source，运行以下代码开启检测
roslaunch yolo_realsense_ros yolo_node.launch
这一步可能会出一些问题，拍照给我看怎么解决或者有一些可以搜搜
----------------------------------------------------------------------------
可视化：
运行终端，输入rviz回车，在rviz中add-by topic，可以看原始图像，深度图像和运行yolo后的图像。
yolo节点的图像发布是yolov5_vis，注意每个话题都是选择image
----------------------------------------------------------------------------
查看msg文件：
新建终端，cd工作空间，source，运行以下代码：
rostopic echo yolov5/3d_detection

