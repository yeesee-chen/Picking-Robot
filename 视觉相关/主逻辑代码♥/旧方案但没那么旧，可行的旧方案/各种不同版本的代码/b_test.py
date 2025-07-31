#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String, Bool, Int32, Int8
import threading
from enum import Enum
import time


class SystemState(Enum):
    """系统主状态"""
    INIT = "init"
    AREA_B = "area_b"
    FINISHED = "finished"
    ERROR = "error"


class AreaBState(Enum):
    """B区状态"""
    NAVIGATE_TO_POINT = "navigate_to_point"
    SCAN_AND_GRAB = "scan_and_grab"
    MOVE_TO_NEXT = "move_to_next"
    COMPLETED = "completed"


class TaskState(Enum):
    """任务执行状态"""
    WAITING_FOR_ARRIVAL = "waiting_for_arrival"
    SETTING_OBSERVATION = "setting_observation"
    WAITING_FOR_VISION = "waiting_for_vision"
    PROCESSING_DATA = "processing_data"
    EXECUTING_ACTION = "executing_action"
    CLEANUP = "cleanup"
    COMPLETED = "completed"


class BAreaTestNode:
    def __init__(self):
        # ROS初始化
        rospy.init_node('b_area_test_node', anonymous=True)
        rospy.loginfo("B区测试节点启动中...")

        # =================== 状态变量 ===================
        self.system_state = SystemState.INIT
        self.area_b_state = AreaBState.NAVIGATE_TO_POINT
        self.task_state = TaskState.WAITING_FOR_ARRIVAL

        # =================== 数据存储 ===================
        self.init_data_storage()

        # =================== ROS通信 ===================
        self.init_ros_communication()

        # =================== 状态机配置 ===================
        self.init_state_machine_config()

        rospy.loginfo("B区测试节点初始化完成")

    def init_data_storage(self):
        """初始化数据存储"""
        # 映射字典
        self.fruit_class_to_voice_id = {
            'pumpkin': 13, 'pepper': 14, 'tomato': 15, 'onion': 16,
            'apple': 11, 'pear': 12
        }
        self.fruit_chinese_to_english = {
            '苹果': 'apple', '梨子': 'pear', '南瓜': 'pumpkin',
            '西红柿': 'tomato', '辣椒': 'pepper', '洋葱': 'onion'
        }

        # B区测试数据 - 预设二维码数据用于测试
        self.b_qr_data = ['apple', 'pear', 'pumpkin', 'tomato', 'pepper', 'onion', 'apple', 'pear']
        rospy.loginfo(f"B区测试数据: {self.b_qr_data}")

        # 视觉数据
        self.fruit_point = None
        self.fruit_class = None
        self.fruit_ripeness = None
        self.is_catchable = False

        # 导航状态
        self.current_waypoint_id = 0
        self.has_arrived = False
        self.last_published_waypoint = -1

        # 任务进度
        self.fruit_count = 0

        # B区特定
        self.b_waypoint_list = [19, 15, 18, 14, 17, 13, 16, 12]
        self.b_qr_indices = [7, 3, 6, 2, 5, 1, 4, 0]
        self.b_current_index = 0
        self.b_guancewei = 0

        # 线程锁
        self.data_lock = threading.Lock()

    def init_ros_communication(self):
        """初始化ROS通信"""
        # 订阅者
        self.class_ripeness_sub = rospy.Subscriber('/fruit_class_ripeness', String, self.vision_class_callback)
        self.point_sub = rospy.Subscriber('/fruit_point', Point, self.vision_point_callback)
        self.arrive_sub = rospy.Subscriber('/weather_arrive', Int8, self.arrival_callback)

        # 发布者
        self.arm_pub = rospy.Publisher('/robot_arm_point', String, queue_size=10)
        self.waypoint_pub = rospy.Publisher('/waypoint_i_d', Int32, queue_size=1)
        self.ggwp_pub = rospy.Publisher('/ggwp', Int32, queue_size=10)

        rospy.sleep(0.5)  # 等待连接建立

    def init_state_machine_config(self):
        """初始化状态机配置"""
        # 超时配置
        self.vision_timeout = 15  # 视觉识别超时
        self.arrival_timeout = 120  # 导航到达超时
        self.action_timeout = 10.0  # 动作执行超时

        # 状态时间戳
        self.state_start_time = time.time()
        self.id_pub_time = time.time()

        # GGWP值管理
        self.last_ggwp_value = 0
        self.ggwp_value = 0

    # =================== 回调函数 ===================
    def vision_class_callback(self, msg):
        with self.data_lock:
            if self.task_state == TaskState.WAITING_FOR_VISION:
                parts = msg.data.split('_')
                self.fruit_class = parts[0]
                self.fruit_ripeness = int(parts[1])
                self.is_catchable = (self.fruit_ripeness == 0)
                rospy.loginfo(f"接收到视觉分类: {self.fruit_class}, 成熟度: {self.fruit_ripeness}")

    def vision_point_callback(self, msg):
        with self.data_lock:
            if self.task_state == TaskState.WAITING_FOR_VISION:
                self.fruit_point = msg
                rospy.loginfo(f"接收到果实坐标: x={msg.x}, y={msg.y}, z={msg.z}")

    def arrival_callback(self, msg):
        with self.data_lock:
            self.has_arrived = (msg.data == 1)
            if self.has_arrived:
                rospy.loginfo(f"到达航点: {self.current_waypoint_id}")

    # =================== 状态机核心 ===================
    def run_state_machine(self):
        """状态机主循环"""
        rate = rospy.Rate(40)

        while not rospy.is_shutdown():
            try:
                # 发布当前航点
                self.publish_current_waypoint()

                # 发布视觉指令
                self.publish_vision_command()

                # 主状态机调度
                if self.system_state == SystemState.INIT:
                    self.handle_init_state()
                elif self.system_state == SystemState.AREA_B:
                    self.handle_area_b()
                elif self.system_state == SystemState.FINISHED:
                    self.handle_finished_state()
                elif self.system_state == SystemState.ERROR:
                    self.handle_error_state()

                # 检查超时
                self.check_timeouts()

            except Exception as e:
                rospy.logerr(f"状态机运行错误: {e}")
                self.transition_to_error()

            rate.sleep()

    # =================== 系统状态处理 ===================
    def handle_init_state(self):
        """处理初始化状态"""
        rospy.loginfo("B区测试系统初始化...")
        self.arm_pub.publish("语音:17;")  # 初始语音播报
        self.arm_pub.publish("观测位:0;")
        rospy.sleep(2)

        # 直接转换到B区
        self.transition_to_area_b()

    def handle_finished_state(self):
        """处理完成状态"""
        rospy.loginfo("B区测试完成！")
        rospy.loginfo(f"总共抓取了 {self.fruit_count} 个果实")
        rospy.sleep(0.1)

    def handle_error_state(self):
        """处理错误状态"""
        rospy.logerr("系统进入错误状态")
        rospy.sleep(1)

    # =================== B区状态处理 ===================
    def handle_area_b(self):
        """B区状态机"""
        if self.area_b_state == AreaBState.NAVIGATE_TO_POINT:
            self.handle_b_navigate()
        elif self.area_b_state == AreaBState.SCAN_AND_GRAB:
            self.handle_b_scan_and_grab()
        elif self.area_b_state == AreaBState.MOVE_TO_NEXT:
            self.handle_b_move_to_next()
        elif self.area_b_state == AreaBState.COMPLETED:
            self.transition_to_finished()

    def handle_b_navigate(self):
        """前往当前 B 区航点"""
        if self.b_current_index == 0:
            # 第一次进入 B 区，前往航点 19
            self.set_waypoint(self.b_waypoint_list[0])
            rospy.loginfo(f"前往B区第一个航点: {self.b_waypoint_list[0]}")

        if self.has_arrived:
            rospy.loginfo(f"已到达B区航点: {self.b_waypoint_list[self.b_current_index]}")
            self.area_b_state = AreaBState.SCAN_AND_GRAB
            self.task_state = TaskState.SETTING_OBSERVATION

    def handle_b_scan_and_grab(self):
        """在当前航点处理抓取"""
        idx = self.b_current_index
        wp = self.b_waypoint_list[idx]
        expected = self.b_qr_data[self.b_qr_indices[idx]]

        rospy.loginfo(f"当前航点: {wp}, 期望水果: {expected}")

        # self.task_state = TaskState.SETTING_OBSERVATION
        self.b_guancewei = self.generate_b_observation(wp)

        # 执行观测任务
        success = self.execute_observation_task(self.b_guancewei)

        if success:
            # 检查识别到的水果是否匹配期望
            if self.fruit_class == expected:
                rospy.loginfo(f"水果匹配成功: {self.fruit_class} == {expected}")
                if self.should_grab_fruit():
                    # 根据高度调整机械臂
                    # if self.fruit_point and self.fruit_point.z <= 0.17:  # 转换为米
                    #     self.arm_pub.publish("机械臂:10,0,90;")
                    #     rospy.sleep(1)
                    self.execute_grab_action()
                else:
                    rospy.loginfo("水果不可抓取（未成熟）")
                    self.arm_pub.publish("观测位:0;")
                    rospy.sleep(1)
            else:
                rospy.loginfo(f"水果不匹配: {self.fruit_class} != {expected}")
                self.arm_pub.publish("观测位:0;")
                rospy.sleep(1)

        self.area_b_state = AreaBState.MOVE_TO_NEXT

    def handle_b_move_to_next(self):
        """移动到下一条目"""
        self.b_current_index += 1

        if self.b_current_index >= len(self.b_waypoint_list):
            rospy.loginfo("B区所有航点已完成")
            self.area_b_state = AreaBState.COMPLETED
        else:
            next_wp = self.b_waypoint_list[self.b_current_index]
            self.set_waypoint(next_wp)
            rospy.loginfo(f"前往下一个B区航点: {next_wp}")
            self.area_b_state = AreaBState.NAVIGATE_TO_POINT
            self.task_state = TaskState.WAITING_FOR_ARRIVAL

    # =================== 通用任务执行 ===================
    def execute_observation_task(self, observation_pos):
        """执行观测任务的通用流程"""
        if self.task_state == TaskState.SETTING_OBSERVATION:
            self.reset_vision_data()  # 清空旧数据
            self.arm_pub.publish(f"观测位:{observation_pos};")
            rospy.sleep(0.4)  # 相机/云台稳定
            rospy.loginfo(f"设置观测位: {observation_pos}")
            self.task_state = TaskState.WAITING_FOR_VISION
            self.state_start_time = time.time()
            return False

        elif self.task_state == TaskState.WAITING_FOR_VISION:
            rospy.sleep(2)
            if self.fruit_class and self.fruit_point:
                rospy.loginfo("接收到完整视觉数据，进入处理阶段")
                self.task_state = TaskState.PROCESSING_DATA
                return False
            elif time.time() - self.state_start_time > self.vision_timeout:
                rospy.logwarn("视觉识别超时，跳过当前目标")
                self.reset_vision_data()
                self.task_state = TaskState.COMPLETED
                return True
            return False

        elif self.task_state == TaskState.PROCESSING_DATA:
            self.broadcast_fruit_info()
            self.task_state = TaskState.COMPLETED
            return True

        elif self.task_state == TaskState.COMPLETED:
            self.task_state = TaskState.WAITING_FOR_ARRIVAL
            return True

        return False

    def execute_grab_action(self):
        """执行抓取动作"""
        if self.fruit_point and self.is_catchable:
            rospy.loginfo("b区测试demo 不进行抓取 只进行导航和语音测试")
        #
        #     # 计算机械臂参数
        #     r = self.fruit_point.x * 100
        #     if r > 46:
        #         r = 46
        #     z = self.fruit_point.z * 100
        #     phi = self.fruit_point.y
        #
        #     rospy.loginfo(f"机械臂参数: r={r}, z={z}, phi={phi}")
        #
        #     # 发送坐标给机械臂
        #     self.arm_pub.publish(f"机械臂:{r},{z},{phi};")
        #     rospy.sleep(3)
        #
        #     # 执行抓取序列
        #     self.arm_pub.publish("爪子:0;")  # 爪子收缩
        #     rospy.loginfo("爪子收缩")
        #     rospy.sleep(2)
        #
        #     self.arm_pub.publish("观测位:0;")  # 复位
        #     rospy.loginfo("机械臂复位")
        #     rospy.sleep(2)
        #
        #     self.arm_pub.publish("爪子:1;")  # 爪子张开
        #     rospy.loginfo("爪子张开")
        #     rospy.sleep(2)
        #
        #     self.fruit_count += 1
        #     rospy.loginfo(f"成功抓取，当前数量: {self.fruit_count}")
        # else:
        #     rospy.loginfo("不满足抓取条件")

        self.reset_vision_data()
        return True

    # =================== 状态转换 ===================
    def transition_to_area_b(self):
        """转换到B区"""
        self.system_state = SystemState.AREA_B
        self.area_b_state = AreaBState.NAVIGATE_TO_POINT
        self.b_current_index = 0
        rospy.loginfo("转换到B区状态")

    def transition_to_finished(self):
        """转换到完成状态"""
        self.system_state = SystemState.FINISHED
        rospy.loginfo("B区测试完成")

    def transition_to_error(self):
        """转换到错误状态"""
        self.system_state = SystemState.ERROR
        rospy.logerr("系统转换到错误状态")

    # =================== 辅助函数 ===================
    def set_waypoint(self, waypoint_id):
        """设置航点"""
        if waypoint_id != self.current_waypoint_id:
            self.current_waypoint_id = waypoint_id
            self.has_arrived = False
            rospy.loginfo(f"设置新航点: {waypoint_id}")
            self.id_pub_time = time.time()

    def publish_current_waypoint(self):
        """发布当前航点"""
        if self.current_waypoint_id != self.last_published_waypoint:
            self.waypoint_pub.publish(self.current_waypoint_id)
            self.last_published_waypoint = self.current_waypoint_id

    def publish_vision_command(self):
        """发布视觉指令"""
        self.ggwp_value = self.generate_ggwp_value()
        if self.ggwp_value != self.last_ggwp_value:
            self.ggwp_pub.publish(self.ggwp_value)
            self.last_ggwp_value = self.ggwp_value

    def reset_vision_data(self):
        """重置视觉数据"""
        self.fruit_class = None
        self.fruit_point = None
        self.fruit_ripeness = None
        self.is_catchable = False

    def should_grab_fruit(self):
        """判断是否应该抓取果实"""
        return self.is_catchable and self.fruit_point is not None

    def broadcast_fruit_info(self):
        """播报果实信息"""
        if self.fruit_class in self.fruit_class_to_voice_id:
            class_id = self.fruit_class_to_voice_id[self.fruit_class]
            self.arm_pub.publish(f"语音:{class_id};")
            rospy.loginfo(f"播报水果类型: {self.fruit_class}")

    def check_timeouts(self):
        """检查各种超时"""
        current_time = time.time()
        navigate_timeout = current_time - self.id_pub_time > self.arrival_timeout and self.has_arrived == 0

        if current_time - self.state_start_time > 180:  # 总体超时保护
            rospy.logwarn("状态执行超时，可能需要人工干预")
        elif navigate_timeout:  # 导航超时保护
            self.has_arrived = 1
            rospy.logwarn("导航执行超时，手工设置航点标志位为1！")

    def generate_b_observation(self, waypoint):
        """生成B区观测位指令"""
        if waypoint in {16, 17, 18, 19}:  # 左侧航点
            return 3
        elif waypoint in {12, 13, 14, 15}:  # 右侧航点
            return 4
        return 0

    def generate_ggwp_value(self):
        """生成视觉识别指令"""
        if self.system_state == SystemState.AREA_B:
            if self.area_b_state == AreaBState.SCAN_AND_GRAB:
                return self.b_guancewei
        return 0


def main():
    try:
        node = BAreaTestNode()
        rospy.loginfo("开始B区测试...")
        node.run_state_machine()
    except rospy.ROSInterruptException:
        rospy.loginfo("B区测试程序被中断")
    except Exception as e:
        rospy.logerr(f"B区测试程序运行失败: {e}")


if __name__ == '__main__':
    main()