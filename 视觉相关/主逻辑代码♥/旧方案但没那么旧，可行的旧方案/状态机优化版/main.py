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
    AREA_A = "area_a"
    AREA_B = "area_b"
    AREA_C = "area_c"
    FINISHED = "finished"
    ERROR = "error"


class AreaAState(Enum):
    """A区状态"""
    INIT_BROADCAST = "init_broadcast"
    NAVIGATE_TO_POINT = "navigate_to_point"
    OBSERVE_LEFT = "observe_left"
    OBSERVE_RIGHT = "observe_right"
    PROCESS_RESULT = "process_result"
    GRAB_FRUIT = "grab_fruit"
    MOVE_TO_NEXT = "move_to_next"
    COMPLETED = "completed"


class AreaBState(Enum):
    """B区状态"""
    NAVIGATE_TO_POINT = "navigate_to_point"
    SCAN_AND_GRAB = "scan_and_grab"
    MOVE_TO_NEXT = "move_to_next"
    COMPLETED = "completed"


class AreaCState(Enum):
    """C区状态"""
    GOTO_B_QR = "goto_b_qr"
    SCAN_B_QR = "scan_b_qr"
    GOTO_C_QR = "goto_c_qr"
    SCAN_C_QR = "scan_c_qr"
    PLAN_TASKS = "plan_tasks"
    NAVIGATE_TO_TARGET = "navigate_to_target"
    EXECUTE_GRAB = "execute_grab"
    MOVE_TO_NEXT_TARGET = "move_to_next_target"
    DUMP_FRUITS = "dump_fruits"
    RETURN_TO_START = "return_to_start"
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


class StateMachineNode:
    def __init__(self):
        # ROS初始化
        rospy.init_node('state_machine_node', anonymous=True)
        rospy.loginfo("状态机节点启动中...")

        # =================== 状态变量 ===================
        self.system_state = SystemState.INIT
        self.area_a_state = AreaAState.INIT_BROADCAST
        self.area_b_state = AreaBState.NAVIGATE_TO_POINT
        self.area_c_state = AreaCState.GOTO_B_QR
        self.task_state = TaskState.WAITING_FOR_ARRIVAL

        # =================== 数据存储 ===================
        self.init_data_storage()

        # =================== ROS通信 ===================
        self.init_ros_communication()

        # =================== 状态机配置 ===================
        self.init_state_machine_config()

        rospy.loginfo("状态机节点初始化完成")

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

        # 二维码数据
        self.b_qr_data = []  # B区水果列表
        self.c_qr_data = []  # C区蔬菜列表
        self.c_number_sequence = ""  # C区数字序列

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
        self.current_observation_side = 0  # 0=左, 1=右

        # A区特定
        self.a_waypoint_list = [1, 3, 5, 7]
        self.a_current_index = 0

        # B区特定
        self.b_waypoint_list = [19, 15, 18, 14, 17, 13, 16, 12]
        self.b_qr_indices = [7, 3, 6, 2, 5, 1, 4, 0]
        self.b_current_index = 0

        # C区特定
        self.c_task_list = []
        self.c_current_index = 0
        self.guancewei = 0
        self.current_observation_mode = 1  # 观测位模式

        # 线程锁
        self.data_lock = threading.Lock()

    def init_ros_communication(self):
        """初始化ROS通信"""
        # 订阅者
        self.class_ripeness_sub = rospy.Subscriber('/fruit_class_ripeness', String, self.vision_class_callback)
        self.point_sub = rospy.Subscriber('/fruit_point', Point, self.vision_point_callback)
        self.arrive_sub = rospy.Subscriber('/weather_arrive', Int8, self.arrival_callback)
        self.qr_sub = rospy.Subscriber('/qr_arm_message', String, self.qr_callback)

        # 发布者
        self.arm_pub = rospy.Publisher('/arm_voice', String, queue_size=10)
        self.waypoint_pub = rospy.Publisher('/waypoint_i_d', Int32, queue_size=1)
        self.ggwp_pub = rospy.Publisher('/ggwp', Int32, queue_size=10)

        rospy.sleep(0.5)  # 等待连接建立

    def init_state_machine_config(self):
        """初始化状态机配置"""
        # 状态转换表
        self.system_transitions = {
            SystemState.INIT: [SystemState.AREA_A],
            SystemState.AREA_A: [SystemState.AREA_C, SystemState.ERROR],
            SystemState.AREA_C: [SystemState.AREA_B, SystemState.ERROR],
            SystemState.AREA_B: [SystemState.FINISHED, SystemState.ERROR],
        }

        # 超时配置
        self.vision_timeout = 10.0  # 视觉识别超时
        self.arrival_timeout = 30.0  # 导航到达超时
        self.action_timeout = 10.0  # 动作执行超时

        # 状态时间戳
        self.state_start_time = time.time()

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

    def qr_callback(self, msg):
        with self.data_lock:
            qr_data = msg.data.strip()
            if self.area_c_state == AreaCState.SCAN_B_QR:
                self.process_b_qr_data(qr_data)
            elif self.area_c_state == AreaCState.SCAN_C_QR:
                self.process_c_qr_data(qr_data)

    # =================== 状态机核心 ===================
    def run_state_machine(self):
        """状态机主循环"""
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            try:
                # 发布当前航点
                self.publish_current_waypoint()

                # 发布视觉指令
                self.publish_vision_command()

                # 主状态机调度
                if self.system_state == SystemState.INIT:
                    self.handle_init_state()
                elif self.system_state == SystemState.AREA_A:
                    self.handle_area_a()
                elif self.system_state == SystemState.AREA_B:
                    self.handle_area_b()
                elif self.system_state == SystemState.AREA_C:
                    self.handle_area_c()
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
        rospy.loginfo("系统初始化...")
        self.arm_pub.publish("语音:17;")  # 初始语音播报
        rospy.sleep(3)
        self.arm_pub.publish("动作组:0;")  # 初始动作
        rospy.sleep(2)

        # 转换到A区
        self.transition_to_area_a()

    def handle_finished_state(self):
        """处理完成状态"""
        rospy.loginfo("所有任务完成！")
        rospy.sleep(1)

    def handle_error_state(self):
        """处理错误状态"""
        rospy.logerr("系统进入错误状态")
        # 可以添加错误恢复逻辑
        rospy.sleep(1)

    # =================== A区状态处理 ===================
    def handle_area_a(self):
        """A区状态机"""
        if self.area_a_state == AreaAState.INIT_BROADCAST:
            self.handle_a_init_broadcast()
        elif self.area_a_state == AreaAState.NAVIGATE_TO_POINT:
            self.handle_a_navigate()
        elif self.area_a_state == AreaAState.OBSERVE_LEFT:
            self.handle_a_observe_left()
        elif self.area_a_state == AreaAState.OBSERVE_RIGHT:
            self.handle_a_observe_right()
        elif self.area_a_state == AreaAState.PROCESS_RESULT:
            self.handle_a_process_result()
        elif self.area_a_state == AreaAState.GRAB_FRUIT:
            self.handle_a_grab_fruit()
        elif self.area_a_state == AreaAState.MOVE_TO_NEXT:
            self.handle_a_move_to_next()
        elif self.area_a_state == AreaAState.COMPLETED:
            self.transition_to_area_c()

    def handle_a_init_broadcast(self):
        """A区初始播报"""
        self.set_waypoint(self.a_waypoint_list[0])
        self.area_a_state = AreaAState.NAVIGATE_TO_POINT
        self.current_observation_side = 0
        rospy.loginfo("A区开始，前往第一个航点")

    def handle_a_navigate(self):
        """A区导航状态"""
        if self.has_arrived:
            if self.current_observation_side == 0:
                self.area_a_state = AreaAState.OBSERVE_LEFT
            else:
                self.area_a_state = AreaAState.OBSERVE_RIGHT
            self.task_state = TaskState.SETTING_OBSERVATION

    def handle_a_observe_left(self):
        """A区左侧观测"""
        success = self.execute_observation_task(1)  # 左侧观测位
        if success:
            if self.should_grab_fruit():
                self.area_a_state = AreaAState.GRAB_FRUIT
            else:
                self.current_observation_side = 1
                self.area_a_state = AreaAState.OBSERVE_RIGHT

    def handle_a_observe_right(self):
        """A区右侧观测"""
        success = self.execute_observation_task(2)  # 右侧观测位
        if success:
            if self.should_grab_fruit():
                self.area_a_state = AreaAState.GRAB_FRUIT
            else:
                self.area_a_state = AreaAState.MOVE_TO_NEXT

    # def handle_a_process_result(self):
    #     """处理收到的水果信息"""

    def handle_a_grab_fruit(self):
        """A区抓取果实"""
        if self.execute_grab_action():
            if self.current_observation_side == 0:
                self.current_observation_side = 1
                self.area_a_state = AreaAState.OBSERVE_RIGHT
            else:
                self.area_a_state = AreaAState.MOVE_TO_NEXT

    def handle_a_move_to_next(self):
        """A区移动到下一点"""
        self.a_current_index += 1
        if self.a_current_index >= len(self.a_waypoint_list):
            self.area_a_state = AreaAState.COMPLETED
        else:
            self.set_waypoint(self.a_waypoint_list[self.a_current_index])
            self.current_observation_side = 0
            self.area_a_state = AreaAState.NAVIGATE_TO_POINT

    # =================== C区状态处理 ===================
    def handle_area_c(self):
        """C区状态机"""
        if self.area_c_state == AreaCState.GOTO_B_QR:
            self.handle_c_goto_b_qr()
        elif self.area_c_state == AreaCState.SCAN_B_QR:
            self.handle_c_scan_b_qr()
        elif self.area_c_state == AreaCState.GOTO_C_QR:
            self.handle_c_goto_c_qr()
        elif self.area_c_state == AreaCState.SCAN_C_QR:
            self.handle_c_scan_c_qr()
        elif self.area_c_state == AreaCState.PLAN_TASKS:
            self.handle_c_plan_tasks()
        elif self.area_c_state == AreaCState.NAVIGATE_TO_TARGET:
            self.handle_c_navigate()
        elif self.area_c_state == AreaCState.EXECUTE_GRAB:
            self.handle_c_execute_grab()
        elif self.area_c_state == AreaCState.MOVE_TO_NEXT_TARGET:
            self.handle_c_move_to_next()
        elif self.area_c_state == AreaCState.DUMP_FRUITS:
            self.handle_c_dump_fruits()
        elif self.area_c_state == AreaCState.RETURN_TO_START:
            self.handle_c_return()
        elif self.area_c_state == AreaCState.COMPLETED:
            self.transition_to_area_b()

    def handle_c_goto_b_qr(self):
        """前往B区二维码"""
        self.set_waypoint(11)
        self.area_c_state = AreaCState.SCAN_B_QR

    def handle_c_scan_b_qr(self):
        """扫描B区二维码"""
        if self.has_arrived and len(self.b_qr_data) > 0:
            self.area_c_state = AreaCState.GOTO_C_QR

    def handle_c_plan_tasks(self):
        """规划C区任务"""
        self.c_task_list = self.replan_c_task()
        self.c_current_index = 0
        if len(self.c_task_list) > 0:
            self.area_c_state = AreaCState.NAVIGATE_TO_TARGET
        else:
            self.area_c_state = AreaCState.COMPLETED

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

    # =================== 通用任务执行 ===================
    def execute_observation_task(self, observation_pos):
        """执行观测任务的通用流程"""
        if self.task_state == TaskState.SETTING_OBSERVATION:
            self.arm_pub.publish(f"观测位:{observation_pos};")
            self.task_state = TaskState.WAITING_FOR_VISION
            self.state_start_time = time.time()
            return False

        elif self.task_state == TaskState.WAITING_FOR_VISION:
            if self.fruit_class and self.fruit_point:
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
            # 发送坐标给机械臂
            r = self.fruit_point.x * 100
            z = self.fruit_point.z * 100
            phi = self.fruit_point.y
            self.arm_pub.publish(f"机械臂:{r},{z},{phi};")

            # 执行抓取序列
            rospy.sleep(2)
            self.arm_pub.publish("爪子:0;")
            rospy.sleep(2)
            self.arm_pub.publish("动作组:0;")
            rospy.sleep(4)
            self.arm_pub.publish("爪子:1;")
            rospy.sleep(2)

            self.fruit_count += 1
            rospy.loginfo(f"成功抓取，当前数量: {self.fruit_count}")

        self.reset_vision_data()
        return True

    # =================== 状态转换 ===================
    def transition_to_area_a(self):
        """转换到A区"""
        self.system_state = SystemState.AREA_A
        self.area_a_state = AreaAState.INIT_BROADCAST
        rospy.loginfo("转换到A区状态")

    def transition_to_area_b(self):
        """转换到B区"""
        self.system_state = SystemState.AREA_B
        self.area_b_state = AreaBState.NAVIGATE_TO_POINT
        self.set_waypoint(20)  # B区起始点
        rospy.loginfo("转换到B区状态")

    def transition_to_area_c(self):
        """转换到C区"""
        self.system_state = SystemState.AREA_C
        self.area_c_state = AreaCState.GOTO_B_QR
        rospy.loginfo("转换到C区状态")

    def transition_to_finished(self):
        """转换到完成状态"""
        self.system_state = SystemState.FINISHED
        rospy.loginfo("所有任务完成")

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

    def publish_current_waypoint(self):
        """发布当前航点"""
        if self.current_waypoint_id != self.last_published_waypoint:
            self.waypoint_pub.publish(self.current_waypoint_id)
            self.last_published_waypoint = self.current_waypoint_id

    def publish_vision_command(self):
        """发布视觉指令"""
        ggwp_value = self.generate_ggwp_value()
        if ggwp_value > 0:
            self.ggwp_pub.publish(ggwp_value)

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
        if self.system_state == SystemState.AREA_A:
            # A区播报成熟度
            ripeness_id = 19 if self.fruit_ripeness else 18
            self.arm_pub.publish(f"语音:{ripeness_id};")
        elif self.system_state == SystemState.AREA_B:
            # B区播报类别
            if self.fruit_class in self.fruit_class_to_voice_id:
                class_id = self.fruit_class_to_voice_id[self.fruit_class]
                self.arm_pub.publish(f"语音:{class_id};")
        elif self.system_state == SystemState.AREA_C:
            # C区播报类别和成熟度
            if self.fruit_class in self.fruit_class_to_voice_id:
                class_id = self.fruit_class_to_voice_id[self.fruit_class]
                ripeness_id = 19 if self.fruit_ripeness else 18
                self.arm_pub.publish(f"语音:{class_id};")
                rospy.sleep(1)
                self.arm_pub.publish(f"语音:{ripeness_id};")

    def check_timeouts(self):
        """检查各种超时"""
        current_time = time.time()
        if current_time - self.state_start_time > 60:  # 总体超时保护
            rospy.logwarn("状态执行超时，可能需要人工干预")

    # =================== 数据处理函数 ===================
    def process_b_qr_data(self, qr_data):
        """处理B区二维码数据"""
        lines = [line.strip() for line in qr_data.split('\n') if line.strip()]
        self.b_qr_data = []
        for fruit in lines:
            if fruit in self.fruit_chinese_to_english:
                self.b_qr_data.append(self.fruit_chinese_to_english[fruit])
        rospy.loginfo(f"B区二维码解析完成: {self.b_qr_data}")

    def process_c_qr_data(self, qr_data):
        """处理C区二维码数据"""
        lines = [line.strip() for line in qr_data.split('\n') if line.strip()]
        if len(lines) >= 9:
            self.c_qr_data = lines[:8]
            self.c_number_sequence = lines[-1]
        rospy.loginfo(f"C区二维码解析完成: 蔬菜{len(self.c_qr_data)}个, 序列: {self.c_number_sequence}")

    def generate_ggwp_value(self):
        """生成视觉识别指令"""
        # 根据当前航点和状态生成相应的视觉指令
        # 这里需要根据您的具体需求实现
        return 0

    def replan_c_task(self):
        """重新规划C区任务"""
        test_strings = self.c_number_sequence
        if not test_strings:
            return []

        parts = test_strings.split(',')

        # 验证输入格式
        try:
            sequence = [int(x) for x in parts]
        except ValueError:
            print("错误：输入包含非数字字符")
            return []

        # 验证数字范围
        if not all(1 <= x <= 12 for x in sequence):
            print("错误：数字必须在1-12范围内")
            return []

        if len(sequence) != 8:
            print("错误：序列长度必须为8")
            return []

        result = []
        flag = 0  # 用于跟踪某种状态

        # 判断起始位置：1-4为右边，5-12为左边
        def is_right_side(pos):
            return pos in (1, 2, 3, 4)

        def is_left_side(pos):
            return pos in (5, 6, 7, 8, 9, 10, 11, 12)

        # 确定初始方向标志
        flag11 = 1 if is_right_side(sequence[0]) else 0

        for i in range(len(sequence)):
            c_now = sequence[i]
            c_next = sequence[i + 1] if i < len(sequence) - 1 else None

            # 处理位置1-8的情况
            if c_now in (1, 2, 3, 4, 5, 6, 7, 8):
                # 如果当前在右边且之前flag为0，则添加特殊指令
                if flag == 0 and flag11 == 1:
                    result.append(20)
                    flag = 1

                if c_next and c_next in (1, 2, 3, 4, 5, 6, 7, 8):
                    if flag11 == 1:  # 在右边
                        c_now_id = c_now + 23
                        if c_now in (1, 2, 3, 4):
                            self.guancewei = 2
                        elif c_now in (5, 6, 7, 8):
                            if c_now_id in (28, 29, 30, 31):
                                self.guancewei = 1
                            else:
                                self.guancewei = 2
                        else:
                            self.guancewei = 1
                        result.append(c_now_id)
                    else:  # 在左边
                        if c_now in (1, 2, 3, 4):
                            c_now_id = c_now + 23
                            if c_now in (1, 2, 3, 4):
                                self.guancewei = 2
                            elif c_now in (5, 6, 7, 8):
                                if c_now_id in (28, 29, 30, 31):
                                    self.guancewei = 1
                                else:
                                    self.guancewei = 2
                            else:
                                self.guancewei = 1
                            result.append(c_now_id)
                        elif c_now in (5, 6, 7, 8):
                            c_now_id = c_now + 27
                            if c_now in (1, 2, 3, 4):
                                self.guancewei = 2
                            elif c_now in (5, 6, 7, 8):
                                if c_now_id in (28, 29, 30, 31):
                                    self.guancewei = 1
                                else:
                                    self.guancewei = 2
                            else:
                                self.guancewei = 1
                            result.append(c_now_id)
                            if c_next in (1, 2, 3, 4):
                                result.extend([21, 20])

                elif c_next and c_next in (9, 10, 11, 12):
                    c_now_id = c_now + 23
                    if c_now in (1, 2, 3, 4):
                        self.guancewei = 2
                    elif c_now in (5, 6, 7, 8):
                        if c_now_id in (28, 29, 30, 31):
                            self.guancewei = 1
                        else:
                            self.guancewei = 2
                    else:
                        self.guancewei = 1
                    result.extend([c_now_id, 20, 21])

                else:
                    # 处理序列末尾的情况
                    if c_next is None:
                        c_now_id = c_now + 23
                        if c_now in (1, 2, 3, 4):
                            self.guancewei = 2
                        elif c_now in (5, 6, 7, 8):
                            if c_now_id in (28, 29, 30, 31):
                                self.guancewei = 1
                            else:
                                self.guancewei = 2
                        else:
                            self.guancewei = 1
                        result.append(c_now_id)
                    else:
                        rospy.loginfo(f"未处理的情况：位置{i}，当前值{c_now}")

            # 处理位置9-12的情况
            elif c_now in (9, 10, 11, 12):
                c_now_id = c_now + 23
                if c_now in (1, 2, 3, 4):
                    self.guancewei = 2
                elif c_now in (5, 6, 7, 8):
                    if c_now_id in (28, 29, 30, 31):
                        self.guancewei = 1
                    else:
                        self.guancewei = 2
                else:
                    self.guancewei = 1
                result.append(c_now_id)

                if c_next:
                    if c_next in (1, 2, 3, 4):
                        result.extend([21, 20])
                    elif c_next in (5, 6, 7, 8):
                        pass  # 不需要额外操作
                    # 对于c_next in (9, 10, 11, 12)的情况，不需要额外操作

            else:
                rospy.logdebug(f"警告：位置{i}的值{c_now}超出有效范围")
                return result

        return result


def main():
    try:
        node = StateMachineNode()
        node.run_state_machine()
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")
    except Exception as e:
        rospy.logerr(f"程序运行失败: {e}")


if __name__ == '__main__':
    main()