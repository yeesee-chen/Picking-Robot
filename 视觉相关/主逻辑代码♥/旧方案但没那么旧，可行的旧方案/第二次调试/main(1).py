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
        self.qr_data = ""
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
        self.b_guancewei = 0

        # C区特定
        self.c_task_list = []
        self.c_current_index = 0
        self.c_obs_list = [] # 观测位模式

        # 线程锁
        self.data_lock = threading.Lock()

    def init_ros_communication(self):
        """初始化ROS通信"""
        # 订阅者
        self.class_ripeness_sub = rospy.Subscriber('/fruit_class_ripeness', String, self.vision_class_callback)
        self.point_sub = rospy.Subscriber('/fruit_point', Point, self.vision_point_callback)
        self.arrive_sub = rospy.Subscriber('/weather_arrive', Int8, self.arrival_callback)
        self.qr_sub = rospy.Subscriber('/qr_message', String, self.qr_callback)

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
        self.arrival_timeout = 5.0  # 导航到达超时
        self.action_timeout = 10.0  # 动作执行超时

        # 状态时间戳
        self.state_start_time = time.time()
        self.id_pub_time = time.time()

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
            self.qr_data = qr_data
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
            # 这里是看有几轮了
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
                self.arm_pub.publish("动作组:0;")
                rospy.sleep(1)
                self.area_a_state = AreaAState.MOVE_TO_NEXT

    def handle_a_grab_fruit(self):
        """A区抓取果实"""
        if self.execute_grab_action():
            if self.current_observation_side == 0:
                self.current_observation_side = 1
                self.area_a_state = AreaAState.OBSERVE_RIGHT
            else:
                self.arm_pub.publish("动作组:0;")
                rospy.sleep(1)
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
        self.process_b_qr_data(self.qr_data)
        if self.has_arrived and len(self.b_qr_data) > 0:
            self.area_c_state = AreaCState.GOTO_C_QR

    def handle_c_goto_c_qr(self):
        """前往C区二维码"""
        self.set_waypoint(21)
        self.area_c_state = AreaCState.SCAN_C_QR

    def handle_c_scan_c_qr(self):
        """扫描C区二维码"""
        self.process_c_qr_data(self.qr_data)
        if self.has_arrived and len(self.c_qr_data) > 0:
            self.area_c_state = AreaCState.PLAN_TASKS

    def handle_c_plan_tasks(self):
        """规划C区任务"""
        self.c_task_list, self.c_obs_list = self.replan_c_task(self.c_number_sequence)
        self.c_current_index = 0
        if len(self.c_task_list) > 0:
            self.area_c_state = AreaCState.NAVIGATE_TO_TARGET
        else:
            self.area_c_state = AreaCState.COMPLETED

    def handle_c_navigate(self):
        # 前往第n个航点
        self.set_waypoint(self.c_task_list[0])
        self.c_current_index = 0
        if self.has_arrived:
            self.area_c_state = AreaCState.EXECUTE_GRAB

    def handle_c_execute_grab(self):
        # 获取当前任务的观测位模式
        obs_mode = self.c_obs_list
        self.task_state = TaskState.SETTING_OBSERVATION
        # 执行通用观测+抓取流程
        if obs_mode[0] != 0:
            success = self.execute_observation_task(obs_mode[0])  # 右侧观测位
            if success:
                if self.should_grab_fruit():
                    self.execute_grab_action()
                else:
                    self.arm_pub.publish("动作组:0;")
                    rospy.sleep(1)
        else:
            self.arm_pub.publish("动作组:0;")
            rospy.sleep(1)
        self.area_c_state = AreaCState.MOVE_TO_NEXT_TARGET
        self.c_current_index = 1

    def handle_c_move_to_next(self):
        self.c_task_list = self.c_task_list[self.c_current_index:]
        self.c_obs_list = self.c_obs_list[self.c_current_index:]
        if len(self.c_task_list) > 0:
            self.area_c_state = AreaCState.NAVIGATE_TO_TARGET
        else:
            self.area_c_state = AreaCState.DUMP_FRUITS

    def handle_c_dump_fruits(self):
        self.set_waypoint(36)
        if self.has_arrived:
            self.arm_pub.publish("动作组:n;") # dump fruit cmd
            self.area_c_state = AreaCState.RETURN_TO_START

    def handle_c_return(self):
        self.set_waypoint(21)
        if self.has_arrived:
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

    def handle_b_navigate(self):
        """前往当前 B 区航点"""
        if self.b_current_index == 0 and self.current_waypoint_id == 20:
            # 第一次进入 B 区，跳到 19
            self.set_waypoint(19)
            if self.has_arrived:
                self.b_current_index = 0
                self.area_b_state = AreaBState.SCAN_AND_GRAB
                return

        if self.has_arrived:
            self.area_b_state = AreaBState.SCAN_AND_GRAB

    def handle_b_scan_and_grab(self):
        """在当前航点处理抓取"""
        idx = self.b_current_index
        wp = self.b_waypoint_list[idx]
        expected = self.b_qr_data[self.b_qr_indices[idx]]

        self.task_state = TaskState.SETTING_OBSERVATION
        self.b_guancewei = self.generate_b_observation(wp)
        if self.fruit_class == expected:
            success = self.execute_observation_task(wp)
            if success:
                if self.should_grab_fruit():
                    z = self.fruit_point.z
                    if z<=17:
                        self.arm_pub.publish("机械臂:10,0,90;")
                    else:
                        pass
                    self.execute_grab_action()
                else:
                    self.arm_pub.publish("动作组:0;")
                    rospy.sleep(1)
        else:
            self.arm_pub.publish("动作组:0;")
            rospy.sleep(1)

        self.area_b_state = AreaBState.MOVE_TO_NEXT

    def handle_b_move_to_next(self):
        """移动到下一条目"""
        self.b_current_index += 1
        #列表逻辑
        if self.b_current_index >= 7:
            self.area_b_state = AreaBState.COMPLETED
        else:
            self.set_waypoint(self.b_waypoint_list[self.b_current_index])
            self.area_b_state = AreaBState.NAVIGATE_TO_POINT

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
            self.id_pub_time = time.time()

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
        navigate_timeout = current_time - self.id_pub_time > self.arrival_timeout and self.has_arrived == 0
        if current_time - self.state_start_time > 60:  # 总体超时保护
            rospy.logwarn("状态执行超时，可能需要人工干预")
        elif navigate_timeout: # 导航超时保护
            self.has_arrived = 1
            rospy.logwarn("导航执行超时，手工设置航点标志位为1！")

    def generate_b_observation(self, waypoint):
        if waypoint in {16, 17, 18, 19}:  # 左侧
            return 3
        elif waypoint in {12, 13, 14, 15}:  # 右侧
            return 4
        return 0

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
        if self.system_state == SystemState.AREA_A:
            if self.area_a_state == AreaAState.OBSERVE_LEFT:
                return 1
            elif self.area_a_state == AreaAState.OBSERVE_RIGHT:
                return 2
            else:
                return 0
        elif self.system_state == SystemState.AREA_B:
            if self.area_b_state == AreaBState.SCAN_AND_GRAB:
                return self.b_guancewei
            else:
                return 0
        elif self.system_state == SystemState.AREA_C:
            if self.area_c_state == AreaCState.EXECUTE_GRAB:
                ggwp = self.c_obs_list[0]
                if ggwp != 0:
                    return ggwp
                else:
                    return 0
            else:
                return 0
        return 0

    def replan_c_task(self, position_str):
        """
        将位置编号字符串转换为对应的点位顺序和观察状态

        参数:
        position_str: 位置编号字符串，如 "1,6,10,3,2,5,7,8"

        返回:
        tuple: (点位顺序列表, 观察状态列表)
        观察状态说明:
        - 0: 目标点位为20,21时
        - 1: 目标位置为1-4时，或目标位置为5-8且点位在32-35之间时
        - 2: 目标位置为9-12时，或目标位置为5-8且点位在24-27之间时
        """
        # 解析输入字符串
        positions = [int(x.strip()) for x in position_str.split(',')]

        # 基本映射规则
        basic_mapping = {
            1: 24, 2: 25, 3: 26, 4: 27,
            9: 32, 10: 33, 11: 34, 12: 35
        }

        # 模糊位置的两种选择
        ambiguous_mapping = {
            5: (24, 32),
            6: (25, 33),
            7: (26, 34),
            8: (27, 35)
        }

        result = []
        obs_result = []
        last_target = None

        def get_current_obs(position, point):
            """根据位置和点位计算观察状态"""
            if point in [20, 21]:
                return 0
            elif position in [1, 2, 3, 4]:
                return 1
            elif position in [9, 10, 11, 12]:
                return 2
            elif position in [5, 6, 7, 8]:
                if 24 <= point <= 27:
                    return 2
                elif 32 <= point <= 35:
                    return 1
            return 0  # 默认值

        # 如果第一个位置在1-4中，先加20
        if positions[0] in [1, 2, 3, 4]:
            result.append(20)
            obs_result.append(0)  # 点位20对应观察状态0

        for i, pos in enumerate(positions):
            current_target = None

            # 处理基本映射
            if pos in basic_mapping:
                current_target = basic_mapping[pos]

            # 处理模糊位置
            elif pos in ambiguous_mapping:
                option1, option2 = ambiguous_mapping[pos]

                if last_target is None:
                    # 如果是第一个位置，默认选择第一组
                    current_target = option1
                elif 24 <= last_target <= 27:
                    # 上一个在24-27组，选择24-27组的点位
                    current_target = option1
                elif 32 <= last_target <= 35:
                    # 上一个在32-35组，选择32-35组的点位
                    current_target = option2
                else:
                    # 其他情况默认选择第一组
                    current_target = option1

            # 检查是否需要添加中间点位
            if last_target is not None and current_target is not None:
                # 从24-27前往32-35
                if 24 <= last_target <= 27 and 32 <= current_target <= 35:
                    result.extend([20, 21])
                    obs_result.extend([0, 0])  # 中间点位都是观察状态0
                # 从32-35前往24-27
                elif 32 <= last_target <= 35 and 24 <= current_target <= 27:
                    result.extend([21, 20])
                    obs_result.extend([0, 0])  # 中间点位都是观察状态0

            # 添加当前目标点位和对应的观察状态
            if current_target is not None:
                result.append(current_target)
                obs_result.append(get_current_obs(pos, current_target))
                last_target = current_target

        return result, obs_result


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