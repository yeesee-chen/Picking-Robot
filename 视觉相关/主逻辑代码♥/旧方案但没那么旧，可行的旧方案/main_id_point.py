#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String, Bool, Int32, Int8
import threading
import re
import time


class MainProcessingNode:
    """
    航点规划：按照顺序规划航点
    串口消息发布代码：送一系列动作组观测位以及需要播报的消息等到话题/arm_voice
        1.发送一些语音播报信息等
        2.发送机械臂需要的动作信息，包括一些动作组（固定的观测位）和坐标（相机识别之后的）坐标
        3.接受二维码信息/qr_arm_message、接受机械臂是否运动结束信息（self.catch_over）
    接受视觉代码并处理：
        1.接受类别(/fruit_class_ripeness)、坐标信息(fruit_point--相机坐标系--)
        2.将相机坐标系坐标转化为机械臂基坐标系坐标---李负责
        3.只有当接收指针(self.receive_class_ripeness,self.receive_point)之后，才进行回调，否则不接收
    """

    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('fruit_processing_node', anonymous=True)
        rospy.loginfo("水果处理节点启动中...")
        # 一些映射的词典
        # 英文类别与语音id的映射
        self.fruit_class_to_voice_id = {
            'pumpkin': 13,
            'pepper': 14,
            'tomato': 15,
            'onion': 16,
            'apple': 11,
            'pear': 12
        }
        # 中文类别与英文类别的映射
        self.fruit_chinese_to_english = {
            '苹果': 'apple',
            '梨子': 'pear',
            '南瓜': 'pumpkin',
            '西红柿': 'tomato',
            '辣椒': 'pepper',
            '洋葱': 'onion'
        }

        # 数据储存器
        self.b_chinese_string_array = []
        self.c_chinese_string_array = []

        # 消息缓存
        self.pub_position = Point()
        self.next_waypoint_flag = 0
        self.fruit_point = None
        self.fruit_class_ripeness = None
        self.fruit_class = None
        self.fruit_ripeness = None
        self.b_qr = None
        self.c_qr = None
        self.number_array = None
        self.arrive = None
        self.current_waypoint_id_ = 0
        self.class_id = None
        self.catchable = False
        self.fruit_count = 0
        self.receive_class_ripeness = 0
        self.receive_point = 0
        self.receive_qr = 0
        self.catch_over = 0
        self.current_region = None
        self.direction = None
        # 设置一个指针，指示现在处于哪个任务期间，0为a区任务，1为b区任务，2为c区任务，默认为0
        self.main_task = 0

        # C区相关变量

        # 新加入状态追踪
        self.last_published_waypoint = -1
        # 航点是否发布
        self.waypoint_published = False

        # 串口消息缓存
        self.serial_message = None

        # 上一次发布的机械臂指令，用于避免重复发布
        self.last_arm_command = None

        # 线程锁，确保消息处理的线程安全
        self.lock = threading.Lock()

        # 订阅者
        self.class_ripeness_sub = rospy.Subscriber('/fruit_class_ripeness', String, self.class_ripeness_callback)
        self.point_sub = rospy.Subscriber('/fruit_point', Point, self.point_callback)
        # 航点是否停止标志位
        self.weather_arrive_sub = rospy.Subscriber('/weather_arrive', Int8, self.weather_arrive_callback)
        # 订阅二维码信息的标志位
        self.qr_sub = rospy.Subscriber('/qr_arm_message', String, self.qr_callback)

        # 发布者
        self.arm_pub = rospy.Publisher('/arm_voice', queue_size=10)
        # self.ggwp_pub = rospy.Publisher('/ggwp', Int32, queue_size=10)
        self.waypoint_i_d_pub = rospy.Publisher('/waypoint_i_d', Int32, queue_size=1)

        # 等待发布者建立连接
        rospy.sleep(0.5)
        rospy.loginfo("主逻辑处理节点初始化完成")

    def initial_voice_broadcast(self):
        """
        初始语音播报：队伍信息
        """
        self.arm_pub.publish("语音:17;")
        rospy.loginfo("发布初始语音播报")

    def point_callback(self, msg):
        """
        水果坐标回调函数
        """
        with self.lock:
            if self.arrive == 1 and self.receive_point ==1:
                self.fruit_point = msg
                rospy.logdebug(f"接收到水果坐标: x={msg.x}, y={msg.y}, z={msg.z}")
                self.receive_point = 0

    def class_ripeness_callback(self, msg):
        """
        水果类别与成熟度回调函数-只有在类别与成熟度指针为1时才进行发布
        """
        with self.lock:
            if self.arrive == 1 and self.receive_class_ripeness == 1:
                self.fruit_class_ripeness = msg.data
                parts = self.fruit_class_ripeness.split('_')
                self.fruit_class = parts[0]
                self.fruit_ripeness = int(parts[1])
                rospy.loginfo(f"接收到水果类别: {self.fruit_class}")
                self.receive_class_ripeness = 0

    def qr_callback(self, msg):
        """
        二维码消息回调函数
        将b区二维码和c区二维码分开储存了，扫描到之后进行储存，不需要2次前往区域扫码
        """
        with self.lock:
            if self.arrive == 1 and self.receive_qr == 1:
                self.c_qr = msg.data.strip()
                rospy.loginfo("接收到二维码消息！")
                self.process_qr_data()
                self.receive_qr = 0
            elif self.arrive == 1 and self.receive_qr == 2:
                self.b_qr = msg.data.strip()
                rospy.loginfo("接收到二维码消息！")
                self.process_qr_data()
                self.receive_qr = 0

    def weather_arrive_callback(self, msg):
        """
        导航是否停止标志位回调函数
        """
        with self.lock:
            self.arrive = msg.data  # 修改：移除了错误的括号
            rospy.logdebug("接收到导航是否结束标志位")

    def process_qr_data(self):
        """
        二维码消息解析函数
        """
        try:
            # 按行分割消息
            lines = [line.strip() for line in self.c_qr.split('\n') if line.strip()]

            rospy.loginfo(f"解析到 {len(lines)} 行数据: {lines}")

            # 判断消息类型
            if self.receive_qr == 1:
                self.process_vegetable_with_numbers_type(lines)
            elif self.receive_qr == 2:
                self.process_fruit_only_type(lines)

        except Exception as e:
            rospy.logerr(f"处理二维码消息时发生错误: {e}")

    def process_vegetable_with_numbers_type(self, lines):
        """
        处理蔬菜+数字类型
        """
        try:
            rospy.loginfo("识别为蔬菜+数字类型")

            if len(lines) >= 9:
                vegetable_lines = lines[:8]
                number_line: String = lines[-1]
                self.number_array = number_line
            else:
                rospy.logwarn("数据行数不足")
                return

            self.c_chinese_string_array = vegetable_lines[:]
            rospy.loginfo(f"存储蔬菜名称数组: {self.c_chinese_string_array}")
            rospy.loginfo(f"存储数字字符串: {self.number_array}")

        except Exception as e:
            rospy.logerr(f"处理蔬菜+数字类型失败: {e}")

    def process_fruit_only_type(self, lines):
        """
        处理纯水果类型
        """
        try:
            rospy.loginfo("识别为纯水果类型")

            fruit_english_array = []
            for fruit in lines:
                if fruit in self.fruit_chinese_to_english:
                    english = self.fruit_chinese_to_english[fruit]
                    fruit_english_array.append(english)
                else:
                    rospy.logwarn(f"未知水果名称: {fruit}")
                    fruit_english_array.append(fruit)

            self.b_chinese_string_array = fruit_english_array[:]
            rospy.loginfo(f"存储水果英文数组: {self.b_chinese_string_array}")

        except Exception as e:
            rospy.logerr(f"处理纯水果类型失败: {e}")

    def process_ripeness_data(self):
        """
        水果成熟度的处理与语音编号的转化
        """
        if self.receive_class_ripeness ==1:
            ripeness_id = 19 if self.fruit_ripeness else 18
            return ripeness_id

    def process_fruit_data(self):
        """
        水果类别处理与语音编号的转化
        """
        if self.receive_class_ripeness == 1:
            try:
                if self.fruit_class in self.fruit_class_to_voice_id:
                    self.class_id = self.fruit_class_to_voice_id[self.fruit_class]
                else:
                    rospy.logwarn(f"未知水果/蔬菜类别: {self.fruit_class}")
                    self.class_id = 0

            except Exception as e:
                rospy.logerr(f"处理水果数据时发生错误: {e}")

            return self.class_id

    def ggwp_pub(self):
        """
        发布ggwp
        """
        ggwp_msg = Int32()
        ggwp_msg.data = self.generate_ggwp_pub()
        self.arm_pub.publish(ggwp_msg)

    def generate_ggwp_pub(self):
        """
        生成视觉所需要的左右观测位问题
        """
        a_left = { 1, 3, 5, 7}
        b_left = {16, 17, 18, 19}
        b_right = {12, 13, 14, 15}
        c = {24,25,26,27,28,29,30,31,32,33,34,35}
        if self.current_waypoint_id_ in a_left:
            if self.next_waypoint_flag == 1 :
                return 2
            else:
                return 1
        elif self.current_waypoint_id_ in b_left:
            return 3
        elif self.current_waypoint_id_ in b_right:
            return 4
        elif self.current_waypoint_id_ in c:
            return 5
        else:
            return 0

    def voice_pub_logic(self):
        """
        语音播报的逻辑
        """
        class_id = self.process_fruit_data()
        ripeness_id = self.process_ripeness_data()
        if self.arrive != 1:
            return
        else:
            ggwp = self.generate_ggwp_pub()
            # a区播报成熟度
            if ggwp in (1,2):
                self.arm_pub.publish(f"语音:{ripeness_id}")
            # b区播报类别
            elif ggwp in (3,4):
                self.arm_pub.publish(f"语音:{class_id}")
            # c区播报类别和成熟度
            elif ggwp == 5 :
                self.arm_pub.publish(f"语音:{class_id}")
                rospy.sleep(1)
                self.arm_pub.publish(f"语音:{ripeness_id}")

    def robot_arm(self):
        """
        机械臂夹取运动组
        """
        # 修改坐标的顺序并发布坐标
        r = self.fruit_point.x * 100
        z = self.fruit_point.z * 100
        phi = self.fruit_point.y
        self.arm_pub.publish(f"机械臂:{r,z,phi};")

        if self.catch_over == 1 :
            # 发布爪子夹紧动作
            self.arm_pub.publish("爪子:0;")
            rospy.sleep(2)
            # 发布回位动作组
            self.arm_pub.publish("动作组:0;")
            rospy.sleep(4)
            # 发布爪子张开动作
            self.arm_pub.publish("爪子:1;")
            self.fruit_count += 1
            rospy.sleep(2)
            # 计数
            self.fruit_count += 1
            # 抓取指针复位
            self.catch_over = 0

    def replan_c_task(self):
        """
        根据获得的c区数组，重新规划顺序
        """
        test_strings = self.number_array
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
                        result.append(c_now_id)
                    else:  # 在左边
                        if c_now in (1, 2, 3, 4):
                            c_now_id = c_now + 23
                            result.append(c_now_id)
                        elif c_now in (5, 6, 7, 8):
                            c_now_id = c_now + 27
                            result.append(c_now_id)
                            if c_next in (1, 2, 3, 4):
                                result.extend([21, 20])

                elif c_next and c_next in (9, 10, 11, 12):
                    c_now_id = c_now + 23
                    result.extend([c_now_id, 20, 21])

                else:
                    # 处理序列末尾的情况
                    if c_next is None:
                        c_now_id = c_now + 23
                        result.append(c_now_id)
                    else:
                        rospy.loginfo(f"未处理的情况：位置{i}，当前值{c_now}")

            # 处理位置9-12的情况
            elif c_now in (9, 10, 11, 12):
                c_now_id = c_now + 23
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

    def publish_waypoint_id(self):
        """
        发布当前航点ID - only once
        """
        # 只在航点变化或首次发布时发布航点ID
        if self.current_waypoint_id_ != self.last_published_waypoint or not self.waypoint_published:
            self.waypoint_i_d_pub.publish(self.current_waypoint_id_)
            self.last_published_waypoint = self.current_waypoint_id_
            self.waypoint_published = True
            rospy.loginfo(f"发布新航点ID: {self.current_waypoint_id_}")

    def change_waypoint(self, new_waypoint_id):
        """
        改变航点并重置发布标志
        """
        if new_waypoint_id != self.current_waypoint_id_:
            self.current_waypoint_id_ = new_waypoint_id
            self.waypoint_published = False
            rospy.loginfo(f"航点更新为: {self.current_waypoint_id_}")

    def run(self):
        """
        主运行循环
        """
        # 初始化
        rospy.loginfo("水果处理节点开始运行...")
        self.next_waypoint_flag = 0
        self.arm_pub.publish("观测位:0;")
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            # 发布当前航点ID
            self.publish_waypoint_id()
            # 发布视觉需要的/ggwp消息
            self.ggwp_pub()
            if self.main_task == 0:
                # 初始语音播报
                if self.current_waypoint_id_ == 0:
                    # 播报并初始化动作
                    self.initial_voice_broadcast()
                    rospy.sleep(5)
                    self.arm_pub.publish("动作组:0;")
                    # self.fruit_class = None
                    # self.fruit_ripeness = None
                    self.change_waypoint(1)

                # A区 (航点1-8)
                elif 1 <= self.current_waypoint_id_ <= 8:
                    rospy.loginfo(f"A区 - 当前航点: {self.current_waypoint_id_}")
                    rospy.loginfo(f"A区 - next_waypoint_flag : {self.next_waypoint_flag}")
                    if self.arrive == 1:
                        rospy.sleep(1)
                        if self.next_waypoint_flag == 0:
                            self.arm_pub.publish("观测位:1;")
                        elif self.next_waypoint_flag == 1:
                            self.arm_pub.publish("观测位:2;")
                        rospy.sleep(2)
                        # 此时开始订阅话题得到point，class，ripeness
                        self.receive_class_ripeness = 1
                        self.receive_point = 1
                        # 保护机制
                        count = 0
                        while self.receive_class_ripeness == 1 and self.receive_point == 1 and self.next_waypoint_flag < 2:
                            count = count + 1
                            rospy.sleep(0.1)
                            if count == 100:
                                rospy.loginfo("无法识别！无法接收水果信息！--触发保护机制--")
                                self.receive_class_ripeness = 0
                                self.receive_point = 0
                                self.fruit_ripeness = None
                                self.fruit_class = None
                                self.fruit_ripeness = None
                                break
                            pass
                        # 播报语音 需要播报成熟度
                        self.voice_pub_logic()
                        self.fruit_class_ripeness = None
                        self.fruit_class = None
                        self.fruit_ripeness = None
                        self.fruit_point = None
                        if self.next_waypoint_flag == 2:
                            if self.current_waypoint_id_ < 8:
                                # 只导航到1，3，5，7，为a区的四个点
                                self.current_waypoint_id_ = self.current_waypoint_id_ + 2
                                self.change_waypoint(self.current_waypoint_id_)
                                self.next_waypoint_flag = 0
                            else:
                                # 过了4轮了去9号点
                                self.change_waypoint(9)
                                # 任务驱动前往c区
                                self.main_task = 2
                                self.next_waypoint_flag = 0
                        # 左边
                        elif self.next_waypoint_flag == 0:
                            if self.catchable:
                                self.catchable = 0
                                # 坐标抓取
                                self.robot_arm()
                                # 前往下一个航点
                                self.next_waypoint_flag = 1
                            else:
                                # 复位
                                self.arm_pub.publish("动作组:0;")
                                rospy.sleep(2)
                                self.next_waypoint_flag = 1
                        # 右边
                        elif self.next_waypoint_flag == 1:
                            # 开始动作组
                            if self.catchable:
                                self.catchable = 0
                                # 坐标抓取
                                self.robot_arm()
                                # 前往下一个航点
                                self.next_waypoint_flag = 2
                            else:
                                self.arm_pub.publish("动作组:0;")
                                rospy.sleep(2)
                                self.next_waypoint_flag = 2

                    # c区
                    if self.main_task == 2 :
                        # a区结束，前往b区扫码处扫码储存，前往c区扫码处扫码储存
                        if self.current_waypoint_id_ == 9:
                            if self.arrive == 1:
                                self.change_waypoint(11)
                        elif self.current_waypoint_id_ == 11:
                            if self.arrive == 1:
                                # 储存到self.b_chinese_string_array
                                self.receive_qr = 2
                                self.change_waypoint(20)
                        # 在c区任务期间，20，21只会进行一次
                        elif self.current_waypoint_id_ == 20 and len(self.number_array) == 0:
                            if self.arrive == 1:
                                self.change_waypoint(21)
                        # 到达c区扫码处并进行任务规划，
                        elif self.current_waypoint_id_ == 21 and len(self.number_array) == 0:
                            if self.arrive == 1:
                                # 储存到
                                # self.c_chinese_string_array
                                # self.number_array
                                self.receive_qr = 1
                                # 此时self.number_array != 0 了，所以当之后运行到20，21，不会进入这个循环
                                c_task = self.replan_c_task()
                                num = len(c_task)
                                for i in range(num):



            rate.sleep()

def main():
    """
    主函数
    """
    try:
        node = MainProcessingNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")
    except Exception as e:
        rospy.logerr(f"程序运行失败: {e}")


if __name__ == '__main__':
    main()
