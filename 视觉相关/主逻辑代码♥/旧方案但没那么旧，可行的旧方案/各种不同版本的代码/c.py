def handle_c_plan_tasks(self):
    """规划C区任务（修正版）"""
    # 规划任务（现在返回三个列表）
    self.c_task_list, self.c_obs_list, self.c_fruit_list = self.replan_c_task(self.c_number_sequence)

    # 重置当前索引
    self.c_current_index = 0

    if len(self.c_task_list) > 0:
        self.area_c_state = AreaCState.NAVIGATE_TO_TARGET
    else:
        self.area_c_state = AreaCState.COMPLETED


def handle_c_execute_grab(self):
    """C区执行抓取（修正版）"""
    # 获取当前任务的观测位模式和期望水果
    obs_mode = self.c_obs_list[0] if self.c_obs_list else 0
    expected_fruit = self.c_fruit_list[0] if self.c_fruit_list else None

    # 如果是中间点位，直接跳过
    if obs_mode == 0:
        rospy.loginfo("C区：中间点位，跳过观测")
        self.arm_pub.publish("观测位:0;")
        rospy.sleep(1)
        self.task_state = TaskState.WAITING_FOR_ARRIVAL
        self.area_c_state = AreaCState.MOVE_TO_NEXT_TARGET
        return

    # 执行C区专用的观测+抓取流程
    success = self.execute_c_observation_and_grab(obs_mode, expected_fruit)

    if success:
        self.area_c_state = AreaCState.MOVE_TO_NEXT_TARGET


def execute_c_observation_and_grab(self, observation_pos, expected_fruit):
    """C区专用的观测+抓取流程（带期望水果参数）"""
    # ... 前面的逻辑保持不变 ...

    elif self.task_state == TaskState.PROCESSING_DATA:
    # 播报果实信息
    self.broadcast_fruit_info()
    self.task_state = TaskState.EXECUTING_ACTION

elif self.task_state == TaskState.EXECUTING_ACTION:
# 判断是否需要抓取：水果匹配且成熟
if expected_fruit and self.fruit_class == expected_fruit:
    rospy.loginfo(f"C区：水果匹配成功 {self.fruit_class} == {expected_fruit}")
    if self.should_grab_fruit():
        rospy.loginfo("C区：水果匹配且成熟，开始抓取")
        self.execute_grab_action()
    else:
        rospy.loginfo("C区：水果匹配但未成熟，不抓取")
        self.arm_pub.publish("观测位:0;")
        rospy.sleep(1)
else:
    rospy.loginfo(f"C区：水果不匹配 {self.fruit_class} != {expected_fruit}，不抓取")
    self.arm_pub.publish("观测位:0;")
    rospy.sleep(1)

self.task_state = TaskState.COMPLETED
return True


# ... 其他逻辑保持不变 ...

def handle_c_move_to_next(self):
    """C区移动到下一个目标（修正版）"""
    # 移除已完成的任务
    if self.c_task_list:
        self.c_task_list = self.c_task_list[1:]
    if self.c_obs_list:
        self.c_obs_list = self.c_obs_list[1:]
    if self.c_fruit_list:  # 新增：移除期望水果
        self.c_fruit_list = self.c_fruit_list[1:]

    # 重置任务状态
    self.task_state = TaskState.WAITING_FOR_ARRIVAL

    # 检查是否还有任务
    if self.c_task_list:
        self.area_c_state = AreaCState.NAVIGATE_TO_TARGET
    else:
        rospy.loginfo("C区：所有抓取任务完成，准备倾倒")
        self.area_c_state = AreaCState.DUMP_FRUITS