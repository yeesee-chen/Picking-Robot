<<<<<<< HEAD
                     #B区
                    elif self.main_task == 1:
                        rospy.loginfo(f"B区 - 当前航点: {self.current_waypoint_id_}")
                        self.receive_class_ripeness = 1
                        self.receive_point = 1
                        if self.current_waypoint_id_ == 20:
                            if self.arrive == 1:
                                self.change_waypoint(19)
                        if self.current_waypoint_id_ == 19:
                            if self.catchable and self.fruit_class==self.b_chinese_string_array[7]:
                                self.catchable = 0
                                # 坐标抓取
                                self.voice_pub_logic()
                                self.robot_arm()
                                # 前往下一个航点

                            else:
                                # 复位
                                self.arm_pub.publish("动作组:0;")
                                rospy.sleep(2)

                            self.fruit_class=None
                            self.next_waypoint_flag = 15
                        if self.current_waypoint_id_ == 15:
                            if self.catchable and self.fruit_class==self.b_chinese_string_array[3]:
                                self.catchable = 0
                                # 坐标抓取
                                self.robot_arm()
                                # 前往下一个航点

                            else:
                                # 复位
                                self.arm_pub.publish("动作组:0;")
                                rospy.sleep(2)
                            self.next_waypoint_flag = 18
                            self.fruit_class = None
                        if self.current_waypoint_id_ == 18:
                            if self.catchable and self.fruit_class==self.b_chinese_string_array[6]:
                                self.catchable = 0
                                # 坐标抓取
                                self.robot_arm()
                                # 前往下一个航点

                            else:
                                # 复位
                                self.arm_pub.publish("动作组:0;")
                                rospy.sleep(2)
                                self.next_waypoint_flag = 14
                            self.fruit_class = None
                        if self.current_waypoint_id_ == 14:
                            if self.catchable and self.fruit_class==self.b_chinese_string_array[2]:
                                self.catchable = 0
                                # 坐标抓取
                                self.robot_arm()
                                # 前往下一个航点
                                self.next_waypoint_flag = 17
                            else:
                                # 复位
                                self.arm_pub.publish("动作组:0;")
                                rospy.sleep(2)
                                self.next_waypoint_flag = 17
                            self.fruit_class = None
                        if self.current_waypoint_id_ == 17:
                            if self.catchable and self.fruit_class==self.b_chinese_string_array[5]:
                                self.catchable = 0
                                # 坐标抓取
                                self.robot_arm()
                                # 前往下一个航点
                                self.next_waypoint_flag = 13
                            else:
                                # 复位
                                self.arm_pub.publish("动作组:0;")
                                rospy.sleep(2)
                                self.next_waypoint_flag = 13
                            self.fruit_class = None
                        if self.current_waypoint_id_ == 13:
                            if self.catchable and self.fruit_class==self.b_chinese_string_array[1]:
                                self.catchable = 0
                                # 坐标抓取
                                self.robot_arm()
                                # 前往下一个航点
                                self.next_waypoint_flag = 16
                            else:
                                # 复位
                                self.arm_pub.publish("动作组:0;")
                                rospy.sleep(2)
                                self.next_waypoint_flag = 16
                            self.fruit_class = None
                        if self.current_waypoint_id_ == 16:
                            if self.catchable and self.fruit_class==self.b_chinese_string_array[4]:
                                self.catchable = 0
                                # 坐标抓取
                                self.robot_arm()
                                # 前往下一个航点
                                self.next_waypoint_flag = 12
                            else:
                                # 复位
                                self.arm_pub.publish("动作组:0;")
                                rospy.sleep(2)
                                self.next_waypoint_flag = 12
                            self.fruit_class = None
                        if self.current_waypoint_id_ == 12:
                            if self.catchable and self.fruit_class==self.b_chinese_string_array[0]:
                                self.catchable = 0
                                # 坐标抓取
                                self.robot_arm()
                                # 前往下一个航点
                                self.next_waypoint_flag = 11
                            else:
                                # 复位
                                self.arm_pub.publish("动作组:0;")
                                rospy.sleep(2)
                                self.next_waypoint_flag = 11
                            self.fruit_class = None




=======
                    # b区
                    elif self.main_task == 1:
>>>>>>> 39e50cbb46c336c573d324c5ae54118c45ce09f7