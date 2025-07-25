PI
#include <plan_manage/ego_replan_fsm.h>

#define PI 3.1415926
#define yaw_error_max 5.0/180*PI
#define yaw_fanal_error_max 5.0/180*PI

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    //初始化状态变量
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);
    nh.param("fsm/w_adjust_", w_adjust, 1.0);

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* initialize main modules (初始化主要模块)*/
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
    dir = POSITIVE;
      goal_last << 0,0,0;                                       

    /* callback (定时器、订阅者、发布者创建)*/
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("/odom_map", 1, &EGOReplanFSM::odometryCallback, this);

    bspline_pub_ = nh.advertise<ego_planner::Bspline>("/planning/bspline", 10);
    data_disp_pub_ = nh.advertise<ego_planner::DataDisp>("/planning/data_display", 100);
    // cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/twd_velocity_controller/cmd_vel",100);
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel",100);
    adjust_cmd_pub_ = nh.advertise<std_msgs::UInt8>("/is_adjust_yaw",100);
    odom_adjust_pub_ = nh.advertise<nav_msgs::Odometry>("/odom_adjust",100);
    dir_pub = nh.advertise<std_msgs::UInt8>("/direction",100);
    stop_pub = nh.advertise<std_msgs::UInt8>("/emergency_stop",100);
    get_arrive_pub = nh.advertise<std_msgs::UInt8>("/weather_arrive",100);

    is_target_receive = false;
    //订阅目标点话题
    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
      waypoint_sub_ = nh.subscribe("/way_points", 1, &EGOReplanFSM::goal_callback, this);
      //waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &EGOReplanFSM::waypointCallback, this);
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      waypoint_sub_ = nh.subscribe("/waypoint", 1, &EGOReplanFSM::goal_callback, this);
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::planGlobalTrajbyGivenWps()
  {
    //参数中获取航点
    std::vector<Eigen::Vector3d> wps(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps[i](0) = waypoints_[i][0];
      wps[i](1) = waypoints_[i][1];
      wps[i](2) = waypoints_[i][2];

      end_pt_ = wps.back();
    }
    //调用planner_manager_的planGlobalTrajWaypoints方法规划全局轨迹
    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    //显示航点
    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      std::vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      // if (exec_state_ == WAIT_TARGET)
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      // else if (exec_state_ == EXEC_TRAJ)
      //   changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      ros::Duration(0.001).sleep();
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
      ros::Duration(0.001).sleep();
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  /**
   * @brief 处理接收到的航路点路径消息，根据消息内容进行全局轨迹规划
   * 
   * 该函数订阅航路点路径消息，当接收到有效消息时，触发全局轨迹规划。
   * 若规划成功，更新目标信息并根据有限状态机当前状态进行状态转换，同时显示全局轨迹；
   * 若规划失败，输出错误信息。
   * 
   * @param msg 接收到的航路点路径消息的常量指针
   */
  void EGOReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {
    // 检查路径消息中第一个位姿的 z 坐标是否小于 -0.1，若小于则认为无效，直接返回
    if (msg->poses[0].pose.position.z < -0.1)
      return;

    // 输出触发信息，并标记触发标志为 true
    cout << "Triggered!" << endl;
    trigger_ = true;
    // 记录初始点为当前里程计位置
    init_pt_ = odom_pos_;

    // 初始化规划成功标志为 false
    bool success = false;
    // 设置终点坐标，z 坐标固定为 1.0
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;
    // 调用规划管理器的 planGlobalTraj 方法进行全局轨迹规划
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // 显示目标点，颜色为青绿色，半径 0.3，编号 0
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      /*** 显示全局轨迹 ***/
      // 定义采样时间步长为 0.1 秒
      constexpr double step_size_t = 0.1;
      // 计算采样点数量
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      // 存储采样点的向量
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      // 遍历每个采样点，计算全局轨迹上对应时间点的位置
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      // 设置终点速度为零向量
      end_vel_.setZero();
      // 标记已获取目标点
      have_target_ = true;
      // 标记有新的目标点
      have_new_target_ = true;

      /*** FSM ***/
      // 若当前状态为等待目标状态，切换到生成新轨迹状态
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      // 若当前状态为执行轨迹状态，切换到重新规划轨迹状态
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // 显示全局轨迹，线宽 0.1，编号 0
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      // 若全局轨迹规划失败，输出错误信息
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

/**
 * @brief 处理接收到的目标点消息，根据消息内容进行全局轨迹规划
 * 
 * 该函数订阅目标点消息，当接收到有效消息时，触发全局轨迹规划。
 * 若规划成功，更新目标信息并根据有限状态机当前状态进行状态转换，同时显示全局轨迹；
 * 若规划失败，输出错误信息。
 * 
 * @param msg 接收到的目标点消息的常量指针
 */
void EGOReplanFSM::goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // 重置计数变量
    count = 0;
    // 重置重规划计数变量
    replan_count = 0;
    // 从消息中提取目标点的最终偏航角
    targetFinalYaw = tf::getYaw(msg->pose.orientation);

    // 创建一个无符号 8 位整型消息，用于表示是否到达目标点
    std_msgs::UInt8 arrive_cmd;
    // 设置消息数据为 0，表示未到达目标点
    arrive_cmd.data = 0;
    // 发布是否到达目标点的消息
    get_arrive_pub.publish(arrive_cmd);

    // 记录目标点的 x 坐标
    final_x = msg->pose.position.x;
    // 记录目标点的 y 坐标
    final_y = msg->pose.position.y;

    // 设置终点坐标，z 坐标使用当前里程计的 z 坐标
    end_pt_ << msg->pose.position.x, msg->pose.position.y, odom_pos_(2);

    // 标记触发标志为 true，表示接收到新的目标点
    //cout << "Triggered!" << endl;
    trigger_ = true;

    // 记录初始点为当前里程计位置
    init_pt_ = odom_pos_;

    // 初始化规划成功标志为 false
    bool success = false;

    // 记录上一个目标点为当前终点
    goal_last = end_pt_;

    // 调用规划管理器的 planGlobalTraj 方法进行全局轨迹规划
    success = planner_manager_->planGlobalTraj(odom_pos_,odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // 显示目标点，颜色为青绿色，半径 0.3，编号 0
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
        /*** 显示全局轨迹 ***/
        // 定义采样时间步长为 0.1 秒
        constexpr double step_size_t = 0.1;
        // 计算采样点数量
        int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
        // 存储采样点的向量
        vector<Eigen::Vector3d> gloabl_traj(i_end);
        // 遍历每个采样点，计算全局轨迹上对应时间点的位置
        for (int i = 0; i < i_end; i++)
        {
            gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
        }
        // 设置终点速度为零向量
        end_vel_.setZero();
        // 标记已获取目标点
        have_target_ = true;
        // 标记有新的目标点
        have_new_target_ = true;

        //goal is too close to current pose

        /*** 有限状态机处理 ***/
        // 若当前状态为等待目标状态，切换到生成新轨迹状态，并标记已接收目标点
        if (exec_state_ == WAIT_TARGET)
        {
            changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
            is_target_receive = true;
        }
        // 若当前状态为执行轨迹状态，切换到重新规划轨迹状态，并标记已接收目标点
        else if (exec_state_ == EXEC_TRAJ)
        {
            changeFSMExecState(REPLAN_TRAJ, "TRIG");
            is_target_receive=true;
        }

        // 显示全局轨迹，线宽 0.1，编号 0
        visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);

        // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
        //visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
        // 若全局轨迹规划失败，输出错误信息
        ROS_ERROR("Unable to generate global trajectory!");
    }
}


  /**
   * @brief 里程计消息回调函数，处理接收到的里程计消息
   * 
   * 该函数在接收到里程计消息时被调用，负责提取里程计的位置、速度、姿态信息，
   * 根据机器人运动方向调整偏航角，并发布调整后的里程计消息。
   * 
   * @param msg 接收到的里程计消息的常量指针
   */
  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    // 提取里程计消息中的位置信息，并存储到 odom_pos_ 向量中
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    // 提取里程计消息中的线速度信息，并存储到 odom_vel_ 向量中
    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    // 注释掉的代码，原本计划通过 estimateAcc 函数估计加速度
    //odom_acc_ = estimateAcc( msg );

    // 提取里程计消息中的四元数姿态信息，并存储到 odom_orient_ 中
    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    // 将 ROS 消息中的四元数转换为 TF 库中的四元数
    tf::quaternionMsgToTF(msg->pose.pose.orientation,quat);
    // 从 TF 四元数中提取滚转、俯仰和偏航角
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // 再次提取滚转、俯仰和偏航角，存储到 vehicleYaw 中，此处存在重复提取
    tf::Matrix3x3(quat).getRPY(roll, pitch, vehicleYaw);

    // 如果机器人运动方向为负方向
    if(dir==NEGATIVE)
    {
        // 根据偏航角的正负进行调整
        if(yaw>0)
        {
            // 偏航角大于 0 时，减去 PI
            yaw -= PI;
        }
        else if(yaw<0)
        {
            // 偏航角小于 0 时，加上 PI
            yaw += PI;
        }
    }
      nav_msgs::Odometry odom_adjust;
      geometry_msgs::Quaternion quat=tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
      odom_adjust = *msg;
      odom_adjust.pose.pose.orientation = quat;
      odom_adjust_pub_.publish(odom_adjust);

    // 创建一个新的里程计消息，用于存储调整后的信息
    nav_msgs::Odometry odom_adjust;
    // 根据调整后的滚转、俯仰和偏航角创建四元数
    geometry_msgs::Quaternion quat=tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
    // 将原始里程计消息内容复制到调整后的消息中
    odom_adjust = *msg;
    // 用调整后的四元数更新姿态信息
    odom_adjust.pose.pose.orientation = quat;
    // 发布调整后的里程计消息
    odom_adjust_pub_.publish(odom_adjust);

    // 标记已经接收到有效的里程计消息
    have_odom_ = true;
  }

  /**
   * @brief 改变有限状态机（FSM）的执行状态，并输出状态转换信息
   * 
   * 该函数用于更新有限状态机的当前执行状态，同时记录状态连续调用的次数，
   * 并输出状态转换的相关信息。
   * 
   * @param new_state 要转换到的新状态，类型为 FSM_EXEC_STATE 枚举
   * @param pos_call 触发状态转换的位置信息，用于日志输出
   */
  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {
    // 检查新状态是否与当前状态相同
    if (new_state == exec_state_)
      // 若相同，连续调用次数加 1
      continously_called_times_++;
    else
      // 若不同，重置连续调用次数为 1
      continously_called_times_ = 1;

    // 定义状态名称数组，用于将枚举值转换为对应的字符串
    static string state_str[8] = {"INIT", "WAIT_TARGET","ADJUST_POSE","GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "TURN_TO_TARGET"};
    // 记录当前状态的枚举值
    int pre_s = int(exec_state_);
    // 更新当前状态为新状态
    exec_state_ = new_state;
    // 输出状态转换信息，包含触发位置、原状态和新状态
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void EGOReplanFSM::checkYawError() {
      yaw_error = yaw_start-yaw;
      //first step : find the real yaw error
      if(abs(yaw_error)>yaw_error_max)
      {
          //if yaw error is larger than PI,it means the symbol between current yaw and target yaw is different
          if(abs(yaw_error)>PI)
          {
              //calculate the real yaw error with symbol
              yaw_error = yaw_error - yaw_error/abs(yaw_error)*2*PI;
              //if real yaw error larger than PI/2, change the direction of robot
              if(abs(yaw_error)>PI/2)
              {
                  changeDirection();

              }
          }
          else
          {
              cmd_vel.linear.x = 0;
              cmd_vel.angular.z =  yaw_error*3;

          }
      }
  }

  double EGOReplanFSM::calculateYawError(double yaw_cur,double yaw_target) {
      double error = yaw_target - yaw_cur;
      if(abs(error)>PI)
      {
          error = error - error/abs(error)*2*PI;
          return error;
      }
      else
      {
          return error;
      }
  }

  void EGOReplanFSM::changeDirection() {
      if(dir == POSITIVE)
      {
          dir = NEGATIVE;
      }else
      {
          dir = POSITIVE;
      }
      std_msgs::UInt8 dir_new;
      dir_new.data = dir;
      dir_pub.publish(dir_new);
  }



  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
    //返回连续调用次数和当前状态的pair
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET","ADJUST_POSE","GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "TURN_TO_TARGET"};
    //注释：初始化；等待航点发布；调整位姿；生成新轨迹；重新规划轨迹；执行轨迹；紧急停车；调整位置转向目标
    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  /**
   * @brief 有限状态机（FSM）执行回调函数，根据当前状态执行相应操作
   * 
   * 该函数由定时器定期触发，负责监控系统状态，根据有限状态机的当前状态
   * 执行不同的操作，如状态转换、轨迹规划、偏航角调整等，并发布数据显示消息。
   * 
   * @param e 定时器事件，包含定时器触发的时间信息
   */
  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    // 静态变量，用于计数回调函数的调用次数
    static int fsm_num = 0;
    fsm_num++;
    // 每调用 100 次，输出 FSM 当前状态和系统信息
    if (fsm_num == 100)
    {
      // 打印 FSM 当前执行状态
      printFSMExecState();
      // 若未收到里程计信息，输出提示信息
      if (!have_odom_)// 有里程计话题发布时 have_odom_ 为 true
        cout << "no odom." << endl;
      // 若未收到新的目标点或位姿，输出等待提示信息
      if (!trigger_) // 收到新的目标点或位姿时 trigger_ 为 true
        cout << "wait for goal." << endl;
      // 重置计数器
      fsm_num = 0;
    }

    // 根据有限状态机的当前执行状态进行不同处理
    switch (exec_state_)
    {
    case INIT:
    {
      // 若未收到里程计信息，直接返回
      if (!have_odom_)
      {
        return;
      }
      // 若未收到触发信号，直接返回
      if (!trigger_)
      {
        return;
      }
      // 转换到等待目标状态
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      // 若未获取到目标点，直接返回
      if (!have_target_)
      {
        // 原注释代码，可控制机器人旋转等待目标点
        // cmd_vel.linear.x = 0;
        // cmd_vel.angular.z = yaw_error/abs(yaw_error)*0.5;
        // cmd_pub_.publish(cmd_vel);
        return;
      }
      else
      {
        // 转换到生成新轨迹状态
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        // 原注释代码，可标记为未获取目标点
        // have_target_ = false;
      }
      break;
    }

    case ADJUST_POSE:
    {
      // 原注释代码，可在特定状态下调用紧急停止
      // if(last_state_!=WAIT_TARGET&&last_state_!=EMERGENCY_STOP)
      // {
      //     callEmergencyStop(odom_pos_);
      // }
      // 原注释代码，可在未获取目标点时进行状态转换
      // if (!have_target_)
      // {
        // 计算偏航角误差
        yaw_error = yaw_start - yaw;
        // 处理偏航角误差大于 PI 的情况
        if(abs(yaw_error) > PI)
        {
            yaw_error = yaw_error - yaw_error / abs(yaw_error) * 2 * PI;
        }
        // 静态变量，用于计数循环次数
        static int count = 0;
        // 每 100 次循环输出方向、当前偏航角和偏航角误差信息
        if(count % 100 == 0)
        {
            string directions[2] = {"POSITIVE", "NAGETIVE"};
            cout << "direction : " << directions[int(dir)] << endl;
            cout << "current yaw: " << yaw << endl;
            cout << "yaw error : " << yaw_error << endl;
            count = 0;
        }
        count += 1;
        // 若偏航角误差大于最大允许误差，调整机器人旋转
        if(abs(yaw_error) > yaw_error_max)
        {
            is_adjust_pose.data = 1;
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = yaw_error * 4;
            cmd_pub_.publish(cmd_vel);
            adjust_cmd_pub_.publish(is_adjust_pose);
        }
        else
        {
            is_adjust_pose.data = 0;
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            cmd_pub_.publish(cmd_vel);
            // 转换到执行轨迹状态
            changeFSMExecState(EXEC_TRAJ, "FSM");
            adjust_cmd_pub_.publish(is_adjust_pose);
            // 获取局部轨迹数据指针
            auto info = &planner_manager_->local_data_;
            // 记录轨迹开始时间
            info->start_time_ = ros::Time::now();
            // 发布 B 样条轨迹
            publishBspline();
        }
      // }
      // 原注释代码，可在获取目标点时进行状态转换
      // else
      // {
      //   changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      //   have_target_ = false;
      // }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      // 原注释代码，可在未获取目标点时进行相应处理
      // if (!have_target_)
      // {
        // 设置轨迹起始点为当前里程计位置
        start_pt_ = odom_pos_;
        // 设置轨迹起始速度为零向量
        start_vel_ << 0, 0, 0;
        // 设置轨迹起始加速度为零向量
        start_acc_.setZero();

        // 原注释代码，可计算起始偏航角
        // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
        // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
        // start_yaw_(1) = start_yaw_(2) = 0.0;

        // 标志位，用于决定是否使用随机多项式初始化
        bool flag_random_poly_init;
        // 若当前状态连续调用次数为 1，不使用随机多项式初始化
        if (timesOfConsecutiveStateCalls().first == 1)
          flag_random_poly_init = false;
        else
          flag_random_poly_init = true;

        // 调用回弹重规划函数
        bool success = callReboundReplan(true, flag_random_poly_init);
        if (success)
        {
            // 获取轨迹起始速度
            Eigen::Vector3d vel_start = planner_manager_->local_data_.velocity_traj_.evaluateDeBoor(0.1);
            // 计算起始偏航角
            yaw_start = atan2(vel_start(1), vel_start(0));
            cout << "yaw start : " << yaw_start << endl;
            // 计算偏航角误差
            yaw_error = yaw_start - yaw;

            // 处理偏航角误差大于 PI 的情况
            if(abs(yaw_error) > PI)
            {
                yaw_error = yaw_error - yaw_error / abs(yaw_error) * 2 * PI;
            }
            cout << "yaw error : " << yaw_error << endl;
            // 若偏航角误差大于 PI/2，调整方向
            if(abs(yaw_error) > PI / 2.0)
            {
                if(yaw > 0)
                {
                    yaw -= PI;
                }
                else if(yaw < 0)
                {
                    yaw += PI;
                }
                // 改变机器人运动方向
                changeDirection();
                // 重新计算偏航角误差
                yaw_error = yaw_start - yaw;
            }
            // 若偏航角误差大于最大允许误差，转换到调整位姿状态
            if(abs(yaw_error) > yaw_error_max)
            {
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z =  yaw_error * 3;
                changeFSMExecState(ADJUST_POSE, "TRIG");
                last_state_ = GEN_NEW_TRAJ;
                is_target_receive = false;
                return;
            }
            cout << "yaw error : " << yaw_error << endl;
            // 获取局部轨迹数据指针
            auto info = &planner_manager_->local_data_;
            // 记录轨迹开始时间
            info->start_time_ = ros::Time::now();
            // 发布 B 样条轨迹
            publishBspline();
            // 转换到执行轨迹状态
            changeFSMExecState(EXEC_TRAJ, "FSM");
            // 标记已逃离紧急状态
            flag_escape_emergency_ = true;
        }
        // 若重规划失败且重规划次数小于等于 5 次，继续尝试生成新轨迹
        else if(replan_count <= 5)
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          replan_count += 1;
        }
        else
        {
          // 计算当前位置与目标点的距离
          double dx = final_x - odom_pos_(0);
          double dy = final_y - odom_pos_(1);
          double dis = sqrt(dx * dx + dy * dy); 

          // 若距离小于等于 0.5，转换到转向目标状态
          if (dis <= 0.5)
          {
            changeFSMExecState(TURN_TO_TARGET, "FSM");
          }
          else
          {
            // 转换到等待目标状态，并标记为未获取目标点
            changeFSMExecState(WAIT_TARGET, "FSM"); 
            have_target_ = false;
          }
        }
      // }
      // 原注释代码，可在获取目标点时进行状态转换
      // else 
      // {
      //   changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      //   have_target_ = false;
      // }
      
      break;
    }

    case REPLAN_TRAJ:
    {
      // 原注释代码，可在未获取目标点时进行相应处理
      // if (!have_target_)
      // {
        // 计算当前位置与目标点的距离
        double dx = final_x - odom_pos_(0);
        double dy = final_y - odom_pos_(1);
        double dis = sqrt(dx * dx + dy * dy);
        // 若距离小于等于 0.5，转换到转向目标状态
        if (dis <= 0.5)
        {
          changeFSMExecState(TURN_TO_TARGET, "FSM");
        }
        // 尝试从当前轨迹进行重规划
        else if (planFromCurrentTraj())
        {
            // 获取轨迹起始速度
            Eigen::Vector3d vel_start = planner_manager_->local_data_.velocity_traj_.evaluateDeBoor(0.1);
            // 计算起始偏航角
            yaw_start = atan2(vel_start(1), vel_start(0));
            // 计算偏航角误差
            yaw_error = yaw_start - yaw;

            // 获取局部轨迹数据指针
            auto info = &planner_manager_->local_data_;
            // 记录轨迹开始时间
            info->start_time_ = ros::Time::now();
            // 发布停止命令
            std_msgs::UInt8 stop_cmd;
            stop_cmd.data = 0;
            stop_pub.publish(stop_cmd);
            // 发布 B 样条轨迹
            publishBspline();
            // 转换到执行轨迹状态
            changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        else
        {
            // 继续尝试重新规划轨迹
            changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      // }
      // 原注释代码，可在获取目标点时进行状态转换
      // else
      // {
      //   changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      //   have_target_ = false;
      // }
      // 原注释代码，可在获取目标点时进行状态转换
      // else
      // {
      //   changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      //   have_target_ = false;
      // }
      break;
    }

    case EXEC_TRAJ:
    {
      // 原注释代码，可在未获取目标点时进行相应处理
      // if (!have_target_)
      // {
          /* 判断是否需要重新规划 */
        // 获取局部轨迹数据指针
        LocalTrajData *info = &planner_manager_->local_data_;
        // 获取当前时间
        ros::Time time_now = ros::Time::now();
        // 计算当前轨迹执行时间
        double t_cur = (time_now - info->start_time_).toSec();
        // 确保当前时间不超过轨迹总时长
        t_cur = min(info->duration_, t_cur);

        // 获取当前位置
        Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

        /* && (end_pt_ - pos).norm() < 0.5 */
        // 若轨迹执行接近结束，转换到转向目标状态
        if (t_cur > info->duration_ - 0.1e-2)
        {
          have_target_ = false;

          changeFSMExecState(TURN_TO_TARGET, "FSM");
          return;
        }
        // 若接近目标点，不进行重新规划
        else if ((end_pt_ - pos).norm() < no_replan_thresh_)
        {
          // cout << "near end" << endl;
          return;
        }
        // 若接近起始点，不进行重新规划
        else if ((info->start_pos_ - pos).norm() < replan_thresh_)
        {
          // cout << "near start" << endl;
          return;
        }
        else
        {
          // 转换到重新规划轨迹状态
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      // }
      // 原注释代码，可在获取目标点时进行状态转换
      // else
      // {
      //   changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      //   have_target_ = false;
      // }
      break;
    }

    case EMERGENCY_STOP:
    {
      // 原注释代码，可在未获取目标点时进行相应处理
      // if (!have_target_)
      // {
        // 若标记为已逃离紧急状态，调用紧急停止函数
        if (flag_escape_emergency_) // 避免重复调用
        {
          callEmergencyStop(odom_pos_);
        }
        else
        {
          // 若机器人速度小于 0.1，转换到生成新轨迹状态
          if (odom_vel_.norm() < 0.1)
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }

        // 标记为未逃离紧急状态
        flag_escape_emergency_ = false;
      // }
      // 原注释代码，可在获取目标点时进行状态转换
      // else 
      // {
      //   changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      //   have_target_ = false;
      // }
      break;
    }

    case TURN_TO_TARGET:
    {
      // 原注释代码，可在未获取目标点时进行相应处理
      // if (!have_target_)
      // {
        // 计算当前位置与目标点的 x、y 方向差值
        double dx = final_x - odom_pos_(0);
        double dy = final_y - odom_pos_(1);
        // 计算朝向目标点的偏航角
        double pos_yaw_err = atan2(dy, dx);

        // 计算当前位置与目标点的距离
        double dis = sqrt(dx * dx + dy * dy);
        printf("dis:%f\n", dis);

        // 若距离大于等于 0.03 且计数器为 0
        if (dis >= 0.03 && count == 0)
        {
          // 计算朝向目标点的偏航角误差
          double yaw_error_1 = pos_yaw_err - yaw;
          // 处理偏航角误差大于 PI 的情况
          if(abs(yaw_error_1) > PI)
          {
            yaw_error_1 = yaw_error_1 - yaw_error_1 / abs(yaw_error_1) * 2 * PI;
          }
          // 若偏航角误差大于 PI/2，调整方向
          if(abs(yaw_error_1) > PI / 2.0)
          {
              if(yaw > 0)
              {
                  yaw -= PI;
              }
              else if(yaw < 0)
              {
                  yaw += PI;
              }
              // 改变机器人运动方向
              changeDirection();
              // 重新计算偏航角误差
              yaw_error_1 = pos_yaw_err - yaw;
          }
          printf("yaw_error_1:%f\n", yaw_error_1);
          // 若偏航角误差大于等于 10 度，仅调整机器人旋转
          if (abs(yaw_error_1) >= 10.0 / 180 * PI)
          {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = yaw_error_1 * 4;
            cmd_pub_.publish(cmd_vel);
            printf("cmd_vel.linear.x:%f\n", cmd_vel.linear.x);
          }
          else
          {
            // 根据机器人运动方向设置线速度
            if (dir == NEGATIVE)
            {
              cmd_vel.linear.x = -0.3;
              cmd_vel.angular.z = yaw_error_1 * 4;
              cmd_pub_.publish(cmd_vel);
            }
            else
            {
              cmd_vel.linear.x = 0.3;
              cmd_vel.angular.z = yaw_error_1 * 4;
              cmd_pub_.publish(cmd_vel);
            }
            printf("cmd_vel.linear.x:%f\n", cmd_vel.linear.x);
          }
        }
        else
        {
          // 标记计数器为 1
          count = 1;
          // 计算最终偏航角误差
          yaw_error = targetFinalYaw - vehicleYaw;
          // 处理偏航角误差大于 PI 的情况
          if(abs(yaw_error) > PI)
          {
            yaw_error = yaw_error - yaw_error / abs(yaw_error) * 2 * PI;
          }
          // 若偏航角误差大于最大允许最终误差，调整机器人旋转
          if(abs(yaw_error) > yaw_fanal_error_max)
          {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = yaw_error * 4;
            cmd_pub_.publish(cmd_vel);
          }
          else
          {
            // 转换到等待目标状态，并标记为未获取目标点
            changeFSMExecState(WAIT_TARGET, "FSM");
            have_target_ = false;
            // 发布到达目标点消息
            std_msgs::UInt8 arrive_cmd;
            arrive_cmd.data = 1;
            get_arrive_pub.publish(arrive_cmd);
          }
        }
      // }
      // 原注释代码，可在获取目标点时进行状态转换
      // else 
      // {
      //   changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      //   have_target_ = false;
      // }
      break;
    }

    }

    // 更新数据显示消息的时间戳
    data_disp_.header.stamp = ros::Time::now();
    // 发布数据显示消息
    data_disp_pub_.publish(data_disp_);
  
  /**
   * @brief 从当前轨迹开始进行重规划
   * 
   * 该函数从当前轨迹获取起始点、速度和加速度信息，计算偏航角误差，
   * 根据偏航角误差和目标接收状态调整方向和速度，然后尝试多次调用回弹重规划函数。
   * 如果多次重规划都失败，则返回 false，否则返回 true。
   * 
   * @return bool 重规划是否成功
   */
  bool EGOReplanFSM::planFromCurrentTraj()
  {
    // 获取局部轨迹数据指针
    LocalTrajData *info = &planner_manager_->local_data_;
    // 获取当前时间
    ros::Time time_now = ros::Time::now();
    // 计算当前时间距离轨迹起始时间的秒数
    double t_cur = (time_now - info->start_time_).toSec();

    // 注释掉的代码，用于输出速度轨迹的控制点
    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    // 获取当前时间点的位置作为重规划的起始点
    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    // 获取当前时间点的速度作为重规划的起始速度
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    // 获取当前时间点的加速度作为重规划的起始加速度
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    // 计算朝向目标点的偏航角
    yaw_start = atan2((end_pt_-odom_pos_)(1),(end_pt_-odom_pos_)(0));
    // 计算偏航角误差
    yaw_error = yaw_start - yaw;

    // 如果接收到新的目标点
    if(is_target_receive)
    {
      // 若偏航角误差的绝对值大于 PI，对偏航角误差进行修正
      if(abs(yaw_error)>PI)
      {
        yaw_error = yaw_error - yaw_error/abs(yaw_error)*2*PI;
      }
      // 若修正后的偏航角误差绝对值大于 PI/2
      if(abs(yaw_error)>PI/2.0)
      {
        // 根据当前偏航角的正负进行调整
        if(yaw>0)
        {
          yaw -= PI;
        }
        else if(yaw<0)
        {
          yaw += PI;
        }
        // 改变机器人的运动方向
        changeDirection();
        // 重新计算偏航角误差
        //yaw_error = - yaw_error/abs(yaw_error)*(PI-abs(yaw_error));
        yaw_error = yaw_start - yaw;
        // 反转起始速度的 x 和 y 分量，z 分量置为 0
        start_vel_ <<-start_vel_(0),-start_vel_(1),0;
        // 反转起始加速度的 x 和 y 分量，z 分量置为 0
        start_acc_ <<-start_acc_(0),-start_acc_(1),0;
        // 注释掉的代码，用于调用紧急停止函数
        //callEmergencyStop(odom_pos_);
        // 创建一个无符号 8 位整型消息，用于发布停止命令
        std_msgs::UInt8 stop_cmd;
        stop_cmd.data = 1;
        // 发布停止命令
        stop_pub.publish(stop_cmd);
      }
      // 标记为未接收到新的目标点
      is_target_receive = false;
    }
    // 第一步：计算偏航角误差

    // 第一次尝试回弹重规划，不使用多项式初始化，不使用随机多项式轨迹
    bool success = callReboundReplan(false, false);

    // 若第一次重规划失败
    if (!success)
    {
      // 第二次尝试回弹重规划，使用多项式初始化，不使用随机多项式轨迹
      success = callReboundReplan(true, false);
      // 注释掉的代码，用于改变有限状态机状态为执行轨迹
      //changeFSMExecState(EXEC_TRAJ, "FSM");
      // 若第二次重规划失败
      if (!success)
      {
        // 第三次尝试回弹重规划，使用多项式初始化，使用随机多项式轨迹
        success = callReboundReplan(true, true);
        // 若第三次重规划失败，返回 false
        {
          return false;
        }
      }
    }

    // 若任意一次重规划成功，返回 true
    return true;
  }

  /**
   * @brief 碰撞检测回调函数，定期检查当前轨迹是否存在碰撞风险
   * 
   * 该函数由定时器定期触发，检查当前执行的轨迹上是否存在障碍物。
   * 若检测到碰撞，会尝试重新规划轨迹；若重新规划失败，根据情况进入紧急停止或重新规划状态。
   * 
   * @param e 定时器事件，包含定时器触发的时间信息
   */
  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    // 获取局部轨迹数据指针
    LocalTrajData *info = &planner_manager_->local_data_;
    // 获取规划管理器中的栅格地图指针
    auto map = planner_manager_->grid_map_;

    // 若有限状态机处于等待目标状态，或者轨迹起始时间无效，则不进行碰撞检测，直接返回
    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- 检查轨迹是否碰撞 ---------- */
    // 定义时间步长，用于遍历轨迹
    constexpr double time_step = 0.01;
    // 计算当前时间距离轨迹起始时间的秒数
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    // 计算轨迹总时长的 2/3 时间点
    double t_2_3 = info->duration_ * 2 / 3;
    // 从当前时间开始，以 time_step 为步长遍历轨迹
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      // 若当前时间小于轨迹总时长的 2/3，且当前遍历时间超过 2/3 时间点，则停止检查
      if (t_cur < t_2_3 && t >= t_2_3) // 若 t_cur < t_2_3，仅检查轨迹的前 2/3 部分
        break;

      // 获取当前时间点轨迹上的三维位置
      Eigen::Vector3d pos_cur = info->position_traj_.evaluateDeBoorT(t);
      // 将三维位置转换为二维位置
      Eigen::Vector2d pos_cur2d;
      pos_cur2d << pos_cur(0),pos_cur(1);
      // 检查二维位置是否存在膨胀后的障碍物
      if (map->getInflateOccupancy2d(pos_cur2d))
      {
        // 尝试从当前轨迹进行重规划
        if (planFromCurrentTraj()) // 尝试重规划
        {
          // 重规划成功，切换到执行轨迹状态
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          // 若从发现碰撞到当前时间小于紧急时间阈值
          if (t - t_cur < emergency_time_) // 0.8s 的紧急时间
          {
            // 输出警告信息，进入紧急停止状态
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            // 未达到紧急时间阈值，进入重新规划状态
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  /**
   * @brief 调用回弹重规划函数进行轨迹重规划
   * 
   * 该函数首先获取局部目标点，设置起始点、速度、加速度和局部目标点的 z 坐标，
   * 然后调用规划管理器的回弹重规划函数进行轨迹规划。若规划成功，将显示最优轨迹。
   * 
   * @param flag_use_poly_init 是否使用多项式初始化，布尔类型
   * @param flag_randomPolyTraj 是否使用随机多项式轨迹，布尔类型
   * @return bool 规划是否成功，成功返回 true，失败返回 false
   */
  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    // 获取局部目标点
    getLocalTarget();

    // 设置起始点的 z 坐标为当前里程计的 z 坐标
    start_pt_(2) = odom_pos_(2);
    // 设置起始速度的 z 分量为 0
    start_vel_(2) = 0;
    // 设置起始加速度的 z 分量为 0
    start_acc_(2) = 0;
    // 设置局部目标点的 z 坐标为当前里程计的 z 坐标
    local_target_pt_(2) = odom_pos_(2);
    // 设置局部目标速度的 z 分量为 0
    local_target_vel_(2) = 0;

    // 调用规划管理器的回弹重规划函数进行轨迹规划
    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    // 标记为没有新的目标点
    have_new_target_ = false;

    // 输出最终规划是否成功的信息
    cout << "final_plan_success=" << plan_success << endl;

    // 若规划成功
    if (plan_success)
    {
      // 获取局部轨迹数据指针
      auto info = &planner_manager_->local_data_;
      // 注释掉的代码，原计划用于发布 B 样条轨迹
      //publishBspline();
      // 获取位置轨迹的控制点
      Eigen::MatrixXd control_points = info->position_traj_.get_control_points();
      // 将控制点的 z 坐标设置为当前里程计的 z 坐标
      for(int i=0;i<control_points.cols();i++) control_points.col(i)(2) = odom_pos_(2);
      // 显示最优轨迹
      visualization_->displayOptimalList(control_points, 0);
    }

    // 返回规划是否成功的结果
    return plan_success;
  }

  /**
   * @brief 调用紧急停止函数并发布紧急停止轨迹
   * 
   * 该函数调用规划管理器的紧急停止函数，让机器人在指定位置紧急停止。
   * 然后将紧急停止后的轨迹信息封装成 B 样条消息并发布出去。
   * 
   * @param stop_pos 机器人需要紧急停止的位置，三维向量表示
   * @return bool 总是返回 true，表示操作成功
   */
  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {
    // 调用规划管理器的紧急停止函数，传入停止位置
    planner_manager_->EmergencyStop(stop_pos);

    // 获取规划管理器的局部轨迹数据指针
    auto info = &planner_manager_->local_data_;

    /* publish traj */
    // 创建一个 B 样条消息对象，用于存储并发布紧急停止轨迹信息
    ego_planner::Bspline bspline;
    // 设置 B 样条的阶数为 3
    bspline.order = 3;
    // 设置 B 样条轨迹的起始时间
    bspline.start_time = info->start_time_;
    // 设置 B 样条轨迹的 ID
    bspline.traj_id = info->traj_id_;

    // 获取位置轨迹的控制点矩阵
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    // 为 B 样条消息的位置点数组预留空间
    bspline.pos_pts.reserve(pos_pts.cols());
    // 遍历位置轨迹的控制点矩阵，将每个控制点添加到 B 样条消息的位置点数组中
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      // 创建一个几何点对象
      geometry_msgs::Point pt;
      // 设置点的 x 坐标
      pt.x = pos_pts(0, i);
      // 设置点的 y 坐标
      pt.y = pos_pts(1, i);
      // 设置点的 z 坐标
      pt.z = pos_pts(2, i);
      // 将点添加到 B 样条消息的位置点数组中
      bspline.pos_pts.push_back(pt);
    }

    // 获取位置轨迹的节点向量
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    // 为 B 样条消息的节点数组预留空间
    bspline.knots.reserve(knots.rows());
    // 遍历位置轨迹的节点向量，将每个节点添加到 B 样条消息的节点数组中
    for (int i = 0; i < knots.rows(); ++i)
    {
      // 将节点添加到 B 样条消息的节点数组中
      bspline.knots.push_back(knots(i));
    }

    // 发布 B 样条消息
    bspline_pub_.publish(bspline);

    // 总是返回 true，表示操作成功
    return true;
  }

  void EGOReplanFSM::getLocalTarget()
  {
    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // todo
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        return;
      }
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
      // cout << "AA" << endl;
    }
  }

  void EGOReplanFSM::publishBspline() {

      auto info = &planner_manager_->local_data_;
      info->start_time_ = ros::Time::now();
      /* publish traj */
      ego_planner::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      //cout<<"optimal point : "<<endl<<pos_pts<<endl;
      bspline.pos_pts.reserve(pos_pts.cols());
      Eigen::Vector3d point_temp;
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
          geometry_msgs::Point pt;
          pt.x = pos_pts(0, i);
          pt.y = pos_pts(1, i);
          pt.z = odom_pos_(2);
          bspline.pos_pts.push_back(pt);
          point_temp<<pt.x,pt.x,pt.x;
          //cout<<"point : "<<point_temp<<endl;
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
          bspline.knots.push_back(knots(i));
      }

      bspline_pub_.publish(bspline);
  }

} // namespace ego_planner
   