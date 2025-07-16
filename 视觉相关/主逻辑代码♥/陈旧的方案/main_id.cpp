#include <ros/ros.h>
#include <geometry_msgs/Point>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <serial/serial.h>
#include <string>
#include <vector>
#include <queue>
#include <map>
#include <algorithm>

class HarvestRobot {
private:
    ros::NodeHandle nh_;

    // 串口通信
    serial::Serial stm32_serial_;

    // 订阅和发布
    //订阅水果点位
    ros::Subscriber fruit_point_sub_;
    //定位水果状态
    ros::Subscriber fruit_class_sub_;
    ros::Publisher waypoint_id_pub_;
    ros::Publisher robot_arm_pub_;
    ros::Publisher ggwp_pub_;
    ros::Publisher robot_voice_pub_;
    ros::Publisher robot_voice_num_pub_;

    // 存储接收到的数据
    std::vector<std::string> first_type_msgs_;  // 第一种信息
    std::vector<std::string> second_type_chinese_;  // 第二种信息中文部分
    std::vector<int> second_type_numbers_;  // 第二种信息数字部分
    std::vector<std::pair<std::string, bool>> third_type_msgs_;  // 第三种信息
    std::vector<std::vector<int>> fourth_type_arrays_;  // 第四种信息数组部分
    std::vector<int> fourth_type_numbers_;  // 第四种信息数字部分

    // 当前状态和流程控制
    int current_waypoint_id_;
    bool is_navigating_;
    bool is_processing_arm_;
    bool is_subscribing_camera_;
    int process_a_step_;
    ros::Timer process_a_timer_;
    ros::Timer process_c_timer_;
    ros::Timer subscribe_timer_;

    // 水果名称映射
    std::map<std::string, std::string> fruit_name_map_;

    // 动作组十六进制代码映射
    std::map<int, std::string> arm_action_map_;

public:
    HarvestRobot() :
        current_waypoint_id_(1),
        is_navigating_(false),
        is_processing_arm_(false),
        is_subscribing_camera_(false),
        process_a_step_(0) {

        // 初始化串口
        try {
            stm32_serial_.setPort("/dev/ttyUSB0");
            stm32_serial_.setBaudrate(115200);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            stm32_serial_.setTimeout(timeout);
            stm32_serial_.open();
        } catch (serial::IOException& e) {
            ROS_ERROR_STREAM("Unable to open port ");
            return;
        }

        if(stm32_serial_.isOpen()) {
            ROS_INFO_STREAM("Serial Port initialized");
        } else {
            ROS_ERROR_STREAM("Serial Port failed to initialize");
        }

        // 初始化发布者和订阅者
        waypoint_id_pub_ = nh_.advertise<std_msgs::Int32>("/waypoint_i_d", 10);
        robot_arm_pub_ = nh_.advertise<std_msgs::String>("/robot_arm", 10);
        ggwp_pub_ = nh_.advertise<std_msgs::Int32>("/ggwp", 10);
        robot_voice_pub_ = nh_.advertise<std_msgs::String>("/robot_voice", 10);
        robot_voice_num_pub_ = nh_.advertise<std_msgs::String>("/robot_voice_num", 10);

        // 初始化水果名称映射
        initFruitNameMap();

        // 初始化动作组映射
        initArmActionMap();

        // 开始导航
        startNavigation();
    }

    ~HarvestRobot() {
        if(stm32_serial_.isOpen()) {
            stm32_serial_.close();
        }
    }
    //拼音与中文的映射
    void initFruitNameMap() {
        fruit_name_map_["pingguo"] = "苹果";
        fruit_name_map_["li"] = "梨子";
        fruit_name_map_["yangcong"] = "洋葱";
        fruit_name_map_["nangua"] = "南瓜";
        fruit_name_map_["xihongshi"] = "西红柿";
        fruit_name_map_["jiao"] = "辣椒";
    }

    void initArmActionMap() {
        // 这里添加所有动作组的十六进制代码
        arm_action_map_[101] = "0x01";
        arm_action_map_[201] = "0x02";
        // 添加更多动作组...
    }

    bool shouldSubscribeCamera(int waypoint_id) {
        // 第一部分、第五部分、第十部分、第十二部分需要订阅相机话题
        return (waypoint_id >= 1 && waypoint_id <= 8) ||   // A区
               (waypoint_id >= 12 && waypoint_id <= 19) || // B区
               (waypoint_id >= 24 && waypoint_id <= 35) || // C区
               (waypoint_id >= 37 && waypoint_id <= 45);   // D区
    }

    void updateCameraSubscription(bool enable) {
        if (enable && !is_subscribing_camera_ && shouldSubscribeCamera(current_waypoint_id_)) {
            fruit_status_sub_ = nh_.subscribe("/yolov5/fruit_status", 10, &HarvestRobot::fruitStatusCallback, this);
            mature_detection_sub_ = nh_.subscribe("/yolov5/mature_detection", 10, &HarvestRobot::matureDetectionCallback, this);
            is_subscribing_camera_ = true;
            ROS_INFO_STREAM("Subscribed to camera topics at waypoint " << current_waypoint_id_);
        }
        else if (!enable && is_subscribing_camera_) {
            fruit_status_sub_.shutdown();
            mature_detection_sub_.shutdown();
            is_subscribing_camera_ = false;
            ROS_INFO_STREAM("Unsubscribed from camera topics at waypoint " << current_waypoint_id_);
        }
    }

    void startNavigation() {
        if (!is_navigating_) {
            is_navigating_ = true;
            std_msgs::Int32 msg;
            msg.data = current_waypoint_id_;
            waypoint_id_pub_.publish(msg);
            ROS_INFO_STREAM("Started navigation to waypoint " << current_waypoint_id_);

            // 3秒内不会发送新的航点id
            ros::Duration(3.0).sleep();
        }
    }

    void processSerialData() {
        if(stm32_serial_.available()) {
            std::string serial_data = stm32_serial_.read(stm32_serial_.available());

            // 处理第一种信息
            if (serial_data.find("苹果") != std::string::npos ||
                serial_data.find("梨子") != std::string::npos) {
                processFirstTypeMessage(serial_data);
            }
            // 处理第二种信息
            else if (serial_data.find("洋葱") != std::string::npos ||
                     serial_data.find("南瓜") != std::string::npos ||
                     serial_data.find("西红柿") != std::string::npos ||
                     serial_data.find("辣椒") != std::string::npos) {
                processSecondTypeMessage(serial_data);
            }
        }
    }

    void processFirstTypeMessage(const std::string& data) {
        // 解析第一种信息并存储
        std::vector<std::string> fruits;
        size_t pos = 0;
        size_t prev = 0;
        while ((pos = data.find('\n', prev)) != std::string::npos) {
            fruits.push_back(data.substr(prev, pos - prev));
            prev = pos + 1;
        }
        fruits.push_back(data.substr(prev));

        first_type_msgs_ = fruits;
        ROS_INFO_STREAM("Received first type message with " << fruits.size() << " fruits");
    }

    void processSecondTypeMessage(const std::string& data) {
        // 解析第二种信息并存储
        size_t numbers_pos = data.find_last_of(' ');
        if (numbers_pos != std::string::npos) {
            // 处理中文部分
            std::string chinese_part = data.substr(0, numbers_pos);
            std::vector<std::string> chinese_items;
            size_t pos = 0;
            size_t prev = 0;
            while ((pos = chinese_part.find('\n', prev)) != std::string::npos) {
                chinese_items.push_back(chinese_part.substr(prev, pos - prev));
                prev = pos + 1;
            }
            chinese_items.push_back(chinese_part.substr(prev));
            second_type_chinese_ = chinese_items;

            // 处理数字部分
            std::string numbers_part = data.substr(numbers_pos + 1);
            std::vector<int> numbers;
            pos = 0;
            prev = 0;
            while ((pos = numbers_part.find(',', prev)) != std::string::npos) {
                numbers.push_back(std::stoi(numbers_part.substr(prev, pos - prev)) + 20);
                prev = pos + 1;
            }
            numbers.push_back(std::stoi(numbers_part.substr(prev)) + 20);
            second_type_numbers_ = numbers;

            ROS_INFO_STREAM("Received second type message with " << chinese_items.size()
                           << " chinese items and " << numbers.size() << " numbers");
        }
    }

    //类别的回调函数，将类别赋值给fruit_class
    void fruit_classCallback(const std_msgs::String::ConstPtr& msg)
        if (!is_subscribing_camera_) return;
        fruit_class = msg->data;
    //成熟度的回调函数，将成熟度赋值给fruit_ripeness
    void fruit_ripenessCallback(const std_msgs::Bool::ConstPtr& msg)
        if (!is_subscribing_camera_) return;
        fruit_ripeness = msg->data;


//
//    void fruitStatusCallback(const std_msgs::String::ConstPtr& msg) {
//        if (!is_subscribing_camera_) return;
//
//        // 处理第三种信息
//        // 订阅类别的话题并赋值为fruit_class
//        std::string fruit_class = msg->data;
//        // std::bool fruit_ripeness =
//
////        //截取换行符之前的字符串，为类别的拼音
////        size_t space_pos = data.find('/n');
////        if (space_pos != std::string::npos) {
////            std::string fruit_pinyin = data.substr(0, space_pos);
////            bool is_mature = (data.substr(space_pos + 1) == "true");
////            third_type_msgs_.emplace_back(fruit_pinyin, is_mature);
//
//            // 语音播报
//            if (fruit_name_map_.find(fruit_pinyin) != fruit_name_map_.end()) {
//                std_msgs::String voice_msg;
//                voice_msg.data = fruit_name_map_[fruit_pinyin];
//                robot_voice_pub_.publish(voice_msg);
//            }
//
//            ROS_INFO_STREAM("Received fruit status: " << fruit_pinyin << " " << (is_mature ? "mature" : "immature"));
//        }
//    }

    void matureDetectionCallback(const std_msgs::String::ConstPtr& msg) {
        if (!is_subscribing_camera_) return;

        // 处理第四种信息
        std::string data = msg->data;
        size_t last_space_pos = data.find_last_of(' ');
        if (last_space_pos != std::string::npos) {
            // 处理数组部分
            std::string array_part = data.substr(0, last_space_pos);
            std::vector<int> array_numbers;
            size_t pos = 0;
            size_t prev = 0;
            while ((pos = array_part.find(' ', prev)) != std::string::npos) {
                array_numbers.push_back(std::stoi(array_part.substr(prev, pos - prev)));
                prev = pos + 1;
            }
            array_numbers.push_back(std::stoi(array_part.substr(prev)));
            fourth_type_arrays_.push_back(array_numbers);

            // 处理数字部分
            int single_number = std::stoi(data.substr(last_space_pos + 1));
            fourth_type_numbers_.push_back(single_number);

            // 发送数字对应的十六进制代码
            std_msgs::String num_msg;
            std::stringstream ss;
            ss << "0x" << std::hex << single_number;
            num_msg.data = ss.str();
            robot_voice_num_pub_.publish(num_msg);

            ROS_INFO_STREAM("Received mature detection: array size " << array_numbers.size()
                           << ", number " << single_number);

            // 开始流程a
            if (!is_processing_arm_) {
                startProcessA();
            }
        }
    }

    void startProcessA() {
        is_processing_arm_ = true;
        process_a_step_ = 0;
        executeProcessAStep();
    }
    // 流程a：机器人到达点位->爪子抓取->回正->张开爪子
    // 流程a之后为回正并张开爪子的状态
    void executeProcessAStep() {
        if (!is_processing_arm_ || fourth_type_arrays_.empty()) return;

        std_msgs::String arm_msg;

        switch (process_a_step_) {
            case 0: {
                // 在发送第一个指令前关闭相机订阅
                updateCameraSubscription(false);

                // 发送第一个数组元素的十六进制代码
                if (!fourth_type_arrays_.front().empty()) {
                    int action_code = fourth_type_arrays_.front().front();
                    if (arm_action_map_.find(action_code) != arm_action_map_.end()) {
                        arm_msg.data = arm_action_map_[action_code];
                        robot_arm_pub_.publish(arm_msg);
                        ROS_INFO_STREAM("Process A step 0: sending arm action " << arm_msg.data);
                    }
                }
                break;
            }
            //动作代码全部要重新写
            case 1: {
                // 发送夹取动作
                arm_msg.data = "0x03"; // 假设0x03是夹取动作
                robot_arm_pub_.publish(arm_msg);
                ROS_INFO_STREAM("Process A step 1: sending grab action");
                break;
            }
            case 2: {
                // 发送回正动作
                arm_msg.data = "0x04"; // 假设0x04是回正动作
                robot_arm_pub_.publish(arm_msg);
                ROS_INFO_STREAM("Process A step 2: sending reset action");
                break;
            }
            case 3: {
                // 发送张开动作
                arm_msg.data = "0x05"; // 假设0x05是张开动作
                robot_arm_pub_.publish(arm_msg);
                ROS_INFO_STREAM("Process A step 3: sending release action");

                // 流程a完成，开始流程c
                startProcessC();
                break;
            }
        }

        process_a_step_++;
        if (process_a_step_ < 4) {
            process_a_timer_ = nh_.createTimer(ros::Duration(2.5), &HarvestRobot::processATimerCallback, this, true);
        } else {
            is_processing_arm_ = false;
        }
    }

    void processATimerCallback(const ros::TimerEvent&) {
        executeProcessAStep();
    }

    void startProcessC() {
        if (!fourth_type_numbers_.empty()) {
            // 对数字部分减1
            fourth_type_numbers_.front()--;
            ROS_INFO_STREAM("Process C: decremented number to " << fourth_type_numbers_.front());

            if (fourth_type_numbers_.front() <= 0) {
                // 数字减到0，可以发送下一个航点
                fourth_type_numbers_.erase(fourth_type_numbers_.begin());
                fourth_type_arrays_.erase(fourth_type_arrays_.begin());

                // 导航到下一个航点
                navigateToNextWaypoint();
            } else {
                // 数字未减到0，2.5秒后继续流程a
                process_c_timer_ = nh_.createTimer(ros::Duration(2.5), &HarvestRobot::processCTimerCallback, this, true);
            }
        }
    }

    void processCTimerCallback(const ros::TimerEvent&) {
        startProcessA();
    }

    void subscribeTimerCallback(const ros::TimerEvent&) {
        updateCameraSubscription(true);
        ROS_INFO_STREAM("Delayed camera subscription activated at waypoint " << current_waypoint_id_);
    }

    void navigateToNextWaypoint() {
        // 清除上一个航点的第四种消息
        fourth_type_arrays_.clear();
        fourth_type_numbers_.clear();

        // 关闭当前航点的相机订阅
        updateCameraSubscription(false);

        // 根据当前部分决定下一个航点
        if (current_waypoint_id_ >= 1 && current_waypoint_id_ <= 8) {
            // A区
            if (current_waypoint_id_ < 8) {
                current_waypoint_id_++;
            } else {
                current_waypoint_id_ = 9; // 下一个部分
            }
        }
        else if (current_waypoint_id_ == 9) {
            current_waypoint_id_ = 10; // 下一个部分
        }
        else if (current_waypoint_id_ == 10) {
            current_waypoint_id_ = 11; // 下一个部分
        }
        else if (current_waypoint_id_ >= 12 && current_waypoint_id_ <= 19) {
            // B区
            if (current_waypoint_id_ < 19) {
                current_waypoint_id_++;
            } else {
                current_waypoint_id_ = 20; // 下一个部分
            }
        }
        else if (current_waypoint_id_ == 20) {
            current_waypoint_id_ = 21; // 下一个部分
        }
        else if (current_waypoint_id_ == 21) {
            current_waypoint_id_ = 22; // 下一个部分
        }
        else if (current_waypoint_id_ == 22) {
            current_waypoint_id_ = 23; // 下一个部分
        }
        else if (current_waypoint_id_ >= 24 && current_waypoint_id_ <= 35) {
            // C区 - 根据第二种消息的数字部分导航
            if (!second_type_numbers_.empty()) {
                // 移除已经导航的点位
                second_type_chinese_.erase(second_type_chinese_.begin());
                second_type_numbers_.erase(second_type_numbers_.begin());

                if (!second_type_numbers_.empty()) {
                    current_waypoint_id_ = second_type_numbers_.front();
                } else {
                    current_waypoint_id_ = 36; // 下一个部分
                }
            } else {
                current_waypoint_id_ = 36; // 下一个部分
            }
        }
        else if (current_waypoint_id_ == 36) {
            current_waypoint_id_ = 37; // 下一个部分
        }
        else if (current_waypoint_id_ >= 37 && current_waypoint_id_ <= 45) {
            // D区
            if (current_waypoint_id_ < 45) {
                current_waypoint_id_++;
            } else {
                current_waypoint_id_ = 46; // 最后一个部分
            }
        }
        else if (current_waypoint_id_ == 46) {
            ROS_INFO_STREAM("Navigation completed!");
            return;
        }

        // 开始导航到新航点
        startNavigation();

        // 执行流程b
        executeProcessB();
    }
    //

    void executeProcessB() {
        // 等待1秒确保到达当前航点
        ros::Duration(1.0).sleep();

        // 发送观测位消息
        std_msgs::Int32 ggwp_msg;

        // 判断当前航点类型
        if (current_waypoint_id_ == 16 || current_waypoint_id_ == 17 || current_waypoint_id_ == 18 ||
            current_waypoint_id_ == 19 || current_waypoint_id_ == 37 || current_waypoint_id_ == 39 ||
            current_waypoint_id_ == 41 || current_waypoint_id_ == 43 || current_waypoint_id_ == 44 ||
            current_waypoint_id_ == 45) {
            // 左边果树
            ggwp_msg.data = 4;
        }
        else if (current_waypoint_id_ == 12 || current_waypoint_id_ == 13 || current_waypoint_id_ == 14 ||
                 current_waypoint_id_ == 15 || current_waypoint_id_ == 38 || current_waypoint_id_ == 40 ||
                 current_waypoint_id_ == 42) {
            // 右边果树
            ggwp_msg.data = 3;
        }
        else if (current_waypoint_id_ == 1 || current_waypoint_id_ == 3 || current_waypoint_id_ == 5 ||
                 current_waypoint_id_ == 7 || current_waypoint_id_ == 24 || current_waypoint_id_ == 25 ||
                 current_waypoint_id_ == 26 || current_waypoint_id_ == 27 || current_waypoint_id_ == 32 ||
                 current_waypoint_id_ == 33 || current_waypoint_id_ == 34 || current_waypoint_id_ == 35) {
            // 左边蔬菜
            ggwp_msg.data = 1;
        }
        else if (current_waypoint_id_ == 2 || current_waypoint_id_ == 4 || current_waypoint_id_ == 6 ||
                 current_waypoint_id_ == 8 || current_waypoint_id_ == 28 || current_waypoint_id_ == 29 ||
                 current_waypoint_id_ == 30 || current_waypoint_id_ == 31) {
            // 右边蔬菜
            ggwp_msg.data = 2;
        }
        else {
            // 其他航点不需要发送观测位消息
            return;
        }

        ggwp_pub_.publish(ggwp_msg);
        ROS_INFO_STREAM("Process B: sent observation position " << ggwp_msg.data);

        // 3秒后订阅相机话题
        if (shouldSubscribeCamera(current_waypoint_id_)) {
            subscribe_timer_ = nh_.createTimer(ros::Duration(3.0), &HarvestRobot::subscribeTimerCallback, this, true);
        }
    }

    void run() {
        ros::Rate rate(10); // 10Hz

        while (ros::ok()) {
            // 处理串口数据
            processSerialData();

            // 处理ROS回调
            ros::spinOnce();

            rate.sleep();
        }
    }
};

int main(int argc, char**  argv) {
    ros::init(argc, argv, "harvest_robot_upper_computer");

    HarvestRobot robot;
    robot.run();

    return 0;
}