#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <mutex>
#include <cmath>
#include <chrono>

class InteractiveWaypointPublisher {
private:
    ros::NodeHandle nh;
    ros::Publisher goal_pub;
    ros::Publisher path_pub;
    ros::Publisher id_pub;
    ros::Subscriber pose_sub;
    ros::Subscriber waypoint_id_sub;

    struct Waypoint {
        geometry_msgs::PoseStamped pose;
        double yaw;  // 直接存储角度（弧度）
    };

    std::vector<Waypoint> waypoints;
    std::vector<int> waypoint_ids;
    int current_target_id;
    bool running;
    std::mutex mtx;

    // 误差阈值配置
    struct Tolerance {
        double position = 0.05;    // 位置误差阈值(米)
        double yaw = 0.087;       // 约5度
    } tolerance;

    geometry_msgs::PoseStamped current_pose;
    double current_yaw;  // 当前朝向（弧度）
    bool position_reached = false;
    bool adjusting_yaw = false;
    bool completed = false;
    ros::Time completion_time;

    // 加载路径点文件
    void loadWaypoints() {
        std::string file_path;
        nh.param<std::string>("waypoint_file", file_path, "/home/ycs/autonomous_exploration_development_environment/src/local_planner/config/waypoint_file.txt");

        std::ifstream file(file_path.c_str());
        if (!file.is_open()) {
            ROS_ERROR("无法打开路径点文件: %s", file_path.c_str());
            return;
        }

        std::string line;
        while(std::getline(file, line)) {
            if(line.empty() || line[0] == '#') continue;

            Waypoint wp;
            int id;
            double x, y, z, global_yaw;

            if(sscanf(line.c_str(), "%d %lf %lf %lf %lf",
                     &id, &x, &y, &z, &global_yaw) >= 5) {
                wp.pose.header.frame_id = "map";
                wp.pose.pose.position.x = x;
                wp.pose.pose.position.y = y;
                wp.pose.pose.position.z = z;
                wp.yaw = global_yaw;

                // 设置默认朝向（不影响实际使用）
                wp.pose.pose.orientation.w = 1.0;

                waypoints.push_back(wp);
                waypoint_ids.push_back(id);
                ROS_INFO("加载路径点 ID: %d 位置 (%.2f, %.2f) 全局朝向 %.2f度",
                       id, x, y, global_yaw*180/M_PI);
            }
        }
        ROS_INFO("共加载 %lu 个路径点，位置容差 %.2f米，角度容差 %.2f度",
               waypoints.size(),
               tolerance.position,
               tolerance.yaw*180/M_PI);
    }

    double angleDifference(double a, double b) {
        double diff = a - b;
        while (diff > M_PI) diff -= 2.0 * M_PI;
        while (diff < -M_PI) diff += 2.0 * M_PI;
        return diff;
    }

    void checkWaypointReached() {
        if(current_target_id == -1 || completed) return;

        auto it = std::find(waypoint_ids.begin(), waypoint_ids.end(), current_target_id);
        if(it == waypoint_ids.end()) return;

        size_t index = std::distance(waypoint_ids.begin(), it);
        const auto& target = waypoints[index].pose.pose;
        const auto& current = current_pose.pose;
        double target_yaw = waypoints[index].yaw;

        // 位置误差
        double pos_error = sqrt(pow(target.position.x - current.position.x, 2) +
                              pow(target.position.y - current.position.y, 2));

        // 如果还没到达位置
        if(!position_reached) {
            if(pos_error <= tolerance.position) {
                position_reached = true;
                adjusting_yaw = true;
                ROS_INFO("已到达目标位置，开始调整朝向");
            } else {
                ROS_INFO_THROTTLE(1.0, "导航中... 位置误差: %.3fm", pos_error);
                return;
            }
        }

        // 角度误差计算
        double yaw_error = fabs(angleDifference(target_yaw, current_yaw));

        if(adjusting_yaw) {
            ROS_INFO_THROTTLE(1.0, "调整朝向中... 角度误差: %.1f° (目标%.1f° 当前%.1f°)",
                            yaw_error*180/M_PI, target_yaw*180/M_PI, current_yaw*180/M_PI);

            if(yaw_error <= tolerance.yaw) {
                ROS_INFO("成功到达路径点 %d (位置误差 %.2fm, 角度误差 %.1f°)",
                       current_target_id, pos_error, yaw_error*180/M_PI);
                adjusting_yaw = false;
                completed = true;
                completion_time = ros::Time::now();
            }
        }
    }

    // 用户输入线程
    void userInputThread() {
        while(running && ros::ok()) {
            std::cout << "\n可用路径点ID: ";
            for(auto id : waypoint_ids) {
                std::cout << id << " ";
            }
            std::cout << "\n输入目标ID (或输入'q'退出): ";

            std::string input;
            std::getline(std::cin, input);

            if(input == "q") {
                running = false;
                ros::shutdown();
                break;
            }

            try {
                int target_id = std::stoi(input);
                setTargetWaypoint(target_id);
            } catch(...) {
                ROS_WARN("无效输入! 请输入数字或'q'退出");
            }
        }
    }

    // 设置目标路径点
    void setTargetWaypoint(int target_id) {
        std::lock_guard<std::mutex> lock(mtx);

        auto it = std::find(waypoint_ids.begin(), waypoint_ids.end(), target_id);
        if(it != waypoint_ids.end()) {
            current_target_id = target_id;
            position_reached = false;
            adjusting_yaw = false;
            completed = false;
            size_t index = std::distance(waypoint_ids.begin(), it);
            ROS_INFO("选择路径点 ID %d 位置 (%.2f, %.2f) 朝向 %.2f度",
                   target_id,
                   waypoints[index].pose.pose.position.x,
                   waypoints[index].pose.pose.position.y,
                   waypoints[index].yaw*180/M_PI);
        } else {
            ROS_WARN("无效ID %d! 可用ID: ", target_id);
            for(auto id : waypoint_ids) std::cout << id << " ";
            std::cout << std::endl;
        }
    }

    // 话题回调函数
    void waypointIdCallback(const std_msgs::Int32::ConstPtr& msg) {
        ROS_INFO("收到导航请求，目标ID: %d", msg->data);
        setTargetWaypoint(msg->data);
    }

public:
    InteractiveWaypointPublisher() : current_target_id(-1), running(true), current_yaw(0.0) {
        nh.param("tolerance/position", tolerance.position, 0.05);
        nh.param("tolerance/yaw", tolerance.yaw, 0.087);

        goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/waypoint_and_pose", 10);
        path_pub = nh.advertise<nav_msgs::Path>("/waypoint_path", 10);
        id_pub = nh.advertise<std_msgs::Int32>("/current_waypoint_id", 10);
        pose_sub = nh.subscribe("/current_pose", 10,
            &InteractiveWaypointPublisher::poseCallback, this);
        waypoint_id_sub = nh.subscribe("/waypoint_i_d", 10,
            &InteractiveWaypointPublisher::waypointIdCallback, this);

        loadWaypoints();
        std::thread(&InteractiveWaypointPublisher::userInputThread, this).detach();
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose = *msg;

        // 从四元数中提取当前朝向（仅用于订阅）
        double x = msg->pose.orientation.x;
        double y = msg->pose.orientation.y;
        double z = msg->pose.orientation.z;
        double w = msg->pose.orientation.w;
        current_yaw = atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z));

        checkWaypointReached();
    }

    void publishWaypoint() {
        std::lock_guard<std::mutex> lock(mtx);

        // 如果已完成且超过2秒，停止发布
        if(completed && (ros::Time::now() - completion_time).toSec() > 2.0) {
            return;
        }

        if(current_target_id == -1 || completed) return;

        auto it = std::find(waypoint_ids.begin(), waypoint_ids.end(), current_target_id);
        if(it == waypoint_ids.end()) return;

        size_t index = std::distance(waypoint_ids.begin(), it);
        waypoints[index].pose.header.stamp = ros::Time::now();

        // 如果还在前往位置阶段，保持当前朝向
        if(!position_reached) {
            goal_pub.publish(waypoints[index].pose);
        }
        // 如果正在调整角度，发布完整位姿
        else {
            goal_pub.publish(waypoints[index].pose);
        }

        // 发布路径用于可视化
        nav_msgs::Path path;
        path.header = waypoints[index].pose.header;
        for(const auto& wp : waypoints) {
            path.poses.push_back(wp.pose);
        }
        path_pub.publish(path);

        // 发布当前ID
        std_msgs::Int32 id_msg;
        id_msg.data = current_target_id;
        id_pub.publish(id_msg);
    }

    void run() {
        ros::Rate rate(10);
        while(running && ros::ok()) {
            publishWaypoint();
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char**  argv) {
    ros::init(argc, argv, "interactive_waypoint_publisher");
    InteractiveWaypointPublisher publisher;
    publisher.run();
    return 0;
}