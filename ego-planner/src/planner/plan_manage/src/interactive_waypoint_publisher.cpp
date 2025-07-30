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
    
    struct Waypoint {
        geometry_msgs::PoseStamped pose;
        double yaw;
    };
    
    std::vector<Waypoint> waypoints;
    std::vector<int> waypoint_ids;
    int current_target_id;
    bool running;
    std::mutex mtx;
    
    geometry_msgs::PoseStamped current_pose;
    double current_yaw;

    // 将yaw角度转换为四元数
    void setQuaternionFromYaw(geometry_msgs::PoseStamped& pose, double yaw) {
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = sin(yaw / 2.0);
        pose.pose.orientation.w = cos(yaw / 2.0);
    }
    
    void loadWaypoints() {
        std::string file_path;
        nh.param<std::string>("waypoint_file", file_path, "/home/boli/ego-planner/src/planner/plan_manage/config/waypoint_file.txt");
        
        std::ifstream file(file_path);
        if (!file.is_open()) {
            ROS_ERROR("无法打开路径点文件: %s", file_path.c_str());
            return;
        }

        std::string line;
        while(std::getline(file, line)) {
            if(line.empty() || line[0] == '#') continue;
            
            Waypoint wp;
            int id;
            double x, y, z, yaw;
            
            if(sscanf(line.c_str(), "%d %lf %lf %lf %lf", &id, &x, &y, &z, &yaw) >= 5) {
                wp.pose.header.frame_id = "map";
                wp.pose.pose.position.x = x;
                wp.pose.pose.position.y = y;
                wp.pose.pose.position.z = z;
                wp.yaw = yaw;
                
                // 确保正确设置四元数
                setQuaternionFromYaw(wp.pose, yaw);
                
                waypoints.push_back(wp);
                waypoint_ids.push_back(id);
                ROS_INFO("加载路径点 ID: %d 位置 (%.2f, %.2f) 朝向 %.2f度", 
                       id, x, y, yaw*180/M_PI);
            }
        }
        ROS_INFO("共加载 %lu 个路径点", waypoints.size());
    }

    void publishWaypointOnce(int target_id) {
        std::lock_guard<std::mutex> lock(mtx);
        
        auto it = std::find(waypoint_ids.begin(), waypoint_ids.end(), target_id);
        if(it == waypoint_ids.end()) {
            ROS_WARN("无效航点ID: %d", target_id);
            return;
        }
        
        size_t index = std::distance(waypoint_ids.begin(), it);
        waypoints[index].pose.header.stamp = ros::Time::now();
        
        // 调试输出：打印即将发布的消息内容
        ROS_INFO("发布消息内容 - 位置: (%.2f, %.2f, %.2f) 朝向: (%.2f, %.2f, %.2f, %.2f)",
               waypoints[index].pose.pose.position.x,
               waypoints[index].pose.pose.position.y,
               waypoints[index].pose.pose.position.z,
               waypoints[index].pose.pose.orientation.x,
               waypoints[index].pose.pose.orientation.y,
               waypoints[index].pose.pose.orientation.z,
               waypoints[index].pose.pose.orientation.w);
        
        // 发布目标位姿
        goal_pub.publish(waypoints[index].pose);
        
        // 发布路径可视化
        nav_msgs::Path path;
        path.header = waypoints[index].pose.header;
        for(const auto& wp : waypoints) {
            path.poses.push_back(wp.pose);
        }
        path_pub.publish(path);
        
        // 发布当前ID
        std_msgs::Int32 id_msg;
        id_msg.data = target_id;
        id_pub.publish(id_msg);
    }

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
                current_target_id = target_id;
                
                auto it = std::find(waypoint_ids.begin(), waypoint_ids.end(), target_id);
                if(it != waypoint_ids.end()) {
                    size_t index = std::distance(waypoint_ids.begin(), it);
                    ROS_INFO("选择路径点 ID %d 位置 (%.2f, %.2f) 朝向 %.2f度", 
                           target_id, 
                           waypoints[index].pose.pose.position.x,
                           waypoints[index].pose.pose.position.y,
                           waypoints[index].yaw*180/M_PI);
                    
                    publishWaypointOnce(target_id);
                } else {
                    ROS_WARN("无效ID %d! 可用ID: ", target_id);
                    for(auto id : waypoint_ids) std::cout << id << " ";
                    std::cout << std::endl;
                }
            } catch(...) {
                ROS_WARN("无效输入! 请输入数字或'q'退出");
            }
        }
    }

public:
    InteractiveWaypointPublisher() : current_target_id(-1), running(true), current_yaw(0.0) {
        goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/waypoint", 10);
        path_pub = nh.advertise<nav_msgs::Path>("/waypoint_path", 10);
        id_pub = nh.advertise<std_msgs::Int32>("/current_waypoint_id", 10);
        pose_sub = nh.subscribe("/current_pose", 10, 
            &InteractiveWaypointPublisher::poseCallback, this);
        
        loadWaypoints();
        std::thread(&InteractiveWaypointPublisher::userInputThread, this).detach();
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose = *msg;
        double x = msg->pose.orientation.x;
        double y = msg->pose.orientation.y;
        double z = msg->pose.orientation.z;
        double w = msg->pose.orientation.w;
        current_yaw = atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z));
    }

    void run() {
        ros::Rate rate(10);
        while(running && ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "interactive_waypoint_publisher");
    InteractiveWaypointPublisher publisher;
    publisher.run();
    return 0;
}
