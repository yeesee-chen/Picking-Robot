#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

#define KEY_EMERGENCY  'e'  // 'e'键
#define KEY_RESUME     's'  // 's'键
#define KEY_QUIT       0x1B // ESC键

int getch()
{
    struct termios oldt, newt;
    int ch;
    
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // 禁用行缓冲和回显
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    
    ch = getchar();
    
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_emergency");
    ros::NodeHandle nh;
    ros::Publisher stop_pub = nh.advertise<std_msgs::UInt8>("/emergency_stop", 10, true); // 添加latch
    
    // 设置正确的提示信息
    ROS_INFO("=========================================");
    ROS_INFO("Keyboard Emergency Controller");
    ROS_INFO("  Press 'e' - Emergency stop");
    ROS_INFO("  Press 's' - Resume operation");
    ROS_INFO("  Press ESC - Exit program");
    ROS_INFO("=========================================");
    
    // 初始发布一次停止信号
    std_msgs::UInt8 init_msg;
    init_msg.data = 0;
    stop_pub.publish(init_msg);
    
    while (ros::ok())
    {

        int c = getch(); // 获取按键
        
        if (c == KEY_EMERGENCY) {
            std_msgs::UInt8 msg;
            msg.data = 1;
            stop_pub.publish(msg);
            ROS_ERROR("EMERGENCY STOP TRIGGERED!");
        } 
        else if (c == KEY_RESUME) {
            std_msgs::UInt8 msg;
            msg.data = 0;
            stop_pub.publish(msg);
            ROS_WARN("OPERATION RESUMED");
        } 
        else if (c == KEY_QUIT) {
            ROS_INFO("Exiting keyboard controller...");
            break;
        }
        else {
            ROS_INFO("Pressed key: 0x%02X ('%c') - Not a command key", c, c);
        }
        
        // 不需要频繁spinOnce，按键时已经处理
    }
    
    // 退出前发送恢复信号
    std_msgs::UInt8 exit_msg;
    exit_msg.data = 0;
    stop_pub.publish(exit_msg);
    
    return 0;
}
