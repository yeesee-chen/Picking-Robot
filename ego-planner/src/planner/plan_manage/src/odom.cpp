#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <float.h>

const double PI = 3.1415926;

// 控制参数
double lookAheadDis = 0.5;
double yawRateGain = 5.0;
double maxYawRate = 1.0;
double maxForwardSpeed = 0.8;     // 默认值设为0.8
double maxBackwardSpeed = -0.4;   // 默认值设为-0.4
double stopDisThre = 0.05;        // 默认值设为0.05
double yawTolerance = 0.087;      // 默认5度的弧度值
double slowDownFactor = 0.5;
double backwardAngleThre = 30.0 * PI/180;

// 车辆状态
double vehicleX = 0, vehicleY = 0, vehicleYaw = 0;
double goalX = 0, goalY = 0, targetFinalYaw = 0;
bool hasGoal = false;
bool positionReached = false;
bool isDrivingBackward = false;
double minDistanceToGoal = 0;

// 速度命令
geometry_msgs::Twist cmd_vel;

// 状态枚举
enum State { 
    IDLE,
    ADJUST_POSE,
    MOVING_TO_POSITION, // 移动到目标位置（前进或倒车）
    ROTATING_TO_YAW     // 在目标位置旋转到目标朝向
};
State currentState = IDLE;

void odomHandler(const nav_msgs::Odometry::ConstPtr& odomIn)
{
    vehicleX = odomIn->pose.pose.position.x;
    vehicleY = odomIn->pose.pose.position.y;
    
    tf::Quaternion q(
        odomIn->pose.pose.orientation.x,
        odomIn->pose.pose.orientation.y,
        odomIn->pose.pose.orientation.z,
        odomIn->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, vehicleYaw);
}

void goalHandler(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goalX = msg->pose.position.x;
    goalY = msg->pose.position.y;
    targetFinalYaw = tf::getYaw(msg->pose.orientation);
    
    hasGoal = true;
    positionReached = false;
    isDrivingBackward = false;
    minDistanceToGoal = DBL_MAX;
    currentState = ADJUST_POSE;
    
    double dx = goalX - vehicleX;
    double dy = goalY - vehicleY;
    double initialDist = sqrt(dx*dx + dy*dy);
    minDistanceToGoal = initialDist;
}

void adjust_pose()
{
    double dx = goalX - vehicleX;
    double dy = goalY - vehicleY;
    
    // 计算目标点相对于车辆的方向角
    double targetDir = atan2(dy, dx);
    double yawError = targetDir - vehicleYaw;

    if(abs(yawError)>PI)
    {
        yawError = yawError - yawError/abs(yawError)*2*PI;
    }

    if(abs(yawError)>PI/2.0)
    {
        if(vehicleYaw>0)
        {
            vehicleYaw -= PI;
        }else if(vehicleYaw<0)
        {
            vehicleYaw += PI;
        }
        //yaw_error = - yaw_error/abs(yaw_error)*(PI-abs(yaw_error));
        isDrivingBackward = true;
        yawError = targetDir - vehicleYaw;
    }
    else
    {
        isDrivingBackward = false;
    }
    if(abs(yawError)>yawTolerance/180*PI)
    {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = yawError/abs(yawError);
        currentState = ADJUST_POSE;
        return;
    }
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    currentState = MOVING_TO_POSITION;
    return;
}

void moveToPosition()
{
    double dx = goalX - vehicleX;
    double dy = goalY - vehicleY;
    double distance = sqrt(dx*dx + dy*dy);
    
    // // 更新最小距离
    if (distance < minDistanceToGoal) {
        minDistanceToGoal = distance;
    }
    
    // // 计算目标点相对于车辆的方向角
    double targetDir = atan2(dy, dx);

    if (distance < stopDisThre) {
        positionReached = true;
        currentState = ROTATING_TO_YAW;
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        return;
    }
    else
    {
        currentState = MOVING_TO_POSITION;
    }

    double yawError = targetDir - vehicleYaw;
    double curvature = 2.0 * sin(yawError) / distance;

    if (isDrivingBackward)
    {
        double speed = std::max(maxBackwardSpeed, -distance * slowDownFactor);
        cmd_vel.linear.x = speed;
        cmd_vel.angular.z = std::min(std::max(curvature * speed * yawRateGain, -maxYawRate), maxYawRate);
    }
    else
    {
        double speed = std::min(maxForwardSpeed, distance * slowDownFactor);
        cmd_vel.linear.x = speed;
        cmd_vel.angular.z = std::min(std::max(curvature * speed * yawRateGain, -maxYawRate), maxYawRate);
    }
}

void rotateToTargetYaw()
{
    // // 计算航向误差（考虑角度环绕）
    double yawError = targetFinalYaw - vehicleYaw;
    if(abs(yawError)>PI)
    {
        yawError = yawError - yawError/abs(yawError)*2*PI;
    }

    if (fabs(yawError) < yawTolerance/180*PI) 
    {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        currentState = IDLE;
        hasGoal = false;
        return;
    }
    else
    {
        currentState = ROTATING_TO_YAW;
    }

    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = std::min(std::max(yawError * yawRateGain, -maxYawRate), maxYawRate);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");
    
    // 加载参数，使用launch文件中的值作为默认值
    nhPrivate.param("lookAheadDis", lookAheadDis, 0.5);
    nhPrivate.param("yawRateGain", yawRateGain, 5.0);
    nhPrivate.param("maxYawRate", maxYawRate, 1.0);
    nhPrivate.param("maxForwardSpeed", maxForwardSpeed, 0.8);
    nhPrivate.param("maxBackwardSpeed", maxBackwardSpeed, -0.4);
    nhPrivate.param("stopDisThre", stopDisThre, 0.05);
    nhPrivate.param("yawTolerance", yawTolerance, 0.087);
    nhPrivate.param("slowDownFactor", slowDownFactor, 0.5);
    nhPrivate.param("backwardAngleThre", backwardAngleThre, 30.0 * PI/180);
    
    ros::Subscriber subGoal = nh.subscribe("/waypoint", 1, goalHandler);
    ros::Subscriber subOdom = nh.subscribe("/state_estimation", 5, odomHandler);
    
    // 发布 geometry_msgs::Twist 消息
    ros::Publisher pubSpeed = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    
    ros::Rate rate(20);  // 20Hz控制频率
    
    while (ros::ok()) {
        ros::spinOnce();
        
        switch (currentState) {
            case ADJUST_POSE:
                adjust_pose();
                break;

            case MOVING_TO_POSITION:
                moveToPosition();  // 移动到目标位置（前进或倒车）
                break;
                
            case ROTATING_TO_YAW:
                rotateToTargetYaw();  // 在目标位置旋转到目标朝向
                break;
                
            case IDLE:
                // 空闲状态发送零速度命令
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                break;
        }
        
        // 发布速度命令
        pubSpeed.publish(cmd_vel);
        rate.sleep();
    }
    
    return 0;
}
