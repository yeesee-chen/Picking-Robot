#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());

double laserCloudTime = 0;
bool systemInited = false;
double systemInitTime = 0;
double vehiclelenth = 0.39;
double vehiclewidth = 0.25;
double GroundZ = 0.05;
double TopZ = 0.3;
double UndergroundZ = -0.3;
double d_height = 0.1;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
float vehicleXRec = 0, vehicleYRec = 0;

float sinVehicleRoll = 0, cosVehicleRoll = 0;
float sinVehiclePitch = 0, cosVehiclePitch = 0;
float sinVehicleYaw = 0, cosVehicleYaw = 0;

double noDecayDis = 4.0;
bool newlaserCloud = false;

ros::Publisher *pubLaserCloudPointer = NULL;

void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom) {
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
        .getRPY(roll, pitch, yaw);

    vehicleRoll = roll;
    vehiclePitch = pitch;
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    vehicleZ = odom->pose.pose.position.z;

    sinVehicleRoll = sin(vehicleRoll);
    cosVehicleRoll = cos(vehicleRoll);
    sinVehiclePitch = sin(vehiclePitch);
    cosVehiclePitch = cos(vehiclePitch);
    sinVehicleYaw = sin(vehicleYaw);
    cosVehicleYaw = cos(vehicleYaw);
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloud2) {
    laserCloudTime = laserCloud2->header.stamp.toSec();

    if (!systemInited) {
        systemInitTime = laserCloudTime;
        systemInited = true;
    }

    laserCloud->clear();
    pcl::fromROSMsg(*laserCloud2, *laserCloud);

    pcl::PointXYZI point;
    laserCloudCrop->clear();
    int laserCloudSize = laserCloud->points.size();

    // 定义车辆在雷达局部坐标系中的边界参数
    double vehicleFrontX = 0;     // 车头在雷达坐标系中的X正方向位置
    double vehicleRearX = -vehiclelenth;     // 车尾在雷达坐标系中的X负方向位置
    double vehicleLeftY = vehiclewidth / 2;      // 左侧在雷达坐标系中的Y正方向位置
    double vehicleRightY = -vehiclewidth / 2;    // 右侧在雷达坐标系中的Y负方向位置
    double vehicleBottomZ = -0.5;     // 车底高度（假设值）
    double vehicleTopZ = 1.0;         // 车顶高度（假设值）

    // for (float x = 2.05; x < 2.25; x += 0.05)
    // {
    //     for (float y = 0.35; y < 2.05; y += 0.1)
    //     {
    //         for (float z = 0; z < 1; z += 0.1)
    //         {
    //             pcl::PointXYZI pre_projPoint;
    //             pre_projPoint.x = x;
    //             pre_projPoint.y = y;
    //             pre_projPoint.z = z;
    //             pre_projPoint.intensity = laserCloudTime - systemInitTime;
    //             laserCloudCrop->push_back(pre_projPoint);
    //         }
    //     }
    // }

    for (int i = 0; i < laserCloudSize; i++) {
        point = laserCloud->points[i];
        float pointX = point.x;
        float pointY = point.y;
        float pointZ = point.z;

        // --- Step 1: 转换到雷达局部坐标系 ---
        // 平移变换（减去雷达的全局坐标）
        float dx = pointX - vehicleX;
        float dy = pointY - vehicleY;
        float dz = pointZ;

        // 旋转变换（消除车辆朝向影响）
        double localX = dx * cosVehicleYaw + dy * sinVehicleYaw;
        double localY = -dx * sinVehicleYaw + dy * cosVehicleYaw;
        double localZ = dz;

        // --- Step 2: 判断点是否在车辆自身范围内 ---
        // bool isVehiclePoint = 
        //     (localX >= vehicleRearX) && (localX <= vehicleFrontX) &&
        //     (localY >= vehicleRightY) && (localY <= vehicleLeftY) &&
        //     (localZ >= vehicleBottomZ) && (localZ <= vehicleTopZ);

        bool isVehiclePoint = (localX <= vehicleFrontX);

        bool ground_or_top = (localZ <= GroundZ);// || (localZ >= TopZ);

        if (isVehiclePoint || ground_or_top) {
            continue; // 跳过车辆自身点
        }

        if ((localZ > GroundZ) && (localZ < GroundZ + 0.2)) {
            for (float i = GroundZ; i >= UndergroundZ; i -= d_height) {
                pcl::PointXYZI projPoint;
                projPoint.x = pointX;
                projPoint.y = pointY;
                projPoint.z = i;
                projPoint.intensity = laserCloudTime - systemInitTime;
                laserCloudCrop->push_back(projPoint);
            }
        }

        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        point.intensity = laserCloudTime - systemInitTime;
        laserCloudCrop->push_back(point);
    }

    newlaserCloud = true;

    sensor_msgs::PointCloud2 croppedCloudMsg;
    pcl::toROSMsg(*laserCloudCrop, croppedCloudMsg);    
    croppedCloudMsg.header.stamp = laserCloud2->header.stamp;
    croppedCloudMsg.header.frame_id = "map";
    pubLaserCloudPointer->publish(croppedCloudMsg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cut_cloud");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    nhPrivate.getParam("vehiclelenth", vehiclelenth);
    nhPrivate.getParam("vehiclewidth", vehiclewidth);
    nhPrivate.getParam("GroundZ", GroundZ);
    nhPrivate.getParam("TopZ", TopZ);

    ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, odometryHandler);
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/terrain_map", 5, laserCloudHandler);
    ros::Publisher g_pub_cropped = nh.advertise<sensor_msgs::PointCloud2>("/scan_cropped",1) ;
    pubLaserCloudPointer = &g_pub_cropped;

    ros::spin();

    return 0;
}