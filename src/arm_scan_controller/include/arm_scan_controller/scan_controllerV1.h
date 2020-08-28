//
// Created by kevin on 2020/3/20.
//

#ifndef ARM_SCAN_CONTROLLER_SCAN_CONTROLLERV1_H
#define ARM_SCAN_CONTROLLER_SCAN_CONTROLLERV1_H

#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <arm_scan_controller/Scanpackage.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Pose.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#define YAW_STEP_DEG        10
#define PITCH_STEP_DEG      10

#define YAW_STEP            YAW_STEP_DEG*M_PI/180.0
#define PITCH_STEP          PITCH_STEP_DEG*YAW_STEP_DEG*M_PI/64800.0

#define PITCH_LIMIT_DEG     120
#define PITCH_LIMIT         PITCH_LIMIT_DEG*M_PI/180.0

#define DATA_PREFIX         "/home/kevin/catkin_workspace/src/arm_scan_controller/scan_dataV2"

class Scan_controller{
private:
    ros::NodeHandle n_;
    ros::Subscriber pose_sub, image_sub, lidar_sub;
    ros::Publisher scanPackage_pub, yaw_cmd_pub, pitch_cmd_pub;

    std_msgs::Float64 pitch, yaw;

    arm_scan_controller::Scanpackage packedData;

    sensor_msgs::Image img;

    bool scan_finished;

    std::ofstream outfile;
    std::string fileAdd;
    std::string imgAdd;

    int imgIndex;

public:
    Scan_controller();
    ~Scan_controller();
    void pose_sub_Callback(const geometry_msgs::Pose::ConstPtr& msg);
    void image_sub_Callback(const sensor_msgs::Image::ConstPtr& msg);
    void lidar_sub_Callback(const sensor_msgs::PointCloud::ConstPtr& msg);

    void next_step();

    float getPitch();
    float getYaw();
    bool isScanFinished();
};




#endif //ARM_SCAN_CONTROLLER_SCAN_CONTROLLERV1_H
