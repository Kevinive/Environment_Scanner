//
// Created by kevin on 2020/3/20.
//

#ifndef ARM_SCAN_CONTROLLER_SCAN_CONTROLLERV3_H
#define ARM_SCAN_CONTROLLER_SCAN_CONTROLLERV3_H

#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <fstream>

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Pose.h>
#include <arm_scan_controller/Scanpackage.h>
#include <arm_scan_controller/ScanPoseXYZ.h>

// for eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#define PITCH_SCAN_START_DEG        30.0
#define YAW_SCAN_START_DEG          0.0
#define YAW_STEP_DEG        2
#define PITCH_STEP_DEG      2
#define IMG_YAW_STEP_DEG        30
#define IMG_PITCH_STEP_DEG        30

#define PITCH_SCAN_START    PITCH_SCAN_START_DEG*M_PI/180.0
#define YAW_SCAN_START      YAW_SCAN_START_DEG*M_PI/180.0
#define YAW_STEP            YAW_STEP_DEG*M_PI/180.0
#define PITCH_STEP          PITCH_STEP_DEG*M_PI/180.0
#define IMG_YAW_STEP_COUNT          int(IMG_YAW_STEP_DEG/YAW_STEP_DEG)
#define IMG_PITCH_STEP_COUNT        int(IMG_PITCH_STEP_DEG/PITCH_STEP_DEG)

#define PITCH_LIMIT_DEG     120
#define PITCH_LIMIT         PITCH_LIMIT_DEG*M_PI/180.0

#define DATA_PREFIX         "/home/kevin/catkin_workspace/src/arm_scan_controller/scan_dataV3"

class Scan_controller{
private:
    ros::NodeHandle n_;
    ros::Subscriber pose_sub, image_sub, lidar_sub, cmd_pose_sub, autoscan_cmd_sub;
    ros::Publisher scanPackage_pub, yaw_cmd_pub, pitch_cmd_pub, autoscan_state_pub;
    ros::ServiceServer scanPose_service;

    std_msgs::Float64 pitch, yaw;

    arm_scan_controller::Scanpackage packedData;
    geometry_msgs::Pose lastPose;
    sensor_msgs::Image img;

    Eigen::Matrix4d T_W_Cen;

    std::ofstream outPosefile, outImgfile;
    std::string posefileAdd, imgfileAdd;
    std::string imgAdd;

    int imgIndex;
    bool staticFlag, autoScanState;

    ros::Timer autoscan_timer;
    ros::Timer autoscan_state_pub_timer;

public:
    Scan_controller();
    ~Scan_controller();
    void pose_sub_Callback(const geometry_msgs::Pose::ConstPtr& msg);
    void image_sub_Callback(const sensor_msgs::Image::ConstPtr& msg);
    void lidar_sub_Callback(const sensor_msgs::PointCloud::ConstPtr& msg);

    bool scanpos_Callback(arm_scan_controller::ScanPoseXYZ::Request  &req,
                            arm_scan_controller::ScanPoseXYZ::Response &res);

    void pub_state();
    void save_pose_data();
    void save_img_data();
    void start_service();
    void stop_service();

    void autoscan_sub_Callback(const std_msgs::Bool::ConstPtr& msg);
    void start_autoscan();
    void stop_autoscan();
    void autoscan_Callback(const ros::TimerEvent&);
    void autoscan_state_pub_Callback(const ros::TimerEvent&);

    double getPitch();
    double getYaw();
    bool setPitch(double val);
    bool setYaw(double val);


};




#endif //ARM_SCAN_CONTROLLER_SCAN_CONTROLLERV3_H
