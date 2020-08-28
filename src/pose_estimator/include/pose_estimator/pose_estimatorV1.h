/*
 * Created by kevin on 2020/3/19.
 *
 * File: pose_estimatorV1.cpp
 * Function:
 * 作为姿态估计节点。
 * 该节点融合 1-机械传感器数据 2-IMU数据 3-视觉里程计数据 输出最优的姿态估计数据。
 *
 * V1版本仅供仿真环境使用，为了简化直接将TF数据转发。
 *
 * Subscribe Topic info:
 * mechanical data:         /tf & /tf_static                tf2_msgs/TFMessage
 * IMU data:                /imu                            sensor_msgs/IMU
 * VO data:                 /vo_pose                        geometry_msgs/Pose
 *
 * Publish Topic info:
 * estimated data:          /pose_estimated                 geometry_msgs/Pose ?
 *
 */

#ifndef POSE_ESTIMATOR_POSE_ESTIMATORV1_H
#define POSE_ESTIMATOR_POSE_ESTIMATORV1_H
#include <string>
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf2_msgs/TFMessage.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Transform.h"

class Pose_estimator{
private:
    ros::NodeHandle n_;
    ros::Subscriber imu_sub, vo_pose_sub, mechanical_sub;
    ros::Publisher pose_estimate_pub;
    ros::Timer pub_timer;

    geometry_msgs::Pose vo_pose;
    geometry_msgs::Transform mechanical_tf;
    sensor_msgs::Imu imu_data;

    tf::TransformListener listener;

    geometry_msgs::Pose estimated_pose;

public:
    Pose_estimator();
    void imu_sub_Callback(const sensor_msgs::Imu::ConstPtr& msg);
    void vo_pose_sub_Callback(const geometry_msgs::Pose::ConstPtr& msg);
    void mechanical_sub_Callback(const tf2_msgs::TFMessage::ConstPtr& msg);

    void pub_estimated_pose(const ros::TimerEvent&);
};

#endif //POSE_ESTIMATOR_POSE_ESTIMATORV1_H
