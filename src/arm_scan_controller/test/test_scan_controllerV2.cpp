//
// Created by kevin on 2020/3/26.
//

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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

bool is_autoscan = true;

ros::ServiceClient client;

void autoscan_sub_Callback(const std_msgs::Bool::ConstPtr& msg){
    is_autoscan = msg->data;
}



void callService(int val){
    double x, y, z;

    if(val & 0x01){
        x = 10.0;
    }else{
        x = -10.0;
    }

    if(val & 0x02){
        y = 10.0;
    }else{
        y = -10.0;
    }

    if(val & 0x04){
        z = 1.0;
    }else{
        z = -1.0;
    }

    ROS_INFO("request for Pos: %lf %lf %lf", x, y, z);
    arm_scan_controller::ScanPoseXYZ srv;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.z = z;

    if (client.call(srv))
    {
        ROS_INFO("Distance: %lf", (double)srv.response.distance);
    }
    else
    {
        ROS_ERROR("Failed to call service ScanPoseXYZ");
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "test_scan_controller");

    ros::NodeHandle n_;

    ros::Subscriber scanok_sub = n_.subscribe("arm_v2/autoscan_state", 10, &autoscan_sub_Callback);
    client = n_.serviceClient<arm_scan_controller::ScanPoseXYZ>("scan_pose_XYZ");

    ros::Rate loop_rate(0.5);

    int count = 0;
    while(ros::ok()){
        loop_rate.sleep();
        ros::spinOnce();
        if(!is_autoscan){
            ROS_INFO("Calling Service.........");
            callService(count);
            ROS_INFO("Finished One Service!");
            count++;
        }else{
            ROS_INFO("Service is busy.........");
        }


    }

    return 0;
}