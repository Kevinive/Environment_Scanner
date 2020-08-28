//
// Created by kevin on 2020/3/24.
//

#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv){
    ros::init(argc, argv, "transfer_broadcaster");

    ros::NodeHandle node;

    tf::TransformBroadcaster br;
    tf::TransformListener lr;
    tf::Transform transform1, transform2;
    transform1.setOrigin( tf::Vector3(2.90327, -6.34594, 0.106495) );
    transform2.setOrigin( tf::Vector3(2.78636, -6.34139, 0.105649) );
    tf::Quaternion q1(0.381391, -0.00530269, 0.924306, 0.0130833);
    tf::Quaternion q2(0.384689, 0.00350636, 0.923001, -0.00840533);

    Isometry3d Rt_1 = Isometry3d::Identity();
    Rt_1.rotate(Quaterniond(0.0130833, 0.381391, -0.00530269, 0.924306).normalized().toRotationMatrix());
    Rt_1.pretranslate(Vector3d(2.90327, -6.34594, 0.106495));
    Isometry3d Rt_2 = Isometry3d::Identity();
    Rt_2.rotate(Quaterniond(-0.00840533, 0.384689, 0.00350636, 0.923001).normalized().toRotationMatrix());
    Rt_2.pretranslate(Vector3d(2.78636, -6.34139, 0.105649));

    Isometry3d T_R_C = Rt_1.inverse() * Rt_2;

    cout << "T_R_C:\n" << T_R_C.matrix() << endl;

    transform1.setRotation(q1);
    transform2.setRotation(q2);
    br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "world", "camera1"));
    br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "world", "camera2"));

    ROS_INFO("init finished!");

    ros::Rate loop_rate(1);

    while(ros::ok()){
        loop_rate.sleep();
        ros::spinOnce();
        br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "world", "camera1"));
        br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "world", "camera2"));

        tf::StampedTransform transform;
        try{
            lr.lookupTransform("/camera1", "/camera2",
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

        tf::Matrix3x3 R = transform.getBasis();
        tf::Vector3 t = transform.getOrigin();

        cout << "R:\n" << R[0][0] << "\t" <<  R[0][1] << "\t" << R[0][2] << "\n" <<
                          R[1][0] << "\t" <<  R[1][1] << "\t" << R[1][2] << "\n" <<
                          R[2][0] << "\t" <<  R[2][1] << "\t" << R[2][2] << endl;

        cout << "t:\n" << t[0] << "\t" <<  t[1] << "\t" << t[2] << endl;


    }

    return 0;
}
