//
// Created by kevin on 2020/3/25.
//

#include <ros/ros.h>
// for eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tf/transform_listener.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_listener");

    ros::NodeHandle node;

    tf::TransformListener listener;

    ros::Rate rate(10.0);
    while (node.ok()){
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/bed_link", "/camera",
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        tf::Vector3 trans = transform.getOrigin();
        tf::Quaternion qua = transform.getRotation();

        Eigen::Isometry3d Rt = Eigen::Isometry3d::Identity();
        Rt.rotate(Eigen::Quaterniond(qua.w(), qua.x(), qua.y(), qua.z()).normalized().toRotationMatrix());
        Rt.pretranslate(Eigen::Vector3d(trans.x(), trans.y(), trans.z()));

        cout << "tf_matrix:\n" << Rt.matrix() << endl;

        cout << "qua:\n" << qua.w() << " " << qua.x() << " " << qua.y() << " " << qua.z() << endl;

        rate.sleep();
    }
    return 0;
};