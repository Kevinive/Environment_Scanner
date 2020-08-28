//
// Created by kevin on 2020/3/24.
//

// std
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>

#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

// for Sophus
#include <sophus/se3.h>
#include <sophus/so3.h>
using Sophus::SO3;
using Sophus::SE3;

// for cv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using cv::Mat;

using namespace std;


int main(int argc, char** argv){

    Eigen::Isometry3d Rt_1 = Eigen::Isometry3d::Identity();
    Rt_1.rotate(Eigen::Quaterniond(0.810028, -0.0818161, -0.118604, -0.568414).normalized().toRotationMatrix());
    Rt_1.pretranslate(Eigen::Vector3d(0.053912, -0.149225, 0.388353));
    Eigen::Isometry3d Rt_2 = Eigen::Isometry3d::Identity();
    Rt_2.rotate(Eigen::Quaterniond(0.834345, -0.0752564, -0.120012, -0.532731).normalized().toRotationMatrix());
    Rt_2.pretranslate(Eigen::Vector3d(0.0668416, -0.144236, 0.387573));

    Eigen::Matrix<double, 3, 4> K;
    K << 111, 0, 333, 0, 0, 222, 444, 0, 0, 0, 1, 0;

    Eigen::Matrix<double, 3, 4> p1;
    p1 = K*Rt_1.matrix();

    cout << "K:\n" << K << endl;
    cout << "Rt_1:\n" << Rt_1.matrix() << endl;
    cout << "p1:\n" << p1 << endl;

    cout << "p1 row1:\n" << p1.row(1) << endl;

    Eigen::Vector3d testV1(1,1,1);
    Eigen::Vector3d testV2(-1,1,1);
    double result1 = testV1.dot(testV2);
    Eigen::Vector3d result2 = testV1.cross(testV2);

    cout << "result1:" << result1 << endl;
    cout << "|testV1|:" << testV1.norm() << endl;
    cout << "result2:\n" << result2 << endl;

}