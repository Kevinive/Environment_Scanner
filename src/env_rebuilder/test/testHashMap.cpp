//
// Created by kevin on 2020/4/8.
//

#include "env_rebuilder/point_cloud.h"
#include "env_rebuilder/lidarpoint.h"

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
using cv::Mat;

int main(int argc, char** argv){

    std::vector<env_rebuilder::LidarPoint::Ptr> pt;
    env_rebuilder::PointCloud pc(-5.0, 5.0, -3.0, 7.0, -2.0, 8.0, 1.0);


    for(int i=0; i < 10; i++){
        pt.push_back(env_rebuilder::LidarPoint::createLidarPoint(Eigen::Vector4d(i-5+0.1,i-3+0.1,i-2+0.1,1)));
        cout << "PointXYZ(" << to_string(i-5+0.1) << ", " << to_string(i-3+0.1) << ", " << to_string(i-2+0.1) << ")" << endl;
    }

    for(int i=0; i < 10; i++){
        pc.insertLidarPoint(pt[i]);
    }

    std::vector<env_rebuilder::LidarPoint::Ptr> ptvec;
    pc.getNearestPoint(pt[4], ptvec);
    pc.getNeighborPoint(pt[4], 1, ptvec);
    pc.getNeighborPoint(pt[4], 2, ptvec);
    for(env_rebuilder::LidarPoint::Ptr lp : ptvec){
        cout << lp->id_ << "\t";
    }
    cout << endl;

    std::vector<env_rebuilder::LidarPoint::Ptr> ptvecAll;
    pc.getAllPoint(ptvecAll);

    int a = 0;

    return 0;
}