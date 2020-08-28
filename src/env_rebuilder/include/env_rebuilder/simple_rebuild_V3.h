//
// Created by kevin on 2020/3/23.
//

#ifndef CATKIN_WORKSPACE_SIMPLE_REBUILD_V3_H
#define CATKIN_WORKSPACE_SIMPLE_REBUILD_V3_H

#include <iostream>
#include <fstream>

#include "env_rebuilder/common_include.h"

// for sophus
#include <sophus/se3.h>

// for eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/viz.hpp>

// for pcl
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

#define CX          640.5
#define CY          360.5
#define FX          537.0397155022208
#define FY          537.0397155022208
#define IMG_WIDTH   1280
#define IMG_HIGHT  720

#define PARAM_FILE      "/home/kevin/catkin_workspace/src/env_rebuilder/config/v3.yaml"

// 定义点云使用的格式：这里用的是XYZRGB
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

#include "env_rebuilder/frame.h"
#include "env_rebuilder/point_cloud.h"
#include "env_rebuilder/feature_map.h"
#include "env_rebuilder/surface.h"
#include "env_rebuilder/camera.h"
#include "env_rebuilder/config.h"
#include "env_rebuilder/lidarpoint.h"
#include "env_rebuilder/feature.h"






#endif //CATKIN_WORKSPACE_SIMPLE_REBUILD_V3_H
