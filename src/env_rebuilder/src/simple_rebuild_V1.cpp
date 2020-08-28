//
// Created by kevin on 2020/3/22.
//

#include <iostream>
#include <fstream>

// for sophus
#include <sophus/se3.h>

// for eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;
using Sophus::SE3;
using namespace Eigen;
using namespace cv;

const double cx = 640.5;
const double cy = 360.5;
const double fx = 537.0397155022208;
const double fy = 537.0397155022208;
const int width = 1280;  	// 宽度
const int height = 720;  	// 高度

// 定义点云使用的格式：这里用的是XYZRGB
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;


bool readDatasetFiles(
        const string& path,
        vector< string >& color_image_files,
        vector< double >& deep_data,
        vector< Isometry3d >& poses
)
{
    ifstream fin( path+"img_seq_pose.txt");
    if ( !fin ) return false;

    while ( !fin.eof() )
    {
        // 数据格式：图像文件名 tx, ty, tz, qx, qy, qz, qw ，注意是 TWC 而非 TCW
        string image;
        fin>>image;
        double data[8];
        for ( double& d:data ) fin>>d;

        // pushback img name
        color_image_files.push_back( path+string("img/")+image );

        // pushback matrix
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.rotate(Quaterniond(data[3], data[4], data[5], data[6]).normalized().toRotationMatrix());
        T.pretranslate(Vector3d(data[0], data[1], data[2]));
//        cout << "included: " << image << "\n" << T.matrix() << endl;
        poses.push_back(T);

        // pushback depth info
        deep_data.push_back(data[7]);
        if ( !fin.good() ) break;
    }
    fin.close();
    return true;
}


void update(const Mat& curr, const Isometry3d& T_C_W, const double depth, PointCloud::Ptr point_cloud){

    Vector3d pixel_C(depth, 0.0, 0.0);
    Vector3d pixel_W = T_C_W * pixel_C;

    PointT p;
    p.x = pixel_W.x();
    p.y = pixel_W.y();
    p.z = pixel_W.z();
    int pixelIndex = int(cy)*curr.step + int(cx)*curr.channels();
    p.b = curr.data[pixelIndex];
    p.g = curr.data[pixelIndex+1];
    p.r = curr.data[pixelIndex+2];

    point_cloud->points.push_back(p);

    //cout << "x: " << p.x << "y: " << p.y <<"z: " << p.z << endl;
}


int main( int argc, char** argv ){
    vector<string> color_image_files;
    vector<double> depth;
    vector<Isometry3d> poses_TCW;

    string datasetName = "scan_dataV2";

    string base_address = "/home/kevin/catkin_workspace/src/arm_scan_controller/" + datasetName + "/";

    bool ret = readDatasetFiles( base_address, color_image_files, depth, poses_TCW );
    if ( !ret )
    {
        cout<<"Reading image files failed!"<<endl;
        return -1;
    }
//    cout<<"read total "<< color_image_files.size() <<" files."<<endl;
//    cout<<"the 66 picture name is: " << color_image_files.at(205) << endl;
//    cout<<"the 66 picture distance is: " << to_string(depth.at(205)) << endl;


    PointCloud::Ptr pointCloud( new PointCloud );

    for ( int index = 1; index<color_image_files.size(); index++ )
    {
        cout<<"*** loop "<<index<<" ***"<<endl;
        Mat curr = imread( color_image_files[index], 0 );
        if (curr.data == nullptr) continue;
        Isometry3d pose_curr_TCW = poses_TCW[index];
        double curr_depth = depth[index];
        update(curr, pose_curr_TCW, curr_depth, pointCloud);
//        imshow("image", curr);
//        waitKey(1);
    }

    pcl::io::savePCDFileBinary("/home/kevin/catkin_workspace/src/env_rebuilder/result/" + datasetName + ".pcd", *pointCloud );

    return 0;
}