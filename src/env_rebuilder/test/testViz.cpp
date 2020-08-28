//
// Created by kevin on 2020/4/4.
//

#include "opencv2/opencv.hpp"
#include "opencv2/viz.hpp"
using namespace cv;
using namespace std;

int main()
{
    //加载深度图（这里用的kinect2 的深度图）
    std::string filename= "/home/kevin/下载/slambook-master/ch13/dense_RGBD/data/depth/3.pgm" ;
    cv::Mat depth = cv::imread(filename,-1);
    //初始化
    viz::Viz3d window("window");
    //显示坐标系
    window.showWidget("Coordinate", viz::WCoordinateSystem());
    int height = depth.rows;
    int width  = depth.cols;
    //创建一个储存point cloud的图片
    Mat point_cloud = Mat::zeros(height, width, CV_32FC3);
    //point cloud 赋值，其中 fx,fy,cx,cy 为Kinect2 的内参
    double fx = 518.0, fy = 519.0, cx = 325.5, cy = 253.5;
    double depthScale = 1000.0;
    for(int row=0; row<depth.rows;row++)
        for (int col = 0; col < depth.cols; col++)
        {
            point_cloud.at<Vec3f>(row, col)[2] = depth.at<unsigned short>(row,col) / depthScale;
            point_cloud.at<Vec3f>(row, col)[0] = point_cloud.at<Vec3f>(row, col)[2] * (col - cx) / fx;
            point_cloud.at<Vec3f>(row, col)[1] = point_cloud.at<Vec3f>(row, col)[2] * (row - cy) / fy;
        }
    cv::viz::WCloud cloud(point_cloud);
    window.showWidget("cloud",cloud);
    while (!window.wasStopped())
    {
        window.spinOnce(1, false);
    }
    return 0;
}