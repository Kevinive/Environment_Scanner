//
// Created by kevin on 2020/3/23.
//

// for eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

// for sophus
#include <sophus/se3.h>
using Sophus::SE3;

using namespace std;
using namespace Eigen;
using namespace cv;

const double cx = 640.5;
const double cy = 360.5;
const double fx = 537.0397155022208;
const double fy = 537.0397155022208;

void triangulate_1(const Point2f& pt1, const Point2f& pt2,
        const Matrix<double, 3, 4> &P1, const Matrix<double, 3, 4> &P2, cv::Mat &x3D)
{
    cv::Mat A;

    Matrix<double, 4, 4> AT;

    AT.row(0) = pt1.x*P1.row(2)-P1.row(0);
    AT.row(1) = pt1.y*P1.row(2)-P1.row(1);
    AT.row(2) = pt2.x*P2.row(2)-P2.row(0);
    AT.row(3) = pt2.y*P2.row(2)-P2.row(1);

    cv::eigen2cv(AT,A);

    cv::Mat u,w,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    x3D = vt.row(3).t();

    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
}

Vector3d triangulation(const Point2f& pt1, const Point2f& pt2,
        const Isometry3d& Rt_1, const Isometry3d& Rt_2)
{

    Vector3d f_ref((pt1.x - cx)/fx, (pt1.y - cy)/fy, 1);
    f_ref.normalize();
    Vector3d f_curr((pt2.x - cx)/fx, (pt2.y - cy)/fy, 1);
    f_curr.normalize();

//    Vector3d f_ref(-0.527917, -0.0432931, 0.848192);
//    f_ref.normalize();
//    Vector3d f_curr(-0.527888, -0.0437347, 0.848187);
//    f_curr.normalize();

    cout << "f_ref:\n" << f_ref << endl;
    cout << "f_curr:\n" << f_curr << endl;
    cout << "p_ref coord:\n" << Rt_1.matrix() << endl;
    cout << "p_curr coord:\n" << Rt_2.matrix() << endl;

    Isometry3d T_R_C = Rt_1.inverse() * Rt_2;
    cout << "T_R_C matrix:\n" << T_R_C.matrix() << endl;

/*
 * Method 3 : 使用ORB-SLAM:Triangulate中方法进行三角化计算，失败！：
 */
/*
    Eigen::Matrix<double, 3, 4> K;
    K << fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0;

//    Eigen::Matrix<double, 3, 4> p1;
//    p1 = K*Rt_1.matrix();
//
//    Eigen::Matrix<double, 3, 4> p2;
//    p2 = K*Rt_2.matrix();

    Eigen::Matrix<double, 3, 4> p1;
    p1 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0;

    Eigen::Matrix<double, 3, 4> p2;
    p2 = K*T_R_C.matrix();

    cv::Mat x3D;
    triangulate_1(pt1, pt2, p1, p2, x3D);

    cout<<"x3D:\n"<<x3D<<std::endl;

    return Vector3d(x3D.at<double>(0), x3D.at<double>(1), x3D.at<double>(2));

*/

/*
 * Method 2 : 使用视觉SLAM十四讲第十三讲中算法计算三角化，失败!；
 * 方程
    // d_ref * f_ref = d_cur * ( R_RC * f_cur ) + t_RC
    // => [ f_ref^T f_ref, -f_ref^T f_cur ] [d_ref] = [f_ref^T t]
    //    [ f_cur^T f_ref, -f_cur^T f_cur ] [d_cur] = [f_cur^T t]
    // 二阶方程用克莱默法则求解并解之
 */

/*
    Vector3d t = T_R_C.translation();
    Vector3d f2 = T_R_C.rotation() * f_curr;
    Vector2d b = Vector2d (t.dot(f_ref), t.dot(f2));

    double A[4];
    A[0] = f_ref.dot ( f_ref );
    A[2] = f_ref.dot ( f2 );
    A[1] = -A[2];
    A[3] = - f2.dot ( f2 );
    double d = A[0]*A[3]-A[1]*A[2];
    Vector2d lambdavec =
            Vector2d (  A[3] * b ( 0,0 ) - A[1] * b ( 1,0 ),
                        -A[2] * b ( 0,0 ) + A[0] * b ( 1,0 )) /d;
    Vector3d xm = lambdavec ( 0,0 ) * f_ref;
    Vector3d xn = t + lambdavec ( 1,0 ) * f2;
    Vector3d d_esti = ( xm+xn ) / 2.0;  // 三角化算得的深度向量
    double depth_estimation = d_esti.norm();   // 深度值
    cout << "depth_vector is:\n" << d_esti << "\n"<< endl;
    cout << "depth_estimation is:\n" << depth_estimation << "\n"<< endl;

    Vector3d pixel_C(depth_estimation, 0.0, 0.0);
    Vector3d pixel_W = Rt_1 * pixel_C;
    return pixel_W;
*/

/*
 * Method 1: From 视觉SLAM十四讲第七讲，使用CV自带三角化函数计算，失败；
 */

    /*Mat T1 = (Mat_<float>(3,4) <<
            Rt_1(0,0), Rt_1(0,1), Rt_1(0,2), Rt_1(0,3),
            Rt_1(1,0), Rt_1(1,1), Rt_1(1,2), Rt_1(1,3),
            Rt_1(2,0), Rt_1(2,1), Rt_1(2,2), Rt_1(2,3));
    Mat T2 = (Mat_<float>(3,4) <<
            Rt_2(0,0), Rt_2(0,1), Rt_2(0,2), Rt_2(0,3),
            Rt_2(1,0), Rt_2(1,1), Rt_2(1,2), Rt_2(1,3),
            Rt_2(2,0), Rt_2(2,1), Rt_2(2,2), Rt_2(2,3));
    Mat K = (Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    vector<Point2f> pts_1, pts_2;
    for( DMatch m:matches ){
        pts_1.push_back(keypoint_1[m.queryIdx].pt);
        pts_2.push_back(keypoint_2[m.trainIdx].pt);
    }
    Mat pts_4d;
    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

    for ( int i=0; i<pts_4d.cols; i++ )
    {
        Mat x = pts_4d.col(i);
        x /= x.at<float>(3,0); // 归一化:此处的归一化是指从齐次坐标变换到非齐次坐标。而不是变换到归一化平面。
        Point3d p (
                x.at<float>(0,0),
                x.at<float>(1,0),
                x.at<float>(2,0)
        );
        points.push_back( p );
    }*/


}

int main(int argc, char** argv){

    Mat raw_img_1 = imread("/home/kevin/catkin_workspace/src/arm_scan_controller/scan_dataV1/img/148.png");
    Mat raw_img_2 = imread("/home/kevin/catkin_workspace/src/arm_scan_controller/scan_dataV1/img/149.png");

//    Mat raw_img_1 = imread("/home/kevin/catkin_workspace/src/cleanbot_controller/scan_dataV1/img/51.png");
//    Mat raw_img_2 = imread("/home/kevin/catkin_workspace/src/cleanbot_controller/scan_dataV1/img/57.png");

    if(raw_img_1.empty() || raw_img_2.empty()){
        cout << "could not load image..." << endl;
        return -1;
    }

    // ORB Detect
    vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<ORB> orb = ORB::create(100, 1.2f,8,31,
                               0,2, ORB::HARRIS_SCORE, 31, 20);

    orb->detect(raw_img_1, keypoints_1, descriptors_1);
    orb->detect(raw_img_2, keypoints_2, descriptors_2);

    orb->compute(raw_img_1, keypoints_1, descriptors_1);
    orb->compute(raw_img_2, keypoints_2, descriptors_2);

    Mat keypoint_img;
    drawKeypoints(raw_img_2, keypoints_2, keypoint_img, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("KeyPoints Image", keypoint_img);


    vector<DMatch> matches;
    BFMatcher matcher(NORM_HAMMING);
    matcher.match(descriptors_2, descriptors_1, matches);

    double min_dist = 1000, max_dist = 0;

    for(int i=0; i<descriptors_2.rows; i++){
        double dist = matches[i].distance;
        if(dist<min_dist) min_dist = dist;
        if(dist>max_dist) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    std::vector< DMatch > good_matches;
    for ( int i = 0; i < descriptors_2.rows; i++ )
    {
        //if ( matches[i].distance <= max ( 2*min_dist, 40.0 ) )
        if ( matches[i].distance <= min_dist+1.0 )
        {
            good_matches.push_back ( matches[i] );
        }
    }

    Isometry3d Rt_1 = Isometry3d::Identity();
    Rt_1.rotate(Quaterniond(0.810028, -0.0818161, -0.118604, -0.568414).normalized().toRotationMatrix());
    Rt_1.pretranslate(Vector3d(0.053912, -0.149225, 0.388353));
    Isometry3d Rt_2 = Isometry3d::Identity();
    Rt_2.rotate(Quaterniond(0.834345, -0.0752564, -0.120012, -0.532731).normalized().toRotationMatrix());
    Rt_2.pretranslate(Vector3d(0.0668416, -0.144236, 0.387573));

    for(int i=0; i<good_matches.size(); i++){
        cout << "In ref image:\ntemplate Point:" << keypoints_1[good_matches[i].trainIdx].pt << endl;
        cout << "In cur image:\ntemplate Point:" << keypoints_2[good_matches[i].queryIdx].pt << endl;

        Point2f p1 = keypoints_1[good_matches[i].trainIdx].pt;
        Point2f p2 = keypoints_2[good_matches[i].queryIdx].pt;

        Vector3d pt_real = triangulation(p1, p2, Rt_1, Rt_2);
    }

    //-- 第五步:绘制匹配结果
    Mat img_match;
    Mat img_goodmatch;
    drawMatches ( raw_img_2, keypoints_2, raw_img_1, keypoints_1, matches, img_match );
    drawMatches ( raw_img_2, keypoints_2, raw_img_1, keypoints_1, good_matches, img_goodmatch );
    imshow ( "all", img_match );
    imshow ( "optimized", img_goodmatch );

    waitKey(0);

    return 0;
}