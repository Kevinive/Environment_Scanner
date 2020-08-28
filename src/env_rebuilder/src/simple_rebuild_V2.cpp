//
// Created by kevin on 2020/3/22.
//

#include "env_rebuilder/simple_rebuild_V2.h"

using namespace std;
using namespace Eigen;
using namespace cv;

const double cx = CX;
const double cy = CY;
const double fx = FX;
const double fy = FY;
const int width = IMG_WIDTH;  	// 宽度
const int height = IMG_HIGHT;  	// 高度

/*
 * 读取数据文件
 */
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

/*
 * 获取ORB特征
 */
void extractORBFeature(const Mat &img, vector<cv::KeyPoint> &kp, Mat &descriptors) {
    cv::Ptr<cv::ORB> orb = cv::ORB::create(100, 1.2f, 8, 31,
                                           0, 2, cv::ORB::HARRIS_SCORE, 31, 20);

    orb->detect(img, kp);
    orb->compute(img, kp, descriptors);
}

/*
 * 获取ORB特征
 */
void extractSIFTFeature(const Mat &img, vector<cv::KeyPoint> &kp, Mat &descriptors) {
    cv::Ptr<cv::xfeatures2d::SIFT> detector = cv::xfeatures2d::SIFT::create(100, 3, 0.04, 10, 1.6);

    detector->detect(img, kp, Mat());
    detector->compute(img, kp, descriptors);
}



/*
 * 更新点云信息
 */
void update_laserPoint(const Mat& curr, const Isometry3d& T_W_C, const double depth, PointCloud::Ptr point_cloud){

    Vector3d pixel_C(depth, 0.0, 0.0);
    Vector3d pixel_W = T_W_C * pixel_C;

    PointT p;
    p.x = pixel_W.x();
    p.y = pixel_W.y();
    p.z = pixel_W.z();
    int pixelIndex = int(cy)*curr.step + int(cx)*curr.channels();
    p.b = curr.data[pixelIndex];
    p.g = curr.data[pixelIndex+1];
    p.r = curr.data[pixelIndex+2];

    point_cloud->points.push_back(p);

//    cout << "x: " << p.x << "y: " << p.y <<"z: " << p.z << endl;
}


// 像素到相机坐标系
inline Vector3d px2cam (const Point2f px ) {
    return Vector3d (
            (px.x - cx)/fx,
            (px.y - cy)/fy,
            1
    );
}

void triangulation(
        const vector<KeyPoint>& keypoint_1,
        const vector<KeyPoint>& keypoint_2,
        const std::vector<DMatch>& matches,
        const Isometry3d& Rt_1, const Isometry3d& Rt_2,
        vector<Point3d>& points
)
{

    for( DMatch m:matches ){
        Vector3d f_ref = px2cam(keypoint_1[m.trainIdx].pt);
        f_ref.normalize();
        Vector3d f_curr = px2cam(keypoint_2[m.queryIdx].pt);
        f_curr.normalize();

        Isometry3d T_R_C = Rt_1.inverse() * Rt_2;

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
        cout << "depth_estimation is:\n" << depth_estimation << "\n"<< endl;

        Vector3d pixel_C(depth_estimation, 0.0, 0.0);
        Vector3d pixel_W = Rt_1 * pixel_C;

        points.push_back(Point3d(pixel_W.x(), pixel_W.y(), pixel_W.z()));
    }

    // the following algorithm FAILED!
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


/*
 * 更新点云信息
 */
void update(const Mat& curr, const Isometry3d& curr_pose,
    const Mat& ref, const Isometry3d& ref_pose, PointCloud::Ptr point_cloud){

    Mat descriptors_curr, descriptors_ref;
    vector<KeyPoint> keypoints_curr, keypoints_ref;

    extractORBFeature(curr, keypoints_curr, descriptors_curr);
    extractORBFeature(ref, keypoints_ref, descriptors_ref);

    if( keypoints_curr.size() < 5 || keypoints_ref.size() < 5) return;

    vector<DMatch> matches;
    BFMatcher matcher(NORM_HAMMING);
    matcher.match(descriptors_curr, descriptors_ref, matches);

    double min_dist = 1000, max_dist = 0;

    for(int i=0; i<descriptors_curr.rows; i++){
        double dist = matches[i].distance;
        if(dist<min_dist) min_dist = dist;
        if(dist>max_dist) max_dist = dist;
    }

//    printf ( "-- Max dist : %f \n", max_dist );
//    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    std::vector< DMatch > good_matches;
    for ( int i = 0; i < descriptors_curr.rows; i++ )
    {
        if ( matches[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            good_matches.push_back ( matches[i] );
        }
    }

    vector<Point3d> pts;
    triangulation(keypoints_ref, keypoints_curr, good_matches, ref_pose, curr_pose, pts);

    cout << "feature find: " << pts.size() << endl;
    for(int i=0; i<pts.size(); i++){
        PointT point;
        point.x = pts[i].x;
        point.y = pts[i].y;
        point.z = pts[i].z;
        int pixelIndex = (keypoints_ref[good_matches[i].trainIdx].pt.y)*ref.step
                + (keypoints_ref[good_matches[i].trainIdx].pt.x)*ref.channels();
        point.b = curr.data[pixelIndex];
        point.g = curr.data[pixelIndex+1];
        point.r = curr.data[pixelIndex+2];

        point_cloud->points.push_back(point);
    }
}


/*
 * main s
 */
int main( int argc, char** argv ){
    vector<string> color_image_files;
    vector<double> depth;
    vector<Isometry3d> poses_TWC;

    string base_address = "/home/kevin/catkin_workspace/src/arm_scan_controller/scan_dataV1/";

    bool ret = readDatasetFiles( base_address, color_image_files, depth, poses_TWC );
    if ( !ret )
    {
        cout<<"Reading image files failed!"<<endl;
        return -1;
    }
//    cout<<"read total "<< color_image_files.size() <<" files."<<endl;
//    cout<<"the 66 picture name is: " << color_image_files.at(205) << endl;
//    cout<<"the 66 picture distance is: " << to_string(depth.at(205)) << endl;

    int startIndex = 63;

    PointCloud::Ptr pointCloud( new PointCloud );
    Mat ref_img = imread( color_image_files[startIndex], 0 );
    Isometry3d pose_ref_TWC = poses_TWC[startIndex];
    double curr_depth = depth[startIndex];
    update_laserPoint(ref_img, pose_ref_TWC, curr_depth, pointCloud);

    for ( int index = startIndex + 1; index<color_image_files.size(); index++ )
    {
        cout<<"*** loop "<<index<<" ***"<<endl;

        Mat curr_img = imread( color_image_files[index], 0 );
        if (curr_img.data == nullptr) continue;

        Isometry3d pose_curr_TWC = poses_TWC[index];
        double curr_depth = depth[index];
        update_laserPoint(curr_img, pose_curr_TWC, curr_depth, pointCloud);
        update(curr_img, pose_curr_TWC, ref_img, pose_ref_TWC, pointCloud);

        ref_img = curr_img;
        pose_ref_TWC = pose_curr_TWC;
    }

    pcl::io::savePCDFileBinary("/home/kevin/catkin_workspace/src/env_rebuilder/result/simple_mapV1.pcd", *pointCloud );

    return 0;
}