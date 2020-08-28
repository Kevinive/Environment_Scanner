//
// Created by kevin on 2020/3/21.
//

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv){

    Mat raw_img_1 = imread("/home/kevin/catkin_workspace/src/arm_scan_controller/scan_data/img/266.png");
    Mat raw_img_2 = imread("/home/kevin/catkin_workspace/src/arm_scan_controller/scan_data/img/267.png");
    if(raw_img_1.empty() || raw_img_2.empty()){
        cout << "could not load image..." << endl;
        return -1;
    }
//    namedWindow("input image", CV_WINDOW_AUTOSIZE);
//    imshow("input image", raw_img_1);

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
    drawKeypoints(raw_img_1, keypoints_2, keypoint_img, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("KeyPoints Image", keypoint_img);

    vector<DMatch> matches;
    BFMatcher matcher(NORM_HAMMING);
    matcher.match(descriptors_1, descriptors_2, matches);

    double min_dist = 1000, max_dist = 0;

    for(int i=0; i<descriptors_1.rows; i++){
        double dist = matches[i].distance;
        if(dist<min_dist) min_dist = dist;
        if(dist>max_dist) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    std::vector< DMatch > good_matches;
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( matches[i].distance <= max ( 2*min_dist, 40.0 ) )
        {
            good_matches.push_back ( matches[i] );
        }
    }

    //-- 第五步:绘制匹配结果
    Mat img_match;
    Mat img_goodmatch;
    drawMatches ( raw_img_1, keypoints_1, raw_img_2, keypoints_2, matches, img_match );
    drawMatches ( raw_img_1, keypoints_1, raw_img_2, keypoints_2, good_matches, img_goodmatch );
    imshow ( "all", img_match );
    imshow ( "optimized", img_goodmatch );

    waitKey(0);

    return 0;
}