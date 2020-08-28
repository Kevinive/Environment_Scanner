//
// Created by kevin on 2020/3/21.
//

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <iostream>

using namespace cv::xfeatures2d;
using namespace cv;
using namespace std;

int main(int argc, char** argv){

    Mat raw_img = imread("/home/kevin/catkin_workspace/src/arm_scan_controller/scan_data/img/166.png");
    if(raw_img.empty()){
        cout << "could not load image..." << endl;
        return -1;
    }
    namedWindow("input image", CV_WINDOW_AUTOSIZE);
    imshow("input image", raw_img);

    // SIFT Detect
    int nfeatures = 100;
    int nOctaveLayers = 3;
    double contrastThreshold = 0.04;
    double edgeThreshold = 10;
    double sigma = 1.6;

    Ptr<SIFT> detector = SIFT::create(nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);
    vector<KeyPoint> keypoints;
    detector->detect(raw_img, keypoints, Mat());

    Mat keypoint_img;
    drawKeypoints(raw_img, keypoints, keypoint_img, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("KeyPoints Image", keypoint_img);

    waitKey(0);

    return 0;
}