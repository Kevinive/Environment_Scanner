//
// Created by kevin on 2020/3/22.
//

#include "env_rebuilder/simple_rebuild_V3.h"

//------------------------TOOLS-----------------------------

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

//-----------------------!TOOLS!---------------------------


/*
 * main function
 */
int main( int argc, char** argv ){

    env_rebuilder::Config::setParameterFile ( PARAM_FILE );

    string dataset_dir = env_rebuilder::Config::get<string> ( "dataset_dir" );
    cout << "dataset: "<< dataset_dir << endl;
    ifstream fin ( dataset_dir+"/img_seq_pose.txt" );
    if ( !fin )
    {
        cout<<"please generate the associate file called img_seq_pose.txt!"<<endl;
        return 1;
    }

    vector< string > color_image_files;
    vector< double > deep_data;
    vector< SE3 > poses;
    env_rebuilder::Camera::Ptr cp = env_rebuilder::Camera::Ptr(new env_rebuilder::Camera());

    while ( !fin.eof() )
    {
        string image;
        fin>>image;
        double data[8];
        for ( double& d:data ) fin>>d;

        color_image_files.push_back ( dataset_dir+"/img/"+image );
        deep_data.push_back(data[7]);

        poses.push_back(SE3 (
                Eigen::Quaterniond ( data[3], data[4], data[5], data[6] ),
                Vector3d ( data[0], data[1], data[2] )
        )
        );

        if ( fin.good() == false )
            break;
    }

    env_rebuilder::Camera::Ptr camera ( new env_rebuilder::Camera );

    cout<<"read total "<<deep_data.size() <<" entries"<<endl;

    // Initialize Maps
    env_rebuilder::PointCloud::Ptr pointcloud(new env_rebuilder::PointCloud);
    env_rebuilder::FeatureMap::Ptr feature_map(new env_rebuilder::FeatureMap);
    std::vector<env_rebuilder::Frame::Ptr> frames;
    PointCloud::Ptr pc( new PointCloud );

    // !temporary param, improved in next gen
    double cx = 640.5;
    double cy = 360.5;
    double fx = 537.0397155022208;
    double fy = 537.0397155022208;

    string last_img_name = "";
    Mat color;
    bool new_img = false;
    for ( int i=1800; i<deep_data.size(); i++ )
    {
        if(i%20 == 0) {
            cout << "****** loop " << i << " ******" << endl;
        }
        if(color_image_files[i] != last_img_name){
            color = cv::imread ( color_image_files[i] );
            last_img_name = color_image_files[i];
            new_img = true;
        }

        if ( color.data==nullptr)
            break;
        Eigen::Vector4d pixel_C(deep_data[i], 0.0, 0.0, 1.0);
        Eigen::Vector4d pixel_W = poses[i].matrix() * pixel_C;

        int pixelIndex = (int(fy)*color.step + int(fx)*color.channels());
        Vector3d pixel_Color(color.data[pixelIndex], color.data[pixelIndex+1], color.data[pixelIndex+2]);

        env_rebuilder::LidarPoint::Ptr lp = env_rebuilder::LidarPoint::createLidarPoint(pixel_W, pixel_Color);
        pointcloud->insertLidarPoint(lp);

        if(new_img){
            cout << "----------Processing img----------";
            env_rebuilder::Frame::Ptr fp = env_rebuilder::Frame::createFrame();

            fp->T_c_w_ = poses[i];
            fp->camera_ = cp;
            fp->img_add_ = color_image_files[i];
            fp->c_point_ = lp;
            fp->is_key_frame_ = true;

            // create frame & feature & feature_map
            // 1st: Feature
            // 2nd: Feature_map
            // 3rd: Frame

            Mat descriptors;
            vector<cv::KeyPoint> keypoints;

            extractORBFeature(color, keypoints, descriptors);
//            cout << "key point number: " << keypoints.size() << endl;
//            cout << "Discriptor's element size is: " << descriptors.elemSize() << endl;
//            cout << "Discriptor's element size1 is: " << descriptors.elemSize1() << endl;

            for(int k = 0; k < keypoints.size(); k++){
                Vector3d pt_c = cp->pixel2camera(Vector2d(keypoints[k].pt.x, keypoints[k].pt.y));
                pt_c.normalize();
                pt_c *= 80.0;
                Vector3d pt_w = poses[i] * pt_c;

                Mat r = descriptors.row(k);

                env_rebuilder::Feature::Ptr featureptr =
                        env_rebuilder::Feature::createCornerFeature(pt_w, descriptors.row(k), fp.get());

                /* test prog
                Mat des1 = (cv::Mat_<int>(1,12)<<0,1,5,1,7,7,6,9,8,7,5,4);
                Mat des2 = (cv::Mat_<int >(1,12)<<0,1,5,1,8,7,6,9,8,7,5,4);
                int dist = feature_map->DescriptorDistance(des1, des2);
                cout << "Hamming Distance is :" << dist << endl;
                cout << "Discriptor's element size is: " << r.elemSize() << endl;
                cout << "Discriptor's element size1 is: " << r.elemSize1() << endl;
                cout << "des2's element size is: " << des2.elemSize() << endl;

                // !test prog*/

                feature_map->insertFeature(featureptr);
                fp->features_.push_back(featureptr);

            }
            frames.push_back(fp);
            new_img = false;

            if(frames.size() >= 2){
                feature_map->merge_features(10);
            }
        }

        if(feature_map->features_.size() > 50){
            feature_map->merge_features(10);
        }

        if(i > 1400){
            cout << "feature number: " << feature_map->features_.size() << endl;
            cout << "frame number:" << frames.size() << endl;
            cout << "-------------------" << endl;
        }

    }

    int count = pointcloud->lidar_points_.size();
    while(count--){
        env_rebuilder::LidarPoint::Ptr lptr = pointcloud->iterator_getNextPoint();
        PointT point;
        point.x = lptr->pos_.x();
        point.y = lptr->pos_.y();
        point.z = lptr->pos_.z();
        point.b = lptr->color_.x();
        point.g = lptr->color_.y();
        point.r = lptr->color_.z();

        pc->points.push_back(point);
    }

    count = feature_map->features_.size();
    while(count--){
        env_rebuilder::Feature::Ptr fptr = feature_map->iterator_getNextFeature();
        PointT point;
        point.x = fptr->pos1_.x();
        point.y = fptr->pos1_.y();
        point.z = fptr->pos1_.z();
        point.b = 0;
        point.g = 0;
        point.r = 255;

        pc->points.push_back(point);
    }

    pcl::io::savePCDFileBinary("/home/kevin/catkin_workspace/src/env_rebuilder/result/simple_mapV3.pcd", *pc );
    return 0;
}