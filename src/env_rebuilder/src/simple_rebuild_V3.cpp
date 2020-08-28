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

void generate_glData(env_rebuilder::Surface::Ptr& spt);
/*
 * main function
 */
int main( int argc, char** argv ){

    // 1 读取数据，初始化系统
    env_rebuilder::Config::setParameterFile ( PARAM_FILE );

    string dataset_dir = env_rebuilder::Config::get<string> ( "dataset_dir" );
    cout << "dataset: "<< dataset_dir << endl;
    ifstream pose_fin ( dataset_dir+"/pose.txt" );
    ifstream img_fin ( dataset_dir+"/img_seq.txt" );
    if ( !pose_fin || !img_fin )
    {
        cout<<"please generate the associate file called img_seq.txt/pose.txt!"<<endl;
        return 1;
    }

    // 初始化源数据容器
    vector< string > color_image_files;
    vector< SE3 > image_poses;
    vector< double > deep_data;
    vector< SE3 > lidar_poses;
    env_rebuilder::Camera::Ptr cp = env_rebuilder::Camera::Ptr(new env_rebuilder::Camera());

    // 读入激光雷达数据
    while ( !pose_fin.eof() )
    {
        double data[8];
        for ( double& d:data ) pose_fin>>d;
        deep_data.push_back(data[7]);
        lidar_poses.push_back(SE3 (
                Eigen::Quaterniond ( data[3], data[4], data[5], data[6] ),
                Vector3d ( data[0], data[1], data[2] )
        )
        );

        if ( !pose_fin.good() )
            break;
    }
    pose_fin.close();

    while ( !img_fin.eof() )
    {
        string image;
        img_fin>>image;
        double data[7];
        for ( double& d:data ) img_fin>>d;

        color_image_files.push_back ( dataset_dir+"/img/"+image );

        image_poses.push_back(SE3 (
                Eigen::Quaterniond ( data[3], data[4], data[5], data[6] ),
                Vector3d ( data[0], data[1], data[2] )
                        )
        );

        if ( img_fin.good() == false )
            break;
    }
    img_fin.close();

    cout<<"read total "<<deep_data.size() <<" lidar entries."<<endl;
    cout<<"read total "<<image_poses.size() <<" image entries."<<endl;

    // 2 初始化处理后的数据容器
//    env_rebuilder::FeatureMap::Ptr feature_map(new env_rebuilder::FeatureMap);
    std::vector<env_rebuilder::Frame::Ptr> frames;
    PointCloud::Ptr pc( new PointCloud );

    Mat color;
//    int merge_Num = 100;

    // 3 绘制点云
    vector<env_rebuilder::LidarPoint::Ptr> allPoint;

    for ( int i=0; i<deep_data.size(); i++ )
    {
        // 绘制激光点云
        if(i%100 == 0) {
            cout << "****** loop " << i << " ******" << endl;
        }

        bool isValid = true;
        if(deep_data[i] > 8.0 || deep_data[i] < 0.1){
            deep_data[i] = 8.0;
            isValid = false;
        }
        Eigen::Vector4d pixel_C(deep_data[i], 0.0, 0.0, 1.0);
        Eigen::Vector4d pixel_W = lidar_poses[i].matrix() * pixel_C;

        Vector3d pixel_Color(255, 255, 255);

        env_rebuilder::LidarPoint::Ptr lp = env_rebuilder::LidarPoint::createLidarPoint(pixel_W, pixel_Color, isValid);
        allPoint.push_back(lp);

    }

    env_rebuilder::PointCloud::Ptr pointcloud =
            env_rebuilder::PointCloud::createPointCloud(allPoint, 1.0);


    // 4 初始化帧对象
    vector<env_rebuilder::Frame::Ptr> allFrame;

    for ( int i=0; i<color_image_files.size(); i++ )
    {
        // 绘制激光点云
        if(i%10 == 0) {
            cout << "****** loop " << i << " ******" << endl;
        }


        env_rebuilder::Frame::Ptr fp = env_rebuilder::Frame::createFrame();
        fp->T_c_w_= image_poses[i];
        fp->camera_ = cp;
        fp->color_ = cv::imread ( color_image_files[i] );;
        fp->img_add_ = color_image_files[i];

        allFrame.push_back(fp);

    }

    // 5 生成面元
    env_rebuilder::Surface surface(pointcloud);
    surface.rebuildV2(allPoint, allFrame);

    /*
    // 验证纹理和点的对应关系
    env_rebuilder::Frame::Ptr templatefp = allFrame[15];
    Mat templateImg = templatefp->color_;


    std::unordered_map<unsigned long, env_rebuilder::SurfaceUnit::Ptr >::iterator tmpit;

    for(tmpit=surface.surfaceunits_.begin(); tmpit != surface.surfaceunits_.end(); tmpit++){
        env_rebuilder::SurfaceUnit::Ptr su = tmpit->second;

        if(su->texture_frame_->id_ != templatefp->id_) continue;

        cv::circle(templateImg, cv::Point(su->texture_pixel_point_[0].x(),su->texture_pixel_point_[0].y()), 1,cv::Scalar(0, 0, 255));
        cv::circle(templateImg, cv::Point(su->texture_pixel_point_[1].x(),su->texture_pixel_point_[1].y()), 1,cv::Scalar(0, 0, 255));
        cv::circle(templateImg, cv::Point(su->texture_pixel_point_[2].x(),su->texture_pixel_point_[2].y()), 1,cv::Scalar(0, 0, 255));
    }

    cv::imshow("templateImg", templateImg);
    cv::waitKey(0);
    */


    // 保存PCD文件
    //pcl::io::savePCDFileASCII("/home/kevin/catkin_workspace/src/env_rebuilder/result/simple_mapV3.pcd", *pc );

    // 保存自定义文件
    string pc_result_file = env_rebuilder::Config::get<string> ( "pointcloud_result" );
    ofstream pc_fout(pc_result_file);
    cout << allPoint.size() << endl;
    pc_fout << allPoint.size() << endl;
    for(int i = 0; i < allPoint.size(); i++){
        pc_fout << allPoint[i]->id_ << " " << allPoint[i]->pos_.x() << " " << allPoint[i]->pos_.y() << " " << allPoint[i]->pos_.z() << " "
            << "255" << " " << "0" << " " << "0" << endl;
    }
    pc_fout.close();

    string mesh_result_file = env_rebuilder::Config::get<string> ( "mesh_result" );
    ofstream mesh_fout(mesh_result_file);
    mesh_fout << surface.surfaceunits_.size() << endl;

    /*for(unsigned long i=0; i<surface.surfaceunits_.size(); i++){
        env_rebuilder::SurfaceUnit::Ptr su = surface.surfaceunits_[i];

        mesh_fout << i << " " << su->points_[0]->id_ << " " << su->points_[1]->id_ << " " << su->points_[2]->id_ << endl;
    }*/

    std::unordered_map<unsigned long, env_rebuilder::SurfaceUnit::Ptr >::iterator it;
    for(it=surface.surfaceunits_.begin(); it != surface.surfaceunits_.end(); it++){
        env_rebuilder::SurfaceUnit::Ptr su = it->second;

        mesh_fout << su->id_ << " " << su->points_[0]->id_ << " " << su->points_[1]->id_ << " " << su->points_[2]->id_ << " "
                << su->texture_pixel_point_[0].x() << " " << su->texture_pixel_point_[0].y() << " "
                << su->texture_pixel_point_[1].x() << " " << su->texture_pixel_point_[1].y() << " "
                << su->texture_pixel_point_[2].x() << " " << su->texture_pixel_point_[2].y() << " "
                << su->texture_frame_->img_add_ << endl;
    }

    mesh_fout.close();

    // 生成OPENGL用的渲染文件
    // 遍历Frame,以Frame为基准搜索对应点和面元
    string gldir = env_rebuilder::Config::get<string> ( "gldata_dir" );
    ofstream texture_fout(gldir+"info.texture");
    for(env_rebuilder::Frame::Ptr fp : allFrame){

        // 初始化子数据文件输出管道
        string texpointDir = gldir + to_string(fp->id_) + ".texPoint";
        string meshindexDir = gldir + to_string(fp->id_) + ".meshIndex";

        ofstream texpoint_fout(texpointDir);
        ofstream meshindex_fout(meshindexDir);

        // 初始化子集
        vector<env_rebuilder::SurfaceUnit::Ptr> sub_supts;
        unordered_map<unsigned long, env_rebuilder::LidarPoint::Ptr> sub_points;
        unsigned long subIndex = 0;

        // 遍历面元，对比其texture_frame是否为当前Frame
        std::unordered_map<unsigned long, env_rebuilder::SurfaceUnit::Ptr >::iterator tmpit;
        for(tmpit=surface.surfaceunits_.begin(); tmpit != surface.surfaceunits_.end(); tmpit++){
            env_rebuilder::SurfaceUnit::Ptr su = tmpit->second;
            if(su->texture_frame_->id_ != fp->id_) continue;
            sub_supts.push_back(su);
        }

        // 如果无面元属于当前Frame 则关闭文件后跳过
        if(sub_supts.empty()){
            texpoint_fout.close();
            meshindex_fout.close();
            continue;
        }

        // 若有面元属于当前Frame 则再次遍历面元，将遇到的点按照新编码写入文件
        // 利用unordered_map管理新的点集
        for(env_rebuilder::SurfaceUnit::Ptr supt : sub_supts){

            // 对面元的角点进行遍历

            for(int i=0; i<3; i++){
                // 若此曾碰到过，则跳过；否则新建此点并加入列表
                if(sub_points.end() == sub_points.find(supt->points_[i]->id_)){
                    env_rebuilder::LidarPoint::Ptr newLptr =
                            env_rebuilder::LidarPoint::createLidarPoint(supt->points_[i]->pos_,Vector3d(0,0,0), true);
                    newLptr->id_ = subIndex;
                    texpoint_fout << newLptr->id_ << " "
                            << newLptr->pos_.x() << " " << newLptr->pos_.y() << " "<< newLptr->pos_.z() << " "
                            << supt->texture_pixel_point_[i].x() << " " << supt->texture_pixel_point_[i].y() << endl;
                    subIndex++;
                    sub_points.insert(make_pair(supt->points_[i]->id_, newLptr));
                }
            }

            meshindex_fout << supt->id_ << " "
            << sub_points[supt->points_[0]->id_]->id_ << " "
            << sub_points[supt->points_[1]->id_]->id_ << " "
            << sub_points[supt->points_[2]->id_]->id_ << endl;

        }

        meshindex_fout.close();
        texpoint_fout.close();

        // 输出总文件
        texture_fout << fp->id_ << " " << fp->img_add_ << " " << fp->color_.cols << " " << fp->color_.rows << " "
                  << texpointDir << " " << sub_points.size() << " "
                  << meshindexDir << " " << sub_supts.size() << endl;
    }
    texture_fout.close();

    return 0;
}

