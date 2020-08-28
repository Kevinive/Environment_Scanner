//
// Created by kevin on 2020/3/20.
//

#include "arm_scan_controller/scan_controllerV3.h"

using namespace std;
using namespace Eigen;

Scan_controller::Scan_controller(){
    yaw.data = 0.0;
    pitch.data = 0.0;
    staticFlag = true;
    autoScanState = true;

    packedData.pose.position.x = packedData.pose.position.y = packedData.pose.position.z = 0.0;
    packedData.pose.orientation.x = packedData.pose.orientation.y = packedData.pose.orientation.z = packedData.pose.orientation.w = 0.0;
    lastPose.position.x = lastPose.position.y = lastPose.position.z = 0.0;
    lastPose.orientation.x = lastPose.orientation.y = lastPose.orientation.z = lastPose.orientation.w = 0.0;

    packedData.lidarData = 0.0;

    imgIndex = 0;

    posefileAdd = imgfileAdd = imgAdd = DATA_PREFIX;
    posefileAdd += "/pose.txt";
    imgfileAdd += "/img_seq.txt";
    imgAdd += "/img/";

    cout << "pose data file: " << posefileAdd << endl;
    cout << "img data file: " << imgfileAdd << endl;
    cout << "img directory: " << imgAdd << endl;

    outPosefile.open(posefileAdd, ios::out | ios::trunc);
    outImgfile.open(imgfileAdd, ios::out | ios::trunc);

    scanPackage_pub = n_.advertise<arm_scan_controller::Scanpackage>("packed_scanData", 10);
    yaw_cmd_pub = n_.advertise<std_msgs::Float64>("arm_v2/yaw_joint_controller/command", 1);
    pitch_cmd_pub = n_.advertise<std_msgs::Float64>("arm_v2/pitch_joint_controller/command", 1);
    autoscan_state_pub = n_.advertise<std_msgs::Bool>("arm_v2/autoscan_state", 1);

    pose_sub = n_.subscribe("pose_estimated", 10, &Scan_controller::pose_sub_Callback, this);
    image_sub = n_.subscribe("camera/image_raw", 10, &Scan_controller::image_sub_Callback, this);
    lidar_sub = n_.subscribe("blocklaser_scan", 10, &Scan_controller::lidar_sub_Callback, this);
    autoscan_cmd_sub = n_.subscribe("cmd_autoscan", 1, &Scan_controller::autoscan_sub_Callback, this);

    Eigen::Matrix4d T_Cen_W;
    T_Cen_W <<  0, 1, 0, 0,
                -1, 0, 0, 0,
                0, 0, 1, 0.2755,
                0, 0, 0, 1;

    T_W_Cen = T_Cen_W.inverse();

    autoscan_timer = n_.createTimer(ros::Rate(3), &Scan_controller::autoscan_Callback, this);
    autoscan_state_pub_timer = n_.createTimer(ros::Rate(1), &Scan_controller::autoscan_state_pub_Callback, this);
    start_autoscan();
}

Scan_controller::~Scan_controller(){
    if(outPosefile.is_open()) {
        outPosefile.close();
    }
    if(outImgfile.is_open()) {
        outImgfile.close();
    }
}


void Scan_controller::autoscan_sub_Callback(const std_msgs::Bool::ConstPtr& msg){
    if(msg->data){
        start_autoscan();
    }else{
        stop_autoscan();
    }
}

void Scan_controller::autoscan_state_pub_Callback(const ros::TimerEvent&){
    std_msgs::Bool state;
    state.data = autoScanState;
    autoscan_state_pub.publish(state);
}

void Scan_controller::start_autoscan(){
    stop_service();
    autoScanState = true;
    setPitch(PITCH_SCAN_START);
    setYaw(YAW_SCAN_START);
    autoscan_timer.start();
}

void Scan_controller::stop_autoscan(){
    autoScanState = false;
    setPitch(0.0);
    setYaw(0.0);
    autoscan_timer.stop();
    start_service();
}


void Scan_controller::autoscan_Callback(const ros::TimerEvent&){
    static int yaw_count = 0;
    static int img_pitch_count = 0;
    if(autoScanState){

        if(getPitch() > PITCH_LIMIT){
            stop_autoscan();
            yaw_count = 0;
            img_pitch_count = 0;
            return;
        }

        pub_state();
        save_pose_data();

        if(img_pitch_count%IMG_PITCH_STEP_COUNT == 0 && yaw_count%IMG_YAW_STEP_COUNT == 0){
            save_img_data();
        }

        if(yaw_count > 360.0 / YAW_STEP_DEG) {
            setPitch(getPitch() + PITCH_STEP);
            yaw_count = 0;
            img_pitch_count ++;
        }

        setYaw(getYaw() + YAW_STEP);
        yaw_count++;
    }
}

void Scan_controller::pose_sub_Callback(const geometry_msgs::Pose::ConstPtr& msg){
    lastPose = packedData.pose;
    packedData.pose = *msg;

    if(!staticFlag){
        Eigen::Isometry3d R1 = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d R2 = Eigen::Isometry3d::Identity();
        R1.rotate(Eigen::Quaterniond(lastPose.orientation.w,lastPose.orientation.x,lastPose.orientation.y,lastPose.orientation.z));
        R2.rotate(Eigen::Quaterniond(packedData.pose.orientation.w,packedData.pose.orientation.x,
                  packedData.pose.orientation.y,packedData.pose.orientation.z));
        R1.pretranslate(Eigen::Vector3d(lastPose.position.x, lastPose.position.y, lastPose.position.z));
        R2.pretranslate(Eigen::Vector3d(packedData.pose.position.x, packedData.pose.position.y, packedData.pose.position.z));

        Eigen::Matrix4d R12 = (R1.inverse() * R2).matrix();

        if(abs(R12(0, 3)) < 1e-5 && abs(R12(1, 3)) < 1e-5 && abs(R12(2, 3)) < 1e-5){
            staticFlag = true;
            ROS_INFO("Arm is static now!");
        }/*else{
            cout << R12(0, 3) << " " << R12(1, 3) << " " << R12(2, 3) << endl;
        }*/
    }
}

void Scan_controller::image_sub_Callback(const sensor_msgs::Image::ConstPtr& msg){
    img = *msg;
}

// TODO 确认雷达扫描点和图像的对应关系（是否对应图像中心像素）
void Scan_controller::lidar_sub_Callback(const sensor_msgs::PointCloud::ConstPtr& msg){
    sensor_msgs::PointCloud pc = *msg;
    geometry_msgs::Point32 pt = pc.points[(pc.points.size()+1)/2];
    packedData.lidarData = sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
}

void Scan_controller::pub_state(){
    packedData.img_data = img.data;
    packedData.img_step = img.step;
    packedData.img_height = img.height;
    packedData.img_width = img.width;
    packedData.img_encoding = img.encoding;
    packedData.img_is_bigendian = img.is_bigendian;

    scanPackage_pub.publish(packedData);
}

void Scan_controller::save_pose_data(){
    outPosefile
            << packedData.pose.position.x << " "
            << packedData.pose.position.y << " "
            << packedData.pose.position.z << " "
            << packedData.pose.orientation.w << " "
            << packedData.pose.orientation.x << " "
            << packedData.pose.orientation.y << " "
            << packedData.pose.orientation.z << " "
            << packedData.lidarData << endl;
}

void Scan_controller::save_img_data(){

    // save Packed Datas
    string imgName = to_string(imgIndex);
    imgName += ".png";

    // save Img
    ++imgIndex;
    string imgPath = imgAdd;
    imgName = to_string(imgIndex);
    imgName += ".png";
    imgPath += imgName;
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::imwrite(imgPath, cv_ptr->image);

    outImgfile
            << imgName << " "
            << packedData.pose.position.x << " "
            << packedData.pose.position.y << " "
            << packedData.pose.position.z << " "
            << packedData.pose.orientation.w << " "
            << packedData.pose.orientation.x << " "
            << packedData.pose.orientation.y << " "
            << packedData.pose.orientation.z << " "
            << endl;
}

void Scan_controller::start_service() {
    scanPose_service = n_.advertiseService("scan_pose_XYZ", &Scan_controller::scanpos_Callback, this);
    ROS_INFO("Ready to provide scan_pose_XYZ service.");
}

void Scan_controller::stop_service() {
    scanPose_service.shutdown();
    ROS_INFO("Stop providing scan_pose_XYZ service.");
}

bool Scan_controller::scanpos_Callback(arm_scan_controller::ScanPoseXYZ::Request  &req,
                      arm_scan_controller::ScanPoseXYZ::Response &res)
{
    Vector4d p_W(req.x, req.y, req.z, 1);
    Vector4d p_Cen = T_W_Cen * p_W;
    double alpha, beta, affin;

    if(sqrt(p_Cen.x()*p_Cen.x() + p_Cen.y()*p_Cen.y() + p_Cen.z()*p_Cen.z()) < 0.2) return false;

    if(abs(p_Cen.x()) < 0.0001){
        if(p_Cen.y() > 0) alpha = M_PI/2;
        else alpha = -M_PI/2;
    }else{
        alpha = atan(p_Cen.y()/p_Cen.x());
        if(p_Cen.x() < 0.0){
            if(p_Cen.y() < 0.0){
                alpha -= M_PI;
            }else{
                alpha += M_PI;
            }
        }
    }

    affin = sqrt(p_Cen.y() * p_Cen.y() + p_Cen.x() * p_Cen.x());
    beta = M_PI/2 - atan(p_Cen.z() / affin);

    if(beta > PITCH_LIMIT) return false;

    setPitch(beta);
    setYaw(alpha);
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    while(!staticFlag){
        loop_rate.sleep();
        ros::spinOnce();
    }
    res.distance = packedData.lidarData;
    res.pose = packedData.pose;
    res.img_data = img.data;
    return true;
}

double Scan_controller::getPitch(){
    return pitch.data;
}
double Scan_controller::getYaw(){
    return yaw.data;
}

bool Scan_controller::setPitch(double val){
    pitch.data = val;
    pitch_cmd_pub.publish(pitch);
    staticFlag = false;
    ROS_INFO("Arm is moving now!");
    return true;
}
bool Scan_controller::setYaw(double val){
    yaw.data = val;
    yaw_cmd_pub.publish(yaw);
    staticFlag = false;
    ROS_INFO("Arm is moving now!");
    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "scan_controller");
    Scan_controller scan_controller;
    ROS_INFO("init finished!");

    ros::spin();

    return 0;
}
