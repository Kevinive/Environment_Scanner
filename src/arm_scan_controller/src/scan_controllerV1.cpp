//
// Created by kevin on 2020/3/20.
//

#include "arm_scan_controller/scan_controllerV1.h"

using namespace std;

Scan_controller::Scan_controller(){
    scan_finished = false;
    yaw.data = 0.0;
    pitch.data = 1;

    packedData.pose.position.x = packedData.pose.position.y = packedData.pose.position.z = 0.0;
    packedData.pose.orientation.x = packedData.pose.orientation.y = packedData.pose.orientation.z = packedData.pose.orientation.w = 0.0;

    packedData.lidarData = 0.0;

    imgIndex = 0;

    fileAdd = imgAdd = DATA_PREFIX;
    fileAdd += "/img_seq_pose.txt";
    imgAdd += "/img/";

    cout << "data file: " << fileAdd << endl;
    cout << "img directory: " << imgAdd << endl;

    outfile.open(fileAdd, ios::out | ios::trunc);

    scanPackage_pub = n_.advertise<arm_scan_controller::Scanpackage>("packed_scanData", 1);
    yaw_cmd_pub = n_.advertise<std_msgs::Float64>("arm_v2/yaw_joint_controller/command", 1);
    pitch_cmd_pub = n_.advertise<std_msgs::Float64>("arm_v2/pitch_joint_controller/command", 1);

    pose_sub = n_.subscribe("pose_estimated", 1, &Scan_controller::pose_sub_Callback, this);
    image_sub = n_.subscribe("camera/image_raw", 1, &Scan_controller::image_sub_Callback, this);
    lidar_sub = n_.subscribe("blocklaser_scan", 1, &Scan_controller::lidar_sub_Callback, this);

}

Scan_controller::~Scan_controller(){
    if(outfile.is_open()) {
        outfile.close();
    }
}

void Scan_controller::pose_sub_Callback(const geometry_msgs::Pose::ConstPtr& msg){
    packedData.pose = *msg;
}

void Scan_controller::image_sub_Callback(const sensor_msgs::Image::ConstPtr& msg){
    img = *msg;
}

void Scan_controller::lidar_sub_Callback(const sensor_msgs::PointCloud::ConstPtr& msg){
    sensor_msgs::PointCloud pc = *msg;
    geometry_msgs::Point32 pt = pc.points[(pc.points.size()+1)/2];
    packedData.lidarData = sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
}

void Scan_controller::next_step(){
    if(!scan_finished){
        // publish Packed Datas
        packedData.img_data = img.data;
        scanPackage_pub.publish(packedData);

        // save Packed Datas
        string imgName = to_string(imgIndex);
        imgName += ".png";

        outfile << imgName << " "
                << packedData.pose.position.x << " "
                << packedData.pose.position.y << " "
                << packedData.pose.position.z << " "
                << packedData.pose.orientation.w << " "
                << packedData.pose.orientation.x << " "
                << packedData.pose.orientation.y << " "
                << packedData.pose.orientation.z << " "
                << packedData.lidarData << endl;

        // save Img
        string imgPath = imgAdd;
        imgPath += imgName;
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::imwrite(imgPath, cv_ptr->image);

        ++imgIndex;

        //turn to next direction
        yaw.data += YAW_STEP;
        pitch.data += PITCH_STEP;
        if(pitch.data > PITCH_LIMIT){
            scan_finished = true;
        }

        yaw_cmd_pub.publish(yaw);
        pitch_cmd_pub.publish(pitch);

    }
}

float Scan_controller::getPitch(){
    return pitch.data;
}
float Scan_controller::getYaw(){
    return yaw.data;
}
bool Scan_controller::isScanFinished(){
    return scan_finished;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "scan_controller");
    Scan_controller scan_controller;
    ROS_INFO("init finished!");

    ros::Rate loop_rate(1);

    int count = 0;
    while(ros::ok() && !scan_controller.isScanFinished()){
        loop_rate.sleep();
        ros::spinOnce();
        scan_controller.next_step();

        if(count%10 == 0){
            std::cout << "Current Pitch: " << scan_controller.getPitch() <<
            " Yaw: " << scan_controller.getYaw() << std::endl;
        }

        ++count;
    }

    return 0;
}
