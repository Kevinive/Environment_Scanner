
#include "pose_estimator/pose_estimatorV1.h"

Pose_estimator::Pose_estimator(){
    vo_pose.position.x = vo_pose.position.y = vo_pose.position.z = 0.0;
    mechanical_tf.translation.x = mechanical_tf.translation.y = mechanical_tf.translation.z = 0.0;
    mechanical_tf.rotation.x = mechanical_tf.rotation.y = mechanical_tf.rotation.z = 0.0;


    pose_estimate_pub = n_.advertise<geometry_msgs::Pose>("pose_estimated", 1);

    //imu_sub = n_.subscribe("imu", 1, &Pose_estimator::imu_sub_Callback, this);
    //vo_pose_sub = n_.subscribe("vo_pose", 1, &Pose_estimator::vo_pose_sub_Callback, this);
    //mechanical_sub = n_.subscribe("tf", 1, &Pose_estimator::mechanical_sub_Callback, this);

    pub_timer = n_.createTimer(ros::Rate(10), &Pose_estimator::pub_estimated_pose, this);
}

void Pose_estimator::imu_sub_Callback(const sensor_msgs::Imu::ConstPtr& msg){
    return;
}
void Pose_estimator::vo_pose_sub_Callback(const geometry_msgs::Pose::ConstPtr& msg){
    return;
}
void Pose_estimator::mechanical_sub_Callback(const tf2_msgs::TFMessage::ConstPtr& msg){
    return;
}


void Pose_estimator::pub_estimated_pose(const ros::TimerEvent&){
    tf::StampedTransform transform;
    try{
        listener.lookupTransform("/odom", "/Camera", ros::Time(0), transform);
    }catch (tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
        return;
    }

    tf::Vector3 origin = transform.getOrigin();
    tf::Quaternion qua = transform.getRotation();

    estimated_pose.position.x = origin.x();
    estimated_pose.position.y = origin.y();
    estimated_pose.position.z = origin.z();

    estimated_pose.orientation.w = qua.w();
    estimated_pose.orientation.x = qua.x();
    estimated_pose.orientation.y = qua.y();
    estimated_pose.orientation.z = qua.z();

    pose_estimate_pub.publish(estimated_pose);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pose_estimator");

    //ros::service::waitForService("spawn");gs

    Pose_estimator pose_estimator;

    ROS_INFO("init finished!");
    ros::spin();

    return 0;
}