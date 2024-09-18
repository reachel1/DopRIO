#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <thread>
#include <ros/callback_queue.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <sstream> 
#include <fstream>
#include <std_msgs/Float32.h>
#include <stdlib.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/filesystem.hpp>



std::ofstream outfile_gt;
double last_time_gt = 0;
void record_gt(const nav_msgs::Odometry::ConstPtr &msg);
Eigen::Matrix4d Inv_se3(const Eigen::Matrix4d &T);

Eigen::Matrix4d TBtoRa, TBtoL, TRatoG1, TRatoG2, TBtoI;

#define PI 3.14159265358979323846   // pi

int main(int argc, char **argv) {
    //Launch our ros node
    ros::init(argc, argv, "get_gt_from_coloradar");
    ros::NodeHandle nh;
    //get params
    if(argc < 4){
        ROS_ERROR("Please input --- rosrun rise get_gt_from_coloradar bag_path gt_topic gt.txt.path");
        std::exit(EXIT_FAILURE);
    }
                    
    //check file path
    std::string bag_path = argv[1];
    std::string gt_topic = argv[2];
    std::string gt_txtpath = argv[3];

    boost::filesystem::path dir_gt(gt_txtpath);
    if (boost::filesystem::create_directories(dir_gt.parent_path())) {
        ROS_INFO("Created folder path to output file.");
        ROS_INFO("Path: %s", dir_gt.parent_path().c_str());
    }
    // If it exists, then delete it
    if (boost::filesystem::exists(gt_txtpath)) {
        ROS_INFO("Output file exists, deleting old file....\n");
        boost::filesystem::remove(gt_txtpath);
    }

    // Open the bag
    rosbag::Bag bag;
    bag.open(bag_path,rosbag::bagmode::Read);
    rosbag::View view_bag;
    view_bag.addQuery(bag);

    // Output imu and gt to stream file
    outfile_gt.open(gt_txtpath);
    if (outfile_gt.fail()) {
        ROS_ERROR("Unable to open output file imu!!");
        std::exit(EXIT_FAILURE);
    }

    std::cout<<"gt: "<<gt_topic<<std::endl;

    // set ext
    Eigen::Vector3d pBtoRa(0.03, 0.12, -0.09);
    Eigen::Quaterniond qBtoRa(0.707388269167, 0.0, 0.0, 0.706825181105);
    Eigen::Matrix3d rBtoRa = qBtoRa.toRotationMatrix();
    TBtoRa = Eigen::Matrix4d::Identity();
    TBtoRa.block<3, 3>(0, 0) = rBtoRa;
    TBtoRa.block<3, 1>(0, 3) = pBtoRa;

    Eigen::Vector3d pBtoI(0.0, 0.0, 0.0);
    Eigen::Quaterniond qBtoI(0.00020953429822, 0.707105451466, 0.707108048814, 0.000209535067884);
    Eigen::Matrix3d rBtoI = qBtoI.toRotationMatrix();
    TBtoI = Eigen::Matrix4d::Identity();
    TBtoI.block<3, 3>(0, 0) = rBtoI;
    TBtoI.block<3, 1>(0, 3) = pBtoI;

    Eigen::Matrix4d TRtoI = TBtoI * Inv_se3(TBtoRa);
    std::cout<<"Trtoi"<<std::endl;
    std::cout<<TRtoI<<std::endl;

    Eigen::Vector3d pBtoL(-0.075, -0.02, 0.03618);
    Eigen::Quaterniond qBtoL(-0.692536998563, 0.0, 0.0, 0.721382357437);
    Eigen::Matrix3d rBtoL = qBtoL.toRotationMatrix();
    TBtoL = Eigen::Matrix4d::Identity();
    TBtoL.block<3, 3>(0, 0) = rBtoL;
    TBtoL.block<3, 1>(0, 3) = pBtoL;

    Eigen::Matrix4d TRatoL = TBtoL * Inv_se3(TBtoRa);
    std::cout<<"Trtol"<<std::endl;
    std::cout<<TRatoL<<std::endl;


    Eigen::Vector3d pRatoG1(-0.02098879114971743, 0.14319756083366733, 0.10664148398024365);
    Eigen::Quaterniond qRatoG1(0.015068881553245208, 0.0019359795017721883, 0.020513743766898216, 0.9996741304589057);
    Eigen::Matrix3d rRatoG1 = qRatoG1.toRotationMatrix();
    TRatoG1 = Eigen::Matrix4d::Identity();
    TRatoG1.block<3, 3>(0, 0) = rRatoG1;
    TRatoG1.block<3, 1>(0, 3) = pRatoG1;

    Eigen::Vector3d pRatoG2(0.024454327286, 0.120848299597, -0.090541007208);
    Eigen::Quaterniond qRatoG2(-0.000256135136, 0.030956805547, -0.004620331700, 0.999510011516);
    Eigen::Matrix3d rRatoG2 = qRatoG2.toRotationMatrix();
    TRatoG2 = Eigen::Matrix4d::Identity();
    TRatoG2.block<3, 3>(0, 0) = rRatoG2;
    TRatoG2.block<3, 1>(0, 3) = pRatoG2;

    // View imu and gt in bag
    std::shared_ptr<rosbag::View> view_gt;
    rosbag::View::iterator view_gt_iter;
    nav_msgs::Odometry::ConstPtr msg_gt;
    view_gt = std::make_shared<rosbag::View>(bag, rosbag::TopicQuery(gt_topic),view_bag.getBeginTime(),view_bag.getEndTime());
    view_gt_iter = view_gt->begin();

    while(ros::ok()){
        if(view_gt_iter == view_gt->end())break;
        msg_gt = view_gt_iter->instantiate<nav_msgs::Odometry>();
        record_gt(msg_gt);
        view_gt_iter++;
    }

    bag.close();
    outfile_gt.close();

}


void record_gt(const nav_msgs::Odometry::ConstPtr &msg){
    if(msg->header.stamp.toSec() == last_time_gt)return;
    else last_time_gt =  msg->header.stamp.toSec();

    const auto& pos = msg->pose.pose.position;
    const auto& ori = msg->pose.pose.orientation;
    Eigen::Vector3d pLtoG(pos.x, pos.y, pos.z);
    Eigen::Quaterniond qLtoG(ori.w, ori.x, ori.y, ori.z);

    Eigen::Matrix3d rLtoG = qLtoG.toRotationMatrix();

    Eigen::Matrix4d TLtoG = Eigen::Matrix4d::Identity();
    TLtoG.block<3, 3>(0, 0) = rLtoG;
    TLtoG.block<3, 1>(0, 3) = pLtoG;
    Eigen::Matrix4d TLtoG1 = TLtoG;

    Eigen::Matrix4d TRatoG = TLtoG1 * TBtoL * Inv_se3(TBtoRa);

    Eigen::Matrix3d Rt = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d RR = Eigen::Matrix3d::Identity();


    Eigen::Vector3d pRatoG = Rt * TRatoG.block(0,3,3,1);
    Eigen::Matrix3d rRatoG = RR * TRatoG.block(0,0,3,3);
    Eigen::Quaterniond qRatoG(rRatoG);

    outfile_gt.precision(9);
    outfile_gt.setf(std::ios::fixed, std::ios::floatfield);
    //outfile_gt <<msg->header.stamp.toSec()<<" "<< pRatoG.x() << " " << pRatoG.y() << " " << pRatoG.z() << " "
        //<< qRatoG.x() << " " << qRatoG.y() << " " << qRatoG.z() << " " << qRatoG.w()<<std::endl;
    outfile_gt <<msg->header.stamp.toSec()<<" "<< pRatoG.y() << " " <<  -pRatoG.x()<< " " << pRatoG.z() << " "
        << qRatoG.x() << " " << qRatoG.y() << " " << -qRatoG.w() << " " << qRatoG.z()<<std::endl;
    // outfile_gt <<msg->header.stamp.toSec()<<" "<< pLtoG.x() << " " <<  pLtoG.y()<< " " << pLtoG.z() << " "
    //     << qLtoG.x() << " " << qLtoG.y() << " " << qLtoG.z() << " " << qLtoG.w()<<std::endl;
}

Eigen::Matrix4d Inv_se3(const Eigen::Matrix4d &T) {
  Eigen::Matrix4d Tinv = Eigen::Matrix4d::Identity();
  Tinv.block(0, 0, 3, 3) = T.block(0, 0, 3, 3).transpose();
  Tinv.block(0, 3, 3, 1) = -Tinv.block(0, 0, 3, 3) * T.block(0, 3, 3, 1);
  return Tinv;
}
