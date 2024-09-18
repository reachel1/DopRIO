#include <string>
#include <fstream>
#include <functional>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/filesystem.hpp>

#include <memory>

#include "eskf_rise.h"
#include "rise_preprocess.h"

using namespace rise;

std::shared_ptr<RisePreprocess> risePre;
std::shared_ptr<Eskf_Rise> riseKf;
std::shared_ptr<Estimate_Vel> riseEstv;

// Main function
int main(int argc, char **argv) {

  // Ensure we have a path, if the user passes it then we should use it
  std::string bag_path = "unset_path_to_bag.bag";
  std::string config_path = "unset_path_to_config.yaml";
  std::string output_path = "unset_path_to_output";
  std::string dataset = "unset_dataset_type";
  std::string gt_path = "unset_gt_path";
  if (argc > 5) {
    dataset = argv[1];
    bag_path = argv[2];
    config_path = argv[3];
    output_path = argv[4];
    gt_path = argv[5];
  }

  // Launch our ros node
  ros::init(argc, argv, "rise_for_serial");
  auto nh = std::make_shared<ros::NodeHandle>("~");
  nh->param<std::string>("bag_path", bag_path, bag_path);
  nh->param<std::string>("config_path", config_path, config_path);
  nh->param<std::string>("output_path", output_path, output_path);
  nh->param<std::string>("dataset", dataset, dataset);
  nh->param<std::string>("gt_file_path", gt_path, gt_path);

  // load the config
  std::cout<<"dataset_type: "<<dataset<<std::endl;
  std::cout<<"bag_path: "<<bag_path<<std::endl;
  std::cout<<"config_path: "<<config_path<<std::endl;
  std::cout<<"output_path: "<<output_path<<std::endl;
  std::cout<<"gt_path: "<<gt_path<<std::endl;

  // load imu and radar topic
  std::string ImuTopic, RadarTopic;
  cv::FileStorage fs(config_path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
      std::cout << "Failed to open config file!" << std::endl;
      std::exit(EXIT_FAILURE);
  }
  fs["imu_topic"] >> ImuTopic;
  fs["radar_topic"] >> RadarTopic;
  printf("Imu topic: %s || Radar topic: %s\n", ImuTopic.c_str(), RadarTopic.c_str());
  fs.release();

  // rise ptr init
  riseEstv = std::make_shared<Estimate_Vel>(output_path);
  riseKf = std::make_shared<Eskf_Rise>(riseEstv, config_path, output_path, gt_path);
  risePre = std::make_shared<RisePreprocess>(nh, riseKf, config_path);

  // load bag
  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);
  rosbag::View view_full;
  rosbag::View view;
  view_full.addQuery(bag);
  ros::Time time_init = view_full.getBeginTime();
  ros::Time time_finish = view_full.getEndTime();
  printf("bag time start = %.6f\n", time_init.toSec());
  printf("bag time end   = %.6f\n", time_finish.toSec());
  view.addQuery(bag, time_init, time_finish);
  if (view.size() == 0) {
    printf("No messages to play on specified topics.  Exiting.\n");
    ros::shutdown();
    return EXIT_FAILURE;
  }
  // rise sub msg
  // imu
  std::shared_ptr<rosbag::View> view_imu = std::make_shared<rosbag::View>(bag, rosbag::TopicQuery(ImuTopic), time_init, time_finish);
  rosbag::View::iterator view_imu_iter = view_imu->begin();
  sensor_msgs::Imu::ConstPtr msg_imu_current;
  msg_imu_current = view_imu_iter->instantiate<sensor_msgs::Imu>();
  // 4dradar
  std::shared_ptr<rosbag::View> view_radar = std::make_shared<rosbag::View>(bag, rosbag::TopicQuery(RadarTopic), time_init, time_finish);
  rosbag::View::iterator view_radar_iter = view_radar->begin();
  sensor_msgs::PointCloud::ConstPtr msg_radar_current1;
  sensor_msgs::PointCloud2::ConstPtr msg_radar_current2;

  if(dataset == "ntu")msg_radar_current1 = view_radar_iter->instantiate<sensor_msgs::PointCloud>();
  else if(dataset == "coloradar")msg_radar_current2 = view_radar_iter->instantiate<sensor_msgs::PointCloud2>();
  else{
    printf("Dataset error .  Exiting.\n");
    ros::shutdown();
    return EXIT_FAILURE;
  }

  // read bag
  bool should_stop = false;
  while (ros::ok()) {
    // Check if we should end since we have run out of measurements
    if (should_stop) {
      printf("rise for serial node break\n");
      break;
    }
    double time_imu = msg_imu_current->header.stamp.toSec();
    double time_radar = -1.0;
    if(dataset == "ntu")time_radar = msg_radar_current1->header.stamp.toSec();
    else if(dataset == "coloradar")time_radar = msg_radar_current2->header.stamp.toSec();


    if(time_radar <= time_imu){
        if(dataset == "ntu")risePre->cloud_callback(msg_radar_current1);
        else if(dataset == "coloradar")risePre->cloud2_callback(msg_radar_current2);
        view_radar_iter++;
        if (view_radar_iter == view_radar->end()) {
            should_stop = true;
            continue;
        }
        if(dataset == "ntu")msg_radar_current1 = view_radar_iter->instantiate<sensor_msgs::PointCloud>();
        else if(dataset == "coloradar")msg_radar_current2 = view_radar_iter->instantiate<sensor_msgs::PointCloud2>();
    }
    else{
        risePre->imu_callback(msg_imu_current);
        view_imu_iter++;
        if (view_imu_iter == view_imu->end()) {
            should_stop = true;
            continue;
        }
        msg_imu_current = view_imu_iter->instantiate<sensor_msgs::Imu>();
    }
  }

  // final save 
  riseKf->saveRadarMap(output_path);
  ros::shutdown();
  // done!
  return EXIT_SUCCESS;
  
}