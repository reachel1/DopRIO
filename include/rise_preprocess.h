#include <string>
#include <fstream>
#include <functional>

#include <ros/ros.h>
#include <ros/time.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/filesystem.hpp>

#include "eskf_rise.h"

using namespace std;

namespace rise{

class RisePreprocess{

public: 

  RisePreprocess(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<Eskf_Rise> risekf, 
              std::string config_path): nh_(nh), state_est(risekf) {

    initializeParams(config_path);

    radar_pub = nh_->advertise<sensor_msgs::PointCloud2>("/filtered_points", 32);
  }

  virtual ~RisePreprocess() {}

  /// Set the callback
  void set_subscriber(){
    radar_sub = nh_->subscribe(radarTopic, 20, &RisePreprocess::cloud_callback, this);
    imu_sub = nh_->subscribe(imuTopic, 1000, &RisePreprocess::imu_callback, this);
  }

  /// IMU Callback function
  void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg);

  /// 4dRadar Callback function
  void cloud_callback(const sensor_msgs::PointCloud::ConstPtr&  eagle_msg);
  void cloud2_callback(const sensor_msgs::PointCloud2::ConstPtr&  rsscan_msg);

  /// 4dRadar filter function
  void RadarRawFilter(const pcl::PointCloud<RadarPointCloudType>::Ptr &raw, pcl::PointCloud<RadarPointCloudType>::Ptr &output){
 
    pcl::PointCloud<RadarPointCloudType>::Ptr radar_cloudf1(new pcl::PointCloud<RadarPointCloudType>);
    pcl::PointCloud<RadarPointCloudType>::Ptr radar_cloudf2(new pcl::PointCloud<RadarPointCloudType>);
    pcl::PointCloud<RadarPointCloudType>::Ptr radar_cloudf3(new pcl::PointCloud<RadarPointCloudType>);
    
    std::cout<<"ori size: "<<raw->size()<<std::endl;
    radar_cloudf1 = RangeFilter(raw);
    std::cout<<"range size: "<<radar_cloudf1->size()<<std::endl;
    radar_cloudf2 = DopplerFilter(radar_cloudf1);
    std::cout<<"doppler size: "<<radar_cloudf2->size()<<std::endl;
    radar_cloudf3 = IntensityFilter(radar_cloudf2);
    std::cout<<"intensity size: "<<radar_cloudf3->size()<<std::endl;
    output = NeighborFilter(radar_cloudf3);
    std::cout<<"neighbor size: "<<output->size()<<std::endl;
            
  }

private:

  void initializeParams(std::string config_path) {

    cv::FileStorage fs(config_path, cv::FileStorage::READ);

    // Check whether the file is successfully opened
    if (!fs.isOpened()) {
        std::cout << "Failed to open config file!" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    // Read topic name
    fs["imu_topic"] >> imuTopic;
    fs["radar_topic"] >> radarTopic;
    printf("Imu topic: %s || Radar topic: %s\n", imuTopic.c_str(), radarTopic.c_str());

    fs["minRange"] >> distance_min_thresh_;
    fs["maxRange"] >> distance_max_thresh_;
    fs["maxDoppler"] >> doppler_max_thresh_;
    fs["minIntensity"] >> intensity_min_thresh_;
    fs["NeighborRange"] >> neighbor_range_;
    fs["NeighborNum"] >> neighbor_num_;
    fs["NeighborStd"] >> neighbor_std_;
  
    fs.release();

  }

  pcl::PointCloud<RadarPointCloudType>::Ptr RangeFilter(const pcl::PointCloud<RadarPointCloudType>::ConstPtr &raw);
  pcl::PointCloud<RadarPointCloudType>::Ptr DopplerFilter(const pcl::PointCloud<RadarPointCloudType>::ConstPtr &raw);
  pcl::PointCloud<RadarPointCloudType>::Ptr IntensityFilter(const pcl::PointCloud<RadarPointCloudType>::ConstPtr &raw);
  pcl::PointCloud<RadarPointCloudType>::Ptr NeighborFilter(const pcl::PointCloud<RadarPointCloudType>::ConstPtr &raw);

  std::shared_ptr<ros::NodeHandle> nh_;

  ros::Subscriber imu_sub;
  ros::Subscriber radar_sub;

  ros::Publisher radar_pub;

  /// Filter parameter
  double distance_min_thresh_, distance_max_thresh_, doppler_max_thresh_, intensity_min_thresh_, neighbor_range_, neighbor_std_;
  int neighbor_num_;

  std::string radarTopic, imuTopic;

  std::shared_ptr<Eskf_Rise> state_est;

  float power_threshold = -1;
};

} // namespace rise