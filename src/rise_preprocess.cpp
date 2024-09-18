#include "rise_preprocess.h"

using namespace std;
using namespace rise;

void RisePreprocess::imu_callback(const sensor_msgs::ImuConstPtr& imu_msg) {

  ImuData imud;
  imud.timestamp = imu_msg->header.stamp.toSec();
  imud.am << imu_msg->linear_acceleration.x,imu_msg->linear_acceleration.y,imu_msg->linear_acceleration.z;
  imud.wm << imu_msg->angular_velocity.x,imu_msg->angular_velocity.y,imu_msg->angular_velocity.z;
  // imu_queue.push_back(imud);
  printf("imu time: %.3f\n",imu_msg->header.stamp.toSec());
  state_est->feed_imu_inf(imud);

}

void RisePreprocess::cloud_callback(const sensor_msgs::PointCloud::ConstPtr&  eagle_msg) { 
  
  RadarPointCloudType radarpoint_raw;
  pcl::PointCloud<RadarPointCloudType>::Ptr radarcloud_raw( new pcl::PointCloud<RadarPointCloudType>);
  pcl::PointCloud<RadarPointCloudType>::Ptr radarcloud_filter( new pcl::PointCloud<RadarPointCloudType>);

  printf("radar time: %.3f\n",eagle_msg->header.stamp.toSec());
  double radar_time = eagle_msg->header.stamp.toSec();
  
  for(int i = 0; i < eagle_msg->points.size(); i++)
  {
      // cout << i << ":    " <<eagle_msg->points[i].x<<endl;
      if(eagle_msg->channels[2].values[i] > power_threshold) //"Power"
      {
          if (eagle_msg->points[i].x == NAN || eagle_msg->points[i].y == NAN || eagle_msg->points[i].z == NAN) continue;
          if (eagle_msg->points[i].x == INFINITY || eagle_msg->points[i].y == INFINITY || eagle_msg->points[i].z == INFINITY) continue;

          radarpoint_raw.x = eagle_msg->points[i].x;
          radarpoint_raw.y = eagle_msg->points[i].y;
          radarpoint_raw.z = eagle_msg->points[i].z;
          radarpoint_raw.intensity = eagle_msg->channels[2].values[i];
          radarpoint_raw.doppler = eagle_msg->channels[0].values[i];

          radarcloud_raw->points.push_back(radarpoint_raw);
      }
  }
  RisePreprocess::RadarRawFilter(radarcloud_raw, radarcloud_filter);
  state_est->feed_radar_pointcloud(radar_time, radarcloud_filter, radarcloud_raw);
}

void RisePreprocess::cloud2_callback(const sensor_msgs::PointCloud2::ConstPtr&  rsscan_msg) { 
  
  sensor_msgs::PointCloud2 rsscan = *rsscan_msg;
  sensor_msgs::PointCloud2Iterator<float> iter_x(rsscan, "x"),iter_y(rsscan, "y"),iter_z(rsscan, "z"),
                                iter_intensity(rsscan, "intensity"),iter_doppler(rsscan, "doppler");

  RadarPointCloudType radarpoint_raw;
  pcl::PointCloud<RadarPointCloudType>::Ptr radarcloud_raw( new pcl::PointCloud<RadarPointCloudType>);
  pcl::PointCloud<RadarPointCloudType>::Ptr radarcloud_filter( new pcl::PointCloud<RadarPointCloudType>);

  printf("radar time: %.3f\n",rsscan.header.stamp.toSec());
  double radar_time = rsscan.header.stamp.toSec();
  
  for (size_t i = 0; i < rsscan.height * rsscan.width; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_doppler)
  {
      
        if (*iter_x == NAN || *iter_y == NAN || *iter_z == NAN) continue;
        if (*iter_x == INFINITY || *iter_y == INFINITY || *iter_z == INFINITY) continue;

        radarpoint_raw.x = *iter_x;
        radarpoint_raw.y = *iter_y;
        radarpoint_raw.z = *iter_z;
        radarpoint_raw.intensity = *iter_intensity;
        radarpoint_raw.doppler = *iter_doppler;

        radarcloud_raw->points.push_back(radarpoint_raw);

  }
  RisePreprocess::RadarRawFilter(radarcloud_raw, radarcloud_filter);
  state_est->feed_radar_pointcloud(radar_time, radarcloud_filter, radarcloud_raw);
}

/// range value filter point
pcl::PointCloud<RadarPointCloudType>::Ptr RisePreprocess::RangeFilter(const pcl::PointCloud<RadarPointCloudType>::ConstPtr &raw){

    pcl::PointCloud<RadarPointCloudType>::Ptr cloud(new pcl::PointCloud<RadarPointCloudType>);
    RadarPointCloudType pointr;
    for(int i = 0; i < raw->size(); i++){
        
        Eigen::Vector3d pos((*raw)[i].x,(*raw)[i].y,(*raw)[i].z);
        double posnorm = pos.norm();
        if(posnorm < distance_min_thresh_ || posnorm > distance_max_thresh_)continue;
        pointr.x = (*raw)[i].x;
        pointr.y = (*raw)[i].y;
        pointr.z = (*raw)[i].z;
        pointr.intensity = (*raw)[i].intensity;
        pointr.doppler = (*raw)[i].doppler;

        cloud->points.push_back(pointr);
    }
    
    return cloud;
}

/// doppler value filter point
pcl::PointCloud<RadarPointCloudType>::Ptr RisePreprocess::DopplerFilter(const pcl::PointCloud<RadarPointCloudType>::ConstPtr &raw){

    pcl::PointCloud<RadarPointCloudType>::Ptr cloud(new pcl::PointCloud<RadarPointCloudType>);
    RadarPointCloudType pointr;
    for(int i = 0; i < raw->size(); i++){
        
        if((*raw)[i].doppler > doppler_max_thresh_)continue;
        pointr.x = (*raw)[i].x;
        pointr.y = (*raw)[i].y;
        pointr.z = (*raw)[i].z;
        pointr.intensity = (*raw)[i].intensity;
        pointr.doppler = (*raw)[i].doppler;

        cloud->points.push_back(pointr);
    }
    
    return cloud;
}

/// intensity value filter point
pcl::PointCloud<RadarPointCloudType>::Ptr RisePreprocess::IntensityFilter(const pcl::PointCloud<RadarPointCloudType>::ConstPtr &raw){

    pcl::PointCloud<RadarPointCloudType>::Ptr cloud(new pcl::PointCloud<RadarPointCloudType>);
    RadarPointCloudType pointr;

    double max_intensity = 0.0;
    for(int i = 0; i < raw->size(); i++){
        if((*raw)[i].intensity > max_intensity)max_intensity = (*raw)[i].intensity;
    }
    for(int i = 0; i < raw->size(); i++){
        
        if((*raw)[i].intensity/max_intensity < intensity_min_thresh_)continue;
        pointr.x = (*raw)[i].x;
        pointr.y = (*raw)[i].y;
        pointr.z = (*raw)[i].z;
        pointr.intensity = (*raw)[i].intensity;
        pointr.doppler = (*raw)[i].doppler;

        cloud->points.push_back(pointr);
    }
    
    return cloud;
}

/// Neighbor value filter point
pcl::PointCloud<RadarPointCloudType>::Ptr RisePreprocess::NeighborFilter(const pcl::PointCloud<RadarPointCloudType>::ConstPtr &raw){

    pcl::PointCloud<RadarPointCloudType>::Ptr cloud(new pcl::PointCloud<RadarPointCloudType>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz(new pcl::PointCloud<pcl::PointXYZ>);
    
    RadarPointCloudType pointr;
    pcl::PointXYZ point_xyz;
    
    for(int i = 0; i < raw->size(); i++)
    {
        point_xyz.x = (*raw)[i].x;
        point_xyz.y = (*raw)[i].y;
        point_xyz.z = (*raw)[i].z;
        cloudxyz->points.push_back(point_xyz);
    }
    // Build KD-tree
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloudxyz);

    // Define parameters for neighborhood search
    double radius = neighbor_range_; // Neighborhood radius
    if(radius < 0){
        for (size_t i = 0; i < cloudxyz->size(); ++i){

            pointr.x = (*raw)[i].x;
            pointr.y = (*raw)[i].y;
            pointr.z = (*raw)[i].z;
            pointr.intensity = (*raw)[i].intensity;
            pointr.doppler = (*raw)[i].doppler;

            cloud->points.push_back(pointr);

        }
        return cloud;
    }
    std::vector<int> indices;
    std::vector<float> distances;

    // Traverse each point in the point cloud
    for (size_t i = 0; i < cloudxyz->size(); ++i)
    {
        // Search for points in the neighborhood
        if (kdtree.radiusSearch(cloudxyz->points[i], radius, indices, distances) > 0)
        {
            // Count points in a neighborhood
            // std::cout << "Neighbors count: " << indices.size() << std::endl;

            // Calculate the standard deviation of the coordinates of points in the neighborhood
            double mean_x = 0, mean_y = 0, mean_z = 0;
            for (auto idx : indices)
            {
                mean_x += cloudxyz->points[idx].x;
                mean_y += cloudxyz->points[idx].y;
                mean_z += cloudxyz->points[idx].z;
            }
            mean_x /= indices.size();
            mean_y /= indices.size();
            mean_z /= indices.size();

            double std_x = 0, std_y = 0, std_z = 0;
            for (auto idx : indices)
            {
                std_x += (cloudxyz->points[idx].x - mean_x) * (cloudxyz->points[idx].x - mean_x);
                std_y += (cloudxyz->points[idx].y - mean_y) * (cloudxyz->points[idx].y - mean_y);
                std_z += (cloudxyz->points[idx].z - mean_z) * (cloudxyz->points[idx].z - mean_z);
            }
            std_x = std::sqrt(std_x / indices.size());
            std_y = std::sqrt(std_y / indices.size());
            std_z = std::sqrt(std_z / indices.size());

            // Outputs the standard deviation for each dimension
            // std::cout << "Standard deviation (x, y, z): "<< std_x << ", " << std_y << ", " << std_z << std::endl;

            if(indices.size() >= neighbor_num_ && std_x < neighbor_std_ && std_y < neighbor_std_ && std_z < neighbor_std_){
                pointr.x = (*raw)[i].x;
                pointr.y = (*raw)[i].y;
                pointr.z = (*raw)[i].z;
                pointr.intensity = (*raw)[i].intensity;
                pointr.doppler = (*raw)[i].doppler;

                cloud->points.push_back(pointr);
            }
        }
        else
        {
            indices.clear();
            distances.clear();
            continue;
        }

        // clear indices and distances for the next search
        indices.clear();
        distances.clear();
    }
    return cloud;
}



