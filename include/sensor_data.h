#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <vector>

struct RadarPointCloudType: public pcl::PointXYZI
{
  PCL_ADD_POINT4D      // x,y,z position in [m]
  PCL_ADD_INTENSITY;
  union
    {
      struct
      {
        float doppler;
      };
      float data_c[4];
    };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Make sure to define the new type of point cloud memory align to SSE 

} EIGEN_ALIGN16; // Force SSE to properly align memory
POINT_CLOUD_REGISTER_POINT_STRUCT
(
    RadarPointCloudType,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, doppler, doppler)
)

struct UncertaintyRadarPoint: public pcl::PointXYZI
{
  PCL_ADD_POINT4D;      // x,y,z position in [m]
  PCL_ADD_INTENSITY;
  
  float doppler;

  EIGEN_ALIGN16 Eigen::Matrix3d covariance;  // 4x4 covariance matrix

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Make sure to define the new type of point cloud memory align to SSE 

} EIGEN_ALIGN16;  // Force SSE to properly align memory
POINT_CLOUD_REGISTER_POINT_STRUCT
(
    UncertaintyRadarPoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, doppler, doppler)
)


struct ImuData
{
  /// Timestamp of the reading
  double timestamp;

  /// Accelerometer reading, linear acceleration (m/s^2)
  Eigen::Vector3d am;

  /// Gyroscope reading, angular velocity (rad/s)
  Eigen::Vector3d wm;

  /// Oritention reading
  Eigen::Quaterniond qm;

  /// Sort function to allow for using of STL containers
  bool operator<(const ImuData &other) const { return timestamp < other.timestamp;}

};

struct RadarVelData
{
  /// Timestamp of the reading
  double timestamp;

  /// Accelerometer reading, linear acceleration (m/s^2)
  Eigen::Vector3d vel;

  /// Gyroscope reading, angular velocity (rad/s)
  Eigen::Vector3d vel_sigma;

  /// Sort function to allow for using of STL containers
  bool operator<(const RadarVelData &other) const { return timestamp < other.timestamp;}

};

struct RadarData
{
  /// Timestamp of the reading
  double timestamp;

  /// Accelerometer reading, linear acceleration (m/s^2)
  pcl::PointCloud<RadarPointCloudType>::Ptr radar_scan{new pcl::PointCloud<RadarPointCloudType>};
  pcl::PointCloud<RadarPointCloudType>::Ptr radar_scan_raw{new pcl::PointCloud<RadarPointCloudType>};

  /// Velocity from estimate doppler
  Eigen::Vector3d velo;
  Eigen::Vector3d velo_sigma;

  /// Sort function to allow for using of STL containers
  bool operator<(const RadarData &other) const { return timestamp < other.timestamp;}

};

#endif // SENSOR_DATA_H