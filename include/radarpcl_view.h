#ifndef RISE_RADARPCL_VISUAL_H
#define RISE_RADARPCL_VISUAL_H

#include <thread>
#include <chrono>

#include <omp.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

#include "sensor_data.h"

namespace rise {

/// The radar point cloud viewer based on pcl viewer
class Radar_PCLView {

   public:

    /// constructor
    Radar_PCLView(const float& leaf_size = 0.1):leaf_size_(leaf_size){

        mapviewer_.reset(new pcl::visualization::PCLVisualizer());
        mapviewer_->setBackgroundColor(1.0, 1.0, 1.0);
        mapviewer_->addCoordinateSystem(10, "world");

        scanvoxel_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);
        mapvoxel_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);

    }

    void UpdateMap(const Sophus::SE3d& pose, pcl::PointCloud<RadarPointCloudType>::Ptr cloud);

    /// Save map to PCD file
    void SaveMap(std::string path) {
        if (local_map_->size() > 0) {
            pcl::PointCloud<pcl::PointXYZI> cloud;
            cloud = *local_map_;
            cloud.height = 1;
            cloud.width = local_map_->size();
            pcl::io::savePCDFileASCII(path, cloud);
            std::cout<< "save map to: " << path<<std::endl;
        } else {
            std::cout<< "map have no point" <<std::endl;
        }
    }

    void mapClean() {
        tmp_cloud_->clear();
        local_map_->clear();
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr RadarUncertain2PointCloudXYZI(const pcl::PointCloud<UncertaintyRadarPoint>::ConstPtr &raw)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudxyzi(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointXYZI point_xyzi;
        for(int i = 0; i < raw->size(); i++)
        {
            point_xyzi.x = (*raw)[i].x;
            point_xyzi.y = (*raw)[i].y;
            point_xyzi.z = (*raw)[i].z;
            point_xyzi.intensity = (*raw)[i].intensity;
            cloudxyzi->points.push_back(point_xyzi);
        }
        return cloudxyzi;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr RadarRaw2PointCloudXYZI(const pcl::PointCloud<RadarPointCloudType>::ConstPtr &raw)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudxyzi(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointXYZI radarpoint_xyzi;
        for(int i = 0; i < raw->size(); i++)
        {
            radarpoint_xyzi.x = (*raw)[i].x;
            radarpoint_xyzi.y = (*raw)[i].y;
            radarpoint_xyzi.z = (*raw)[i].z;
            radarpoint_xyzi.intensity = (*raw)[i].intensity;
            cloudxyzi->points.push_back(radarpoint_xyzi);
        }
        return cloudxyzi;
    }


   private:

    float leaf_size_;
    
    pcl::visualization::PCLVisualizer::Ptr mapviewer_ = nullptr;  // map viewer

    pcl::VoxelGrid<pcl::PointXYZI> mapvoxel_filter_;
    pcl::VoxelGrid<pcl::PointXYZI> scanvoxel_filter_;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cloud_{new pcl::PointCloud<pcl::PointXYZI>};  //
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_{new pcl::PointCloud<pcl::PointXYZI>};

};
}  // namespace rise

#endif  // RISE_RADARPCL_VISUAL_H