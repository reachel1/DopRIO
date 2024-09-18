#include "radarpcl_view.h"

using namespace rise;

void Radar_PCLView::UpdateMap(const Sophus::SE3d& pose, pcl::PointCloud<RadarPointCloudType>::Ptr cloud) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_trans->points.resize(cloud->points.size());
    Eigen::Matrix4f transform = pose.matrix().cast<float>();

    #pragma omp parallel for
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const auto& point = cloud->points[i];
        Eigen::Vector4f pt(point.x, point.y, point.z, 1.0f);
        Eigen::Vector4f pt_transformed = transform * pt;

        if (std::isnan(pt_transformed.x()) || std::isnan(pt_transformed.y()) || std::isnan(pt_transformed.z()) ||
            std::isinf(pt_transformed.x()) || std::isinf(pt_transformed.y()) || std::isinf(pt_transformed.z())) {
            #pragma omp critical
            {
                std::cerr << "Found NaN or Inf in transformed point: ("
                        << pt_transformed.x() << ", " << pt_transformed.y() << ", " << pt_transformed.z() << ")" << std::endl;
            }
        }

        pcl::PointXYZI transformed_point;
        transformed_point.x = pt_transformed.x();
        transformed_point.y = pt_transformed.y();
        transformed_point.z = pt_transformed.z();
        transformed_point.intensity = point.intensity;
        cloud_trans->points[i] = transformed_point;
    }

    tmp_cloud_->clear();
    tmp_cloud_ = cloud_trans;
    scanvoxel_filter_.setInputCloud(tmp_cloud_);
    scanvoxel_filter_.filter(*tmp_cloud_);
    tmp_cloud_->is_dense = false;
    *local_map_ += *tmp_cloud_;
    local_map_->is_dense = false;
    mapvoxel_filter_.setInputCloud(local_map_);
    mapvoxel_filter_.filter(*local_map_);

    if (local_map_->size() > 100000) {
        leaf_size_ *= 1.26;
        mapvoxel_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    }
    
    if(mapviewer_ != nullptr)
    {
        mapviewer_->removePointCloud("local_map");
        mapviewer_->removeCoordinateSystem("vehicle");

        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fieldColor(local_map_, "z");
        mapviewer_->addPointCloud<pcl::PointXYZI>(local_map_, fieldColor, "local_map");

        Eigen::Affine3f T;
        T.matrix() = pose.matrix().cast<float>();
        mapviewer_->addCoordinateSystem(5, T, "vehicle");
        mapviewer_->spinOnce(0);

    }
}