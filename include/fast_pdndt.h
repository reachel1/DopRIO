#ifndef RISE_FAST_PDNDT_H
#define RISE_FAST_PDNDT_H

#include <execution>
#include <numeric> 
#include <mutex>
#include <Eigen/Dense>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>
#include <boost/filesystem.hpp>

#include "sensor_data.h"
#include "math_ops.h"
#include "hash_vec.h"
#include "rise_state.h"

namespace rise {

class FastPDNDT {
    public:
    enum class NearbyType {
        CENTER,   // Center only
    };

    struct Options {
        int max_iteration_ = 15;        // Maximum number of iterations
        double voxel_size_ = 1;      // Voxel size
        double inv_voxel_size_ = 1;  // The inverse of voxel size
        
        double eps_ = 1e-6;            // Convergence decision condition
        double res_outlier_th_ = 10;  // NDT outlier rejection threshold
        double dop_outlier_th_ = 0.5;  // Doppler outlier rejection threshold
        size_t capacity_ = 100000;     // The number of local voxel maps cached

        NearbyType nearby_type_ = NearbyType::CENTER;

        double kf_distance_ = 0.3;            // Keyframe distance
        double kf_angle_deg_ = 10;            // Keyframe rotation Angle
        int kf_frame_ = 10;

        double range_cov_ = 0.1; 
        double azimuth_cov_ = 0.5;
        double elevation_cov_ = 1; 
        double doppler_cov_ = 0.1;

        double pointupdate_th_ = 0.8;
    };

    /// Index of voxels
    using KeyType = Eigen::Matrix<int, 3, 1>;  

    /// Voxel built-in structure
    struct VoxelData {
        VoxelData() {}
        VoxelData(const UncertaintyRadarPoint& pt) {
            pts_.emplace_back(pt);
        }

        void AddPoint(const UncertaintyRadarPoint& pt) {
            pts_.emplace_back(pt);
        }

        std::vector<UncertaintyRadarPoint> pts_;       // Inner point
        Eigen::Matrix<double, 3, 1> mu_ = Eigen::Matrix<double, 3, 1>::Zero();     // Voxel mean 
        Eigen::Matrix<double, 3, 3> sigma_ = Eigen::Matrix<double, 3, 3>::Zero();  // Voxel cov
        Eigen::Matrix<double, 3, 3> info_ = Eigen::Matrix<double, 3, 3>::Zero();   // Voxel info

        bool ndt_estimated_ = false;  // NDT has been estimated?
        int num_pts_ = 0;             // The total number of points used to update estimates
    };

    FastPDNDT() {
        options_.inv_voxel_size_ = 1.0 / options_.voxel_size_;
    }

    FastPDNDT(Options options) : options_(options) {
        options_.inv_voxel_size_ = 1.0 / options_.voxel_size_;
    }

    /// Get some statistics
    int NumGrids() const { return grids_.size(); }

    bool first_flag(){return flag_first_scan_;}

    std::vector<std::pair<Eigen::Matrix<double,3,1>, Eigen::Matrix<double, 3, 3>>> Print_VoxelInfo(){
        
        std::vector<std::pair<Eigen::Matrix<double,3,1>, Eigen::Matrix<double, 3, 3>>> VoxelmapInfo;
        for (const auto& pair : grids_) {
            const KeyType& key = pair.first;
            // Get to data_ mesomorphism VoxelData Iterator 
            std::list<KeyAndData>::iterator iter = pair.second;
            if(!iter->second.ndt_estimated_)continue;
            std::pair<Eigen::Matrix<double,3,1>, Eigen::Matrix<double, 3, 3>> ms_pair{iter->second.mu_, iter->second.sigma_};
            VoxelmapInfo.push_back(ms_pair);
        }
        return VoxelmapInfo;

    };

    /// Set the source point cloud
    void setSource(pcl::PointCloud<RadarPointCloudType>::Ptr scan);

    /// Return to the source point cloud
    pcl::PointCloud<RadarPointCloudType>::Ptr getSource(){

        pcl::PointCloud<RadarPointCloudType>::Ptr source_clone(new pcl::PointCloud<RadarPointCloudType>);
        for(int idx = 0; idx < source_->size(); idx++){
            // transform Radarpoint to UncertaintyRadarPoint(add covariance)
            RadarPointCloudType point;
            UncertaintyRadarPoint raw_point = source_->points[idx];
            point.x = raw_point.x;
            point.y = raw_point.y;
            point.z = raw_point.z;
            point.intensity = raw_point.intensity;
            point.doppler = raw_point.doppler;
            source_clone->points.push_back(point);

        }
        return source_clone;
    }

    /// Registration calculation residuals and Jacobi
    void AlignPNDT(const Sophus::SE3d& ItoG_pose, const Sophus::SE3d& RtoI_pose, const Eigen::Matrix<double,3,1>& wI_bg, 
                    const Eigen::Matrix<double,3,1>& velG, Eigen::Matrix<double, 21, 21>& HTVH, Eigen::Matrix<double, 21, 1>& HTVr);
    void AlignPDNDT(const Sophus::SE3d& ItoG_pose, const Sophus::SE3d& RtoI_pose, const Eigen::Matrix<double,3,1>& wI_bg, 
                    const Eigen::Matrix<double,3,1>& velG, Eigen::Matrix<double, 21, 21>& HTVH, Eigen::Matrix<double, 21, 1>& HTVr);
    void calGrad(const Sophus::SE3d& ItoG, const Sophus::SE3d& RtoI_pose, const Eigen::Matrix<double,3,1>& wI_bg, 
                    const Eigen::Matrix<double,3,1>& velG);

    /// Determines whether it is a keyframe
    void IsKeyframe(const Sophus::SE3d& pose);

    /// Update radar point uncertain using doppler res
    bool SourceUpdateUncertain(RiseState x_k_1, RiseState x_k, Eigen::Matrix<double, 3, 1> wI, 
                            Eigen::Matrix<double, 21, 21> cov, Eigen::Matrix<double, 3, 1> vr);

    Options options_;

   protected:

    /// Voxel submap Add cloud
    void AddCloud(pcl::PointCloud<UncertaintyRadarPoint>::Ptr cloud_world);

    /// Update internal voxel data
    void UpdateVoxel(VoxelData& v);

    using KeyAndData = std::pair<KeyType, VoxelData>;  
    std::list<KeyAndData> data_;                       // Grid (or voxel) data
    std::unordered_map<KeyType, std::list<KeyAndData>::iterator, hash_vec<3>> grids_;  // An iterator that stores raster data

    bool flag_first_scan_ = true;  // First frame point cloud special processing
    bool first_enough = true;

    Sophus::SE3d last_kf_pose_;   // The pose of the previous keyframe

    int cnt_frame_ = 0; //Count between key frames

    std::ofstream out_printf; //Output print message

    pcl::PointCloud<UncertaintyRadarPoint>::Ptr source_{new pcl::PointCloud<UncertaintyRadarPoint>};

};

}  // namespace rise

#endif  // RISE_FAST_NDT_H