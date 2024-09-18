#include "fast_pdndt.h"

using namespace rise;
using namespace std;

void FastPDNDT::setSource(pcl::PointCloud<RadarPointCloudType>::Ptr scan){

    source_ = pcl::PointCloud<UncertaintyRadarPoint>::Ptr(new pcl::PointCloud<UncertaintyRadarPoint>());;

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
    covariance(0,0) = pow(options_.range_cov_,2);
    covariance(1,1) = pow(options_.azimuth_cov_* M_PI/180,2);
    covariance(2,2) = pow(options_.elevation_cov_* M_PI/180,2);

    int point1 = 0, point2 = 0, point3 = 0;

    for(int idx = 0; idx < scan->size(); idx++){
        
        // transform Radarpoint to UncertaintyRadarPoint(add covariance)
        UncertaintyRadarPoint point;
        RadarPointCloudType raw_point = scan->points[idx];
        point.x = raw_point.x;
        point.y = raw_point.y;
        point.z = raw_point.z;
        point.intensity = raw_point.intensity;
        point.doppler = raw_point.doppler;
        // (x,y,z,d) = J(range, azimuth, elevation, doppler)
        Eigen::Matrix3d jacobian = Eigen::Matrix3d::Identity();
        double range = Eigen::Vector3d(raw_point.x, raw_point.y, raw_point.z).norm();
        double elevation = asin(raw_point.z/range);
        double azimuth = atan2(raw_point.y, raw_point.x);
        double cos_elevation = std::cos(elevation);
        double sin_elevation = std::sin(elevation);
        double cos_azimuth = std::cos(azimuth);
        double sin_azimuth = std::sin(azimuth);

        jacobian << cos_elevation * cos_azimuth, -range * cos_elevation * sin_azimuth, -range * sin_elevation * cos_azimuth,
                    cos_elevation * sin_azimuth, range * cos_elevation * cos_azimuth, -range * sin_elevation * sin_azimuth,
                    sin_azimuth, 0, range * cos_elevation;

        // Cov_xyzd = J * Cov_raed * J.transpose()
        point.covariance = jacobian * covariance * jacobian.transpose();
        source_->points.push_back(point);
    }
}

bool FastPDNDT::SourceUpdateUncertain(RiseState x_k_1, RiseState x_k, Eigen::Matrix<double, 3, 1> wI, 
                                        Eigen::Matrix<double, 21, 21> cov, Eigen::Matrix<double, 3, 1> vr){

    Eigen::Vector3d v_radar1 = x_k_1.GetextSE3().so3().matrix().transpose() * 
                        (x_k_1.GetSE3().so3().matrix().transpose() * x_k_1.GetVel() + 
                        Sophus::SO3d::hat(wI-x_k_1.GetBiasGYR()) * x_k_1.GetextSE3().translation());
    Eigen::Vector3d v_radar2 = x_k.GetextSE3().so3().matrix().transpose() * 
                        (x_k.GetSE3().so3().matrix().transpose() * x_k.GetVel() + 
                        Sophus::SO3d::hat(wI-x_k.GetBiasGYR()) * x_k.GetextSE3().translation());
                        
    if(fabs(v_radar1(0)) < 1e-3 || fabs(v_radar1(1)) < 1e-3 || fabs(v_radar1(2)) < 1e-3 || 
            fabs(v_radar2(0)) < 1e-3 || fabs(v_radar2(1)) < 1e-3 || fabs(v_radar2(2)) < 1e-3)return false;

    std::cout<<"vr_esi: "<<vr.transpose()<<", vr1: "<<v_radar1.transpose()<<", vr2: "<<v_radar2.transpose()<<std::endl;
    int dop_down_num = 0, delete_num = 0;
    // check new velocity
    for(int idx = 0;idx < source_->size();idx++){

        double obs_dopp = -source_->at(idx).doppler;
        Eigen::Vector3d q = Eigen::Vector3d(source_->at(idx).x, source_->at(idx).y, source_->at(idx).z);
        Eigen::Vector3d q_vector = q/q.norm(); 
        double esi_dop1 = q_vector.dot(v_radar1);
        double esi_dop2 = q_vector.dot(v_radar2);

        double delta_dop = fabs(obs_dopp - esi_dop2) - fabs(obs_dopp - esi_dop1);
        if(delta_dop < 0)dop_down_num++;
    }

    if((double)dop_down_num/source_->size() < options_.pointupdate_th_){
        std::cout<<"[Unupdate point]down num: "<<dop_down_num<<"("<<(double)dop_down_num/(double)(source_->size())<<")"<<std::endl;
        return false;}
    // erase outliner points
    auto it = source_->points.begin();
    while (it != source_->points.end()) {

        double obs_dopp = -(*it).doppler;
        Eigen::Vector3d q = Eigen::Vector3d((*it).x, (*it).y, (*it).z);
        Eigen::Vector3d q_vector = q/q.norm(); 
        double r2 = std::pow(q.norm(),2); 

        double esi_dop1 = q_vector.dot(v_radar1);
        double esi_dop2 = q_vector.dot(v_radar2);

        double delta_dop = fabs(obs_dopp - esi_dop2) - fabs(obs_dopp - esi_dop1);
        if(delta_dop >= 0){it = source_->points.erase(it);delete_num++;continue;}
        else it++;
        
    }

    for(int idx = 0;idx < source_->size();idx++){

        double obs_dopp = -source_->at(idx).doppler;
        Eigen::Vector3d q = Eigen::Vector3d(source_->at(idx).x, source_->at(idx).y, source_->at(idx).z);
        Eigen::Vector3d q_vector = q/q.norm(); 
        // if(fabs(q_vector(0)) < 1e-3 || fabs(q_vector(1)) < 1e-3 || fabs(q_vector(2)) < 1e-3)continue;
        double r = q.norm();
        double r2 = std::pow(q.norm(),2); 

        double esi_dop1 = q_vector.dot(v_radar1);
        double esi_dop2 = q_vector.dot(v_radar2);

        double delta_dop = fabs(obs_dopp - esi_dop2) - fabs(obs_dopp - esi_dop1);
        if(delta_dop >= 0){
            continue;
        }

        // Find the partial of the absolute value. Here v_q^T = A
        Eigen::Matrix<double, 3, 3> cov_Radarxyz_r2 = source_->at(idx).covariance/r2;
        Eigen::Matrix<double, 1, 3> vT = v_radar2.transpose();
        Eigen::Matrix<double, 1, 1> dop_noise(pow(options_.doppler_cov_, 2));
        Eigen::Matrix<double, 3, 1> K = cov_Radarxyz_r2 * vT.transpose() * (vT * cov_Radarxyz_r2 * vT.transpose() + dop_noise.inverse()).inverse();
        q_vector = q_vector + K * (obs_dopp - esi_dop2);
        cov_Radarxyz_r2 = (Eigen::Matrix<double,3,3>::Identity() - K *vT) * cov_Radarxyz_r2;
        Eigen::Matrix<double, 3, 3> cov_RadarXYZ = r2 * cov_Radarxyz_r2; 
        //else cov_RadarXYZ = r2 * (cov_Radarxyz_r2 + cov_Radarxyz_r2 * v_q * 
                                        //(vq_T * cov_Radarxyz_r2 * v_q + sigma_dopq).inverse() * vq_T * cov_Radarxyz_r2);
        
        // q = r * q_vector; 
        // source_->at(idx).x = q(0);
        // source_->at(idx).y = q(1);
        // source_->at(idx).z = q(2);
        source_->at(idx).covariance = cov_RadarXYZ;
        
    }
    std::cout<<"[Update point]update num: "<<dop_down_num<<"("<<(double)dop_down_num/(double)(dop_down_num + delete_num)<<")"
             <<",erase num: "<<delete_num<<std::endl;
    return true;
    
}

void FastPDNDT::IsKeyframe(const Sophus::SE3d& pose){
    
    Sophus::SE3d kf_pose_ = pose;
    pcl::PointCloud<UncertaintyRadarPoint>::Ptr source_world(new pcl::PointCloud<UncertaintyRadarPoint>);
    
    Eigen::Matrix4f transform = kf_pose_.matrix().cast<float>();
    // pcl::transformPointCloud(*source_, *source_world, kf_pose_.matrix().cast<float>());
    for (const auto& point : source_->points) {
        Eigen::Vector4f pt(point.x, point.y, point.z, 1.0f);
        Eigen::Vector4f pt_transformed = transform * pt;

        if (std::isnan(pt_transformed.x()) || std::isnan(pt_transformed.y()) || std::isnan(pt_transformed.z()) ||
            std::isinf(pt_transformed.x()) || std::isinf(pt_transformed.y()) || std::isinf(pt_transformed.z())) {
            std::cerr << "Found NaN or Inf in transformed point: (" 
                    << pt_transformed.x() << ", " << pt_transformed.y() << ", " << pt_transformed.z() << ")" << std::endl;
        }

        UncertaintyRadarPoint transformed_point;
        transformed_point.x = pt_transformed.x();
        transformed_point.y = pt_transformed.y();
        transformed_point.z = pt_transformed.z();
        transformed_point.intensity = point.intensity;
        transformed_point.doppler = point.doppler;
        transformed_point.covariance = point.covariance;
        source_world->points.push_back(transformed_point);
    }
    cnt_frame_++;
    if (flag_first_scan_) {
        // first frame, add to local map
        last_kf_pose_ = kf_pose_;
        AddCloud(source_world);
        flag_first_scan_ = false;
        return;
    }

    Sophus::SE3d delta = last_kf_pose_.inverse() * kf_pose_;
    if (cnt_frame_ > options_.kf_frame_ || delta.translation().norm() > options_.kf_distance_ ||
           delta.so3().log().norm() > options_.kf_angle_deg_ * math::kDEG2RAD) {
        last_kf_pose_ = kf_pose_;
        cnt_frame_ = 0;
        // if key frame, add to local map
        AddCloud(source_world);
    }
}

void FastPDNDT::AddCloud(pcl::PointCloud<UncertaintyRadarPoint>::Ptr cloud_world) {
    // record which voxel should be updated
    std::set<KeyType, less_vec<3>> active_voxels; 

    for (const auto& p : cloud_world->points) {
        
        Eigen::Vector3d pt = Eigen::Vector3d(p.x, p.y, p.z);
        auto key = (pt * options_.inv_voxel_size_).cast<int>();

        auto iter = grids_.find(key);
        if (iter == grids_.end()) {
            // grid not found, set a new voxel/grid and put it at front
            data_.push_front({key, {p}});
            grids_.insert({key, data_.begin()});

            if (data_.size() >= options_.capacity_) {
                // delete a data at the end
                grids_.erase(data_.back().first);
                data_.pop_back();
            }
        } else {
            // Grid exists, add points and update(put it at front)
            iter->second->second.AddPoint(p);
            data_.splice(data_.begin(), data_, iter->second); 
            iter->second = data_.begin();                      
        }

        active_voxels.emplace(key);

    }
    
    // update active_voxels
    // std::mutex mtx;
    // std::for_each(std::execution::par_unseq, active_voxels.begin(), active_voxels.end(),
    //               [this](const auto& key) { std::lock_guard<std::mutex> lock(mtx);UpdateVoxel(grids_[key]->second); });
    for(auto key = active_voxels.begin();key != active_voxels.end();key++){
        UpdateVoxel(grids_[*key]->second);
    }
}

void FastPDNDT::UpdateVoxel(VoxelData& v) {

    // voxel
    double total_wi = 0.0, total_wi_2 = 0.0;
    Eigen::Vector3d wp_mu = Eigen::Vector3d::Zero();
    Eigen::Matrix3d wp_sigma = Eigen::Matrix3d::Zero();
    v.num_pts_ = v.pts_.size();
    // if(v.num_pts_ < 3)return;
    // cal mean of voxel
    for (const auto& point : v.pts_) {
        double wi = std::exp(-(point.covariance(0,0) + point.covariance(1,1) + point.covariance(2,2)));
        //double wi = 1/v.num_pts_;
        Eigen::Vector3d posd = Eigen::Vector3d(point.x, point.y, point.z);
        wp_mu += wi * posd;
        total_wi += wi;
        total_wi_2 += std::pow(wi,2);
    }
    v.mu_ = wp_mu/total_wi;
    // cal sigma of voxel
    for (const auto& point : v.pts_) {
        double wi = std::exp(-(point.covariance(0,0) + point.covariance(1,1) + point.covariance(2,2)));
        //double wi = 1/v.num_pts_;
        Eigen::Vector3d posd = Eigen::Vector3d(point.x, point.y, point.z);
        Eigen::Vector3d pos_mu = posd - v.mu_;
        wp_sigma += wi * (point.covariance + pos_mu * pos_mu.transpose());
    }
    if(v.num_pts_ > 1)v.sigma_ = total_wi / (std::pow(total_wi,2) - total_wi_2) * wp_sigma;
    else v.sigma_ = wp_sigma / total_wi;

    // get infomation of voxel
    v.info_ = v.sigma_.inverse(); 
    v.ndt_estimated_ = true;
    
}

void FastPDNDT::AlignPNDT(const Sophus::SE3d& ItoG_pose, const Sophus::SE3d& RtoI_pose, const Eigen::Matrix<double,3,1>& wI_bg, 
                    const Eigen::Matrix<double,3,1>& velG, Eigen::Matrix<double, 21, 21>& HTVH, Eigen::Matrix<double, 21, 1>& HTVr) {

    assert(grids_.empty() == false);
    std::cout << " pts: " << source_->size() << ", grids: " << grids_.size();

    // cal SE3 T_RtoG
    Sophus::SE3d pose = ItoG_pose * RtoI_pose;

    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    std::vector<bool> effect_pts(index.size(), false);
    std::vector<Eigen::Matrix<double, 3, 21>> jacobians(index.size());
    std::vector<Eigen::Matrix<double, 3, 1>> errors(index.size());
    std::vector<Eigen::Matrix<double, 3, 3>> infos(index.size());
    std::vector<double> weights(index.size());

    int big_ndterror = 0;
    #pragma omp parallel for
    for(int idx = 0;idx < source_->size();idx++){
        Eigen::Vector3d q = Eigen::Vector3d(source_->at(idx).x, source_->at(idx).y, source_->at(idx).z);
        Eigen::Vector3d q_vector = q/q.norm();  // Direction vector
        Eigen::Matrix<double,3,1> qs = pose * q;  

        double ii = source_->at(idx).intensity;
        double wi = std::exp(-(source_->at(idx).covariance(0,0) + source_->at(idx).covariance(1,1) + source_->at(idx).covariance(2,2)));
        // std::cout<<ii<<" "<<wi<<std::endl;

        // Cal qs grid and its nearest neighbor grids
        Eigen::Matrix<int,3,1> key = (qs * options_.inv_voxel_size_).cast<int>();
        auto it = grids_.find(key);
        
        /// Here check whether the Gaussian distribution has been estimated
        if (it != grids_.end() && it->second->second.ndt_estimated_) {
            auto& v = it->second->second;  // voxel
            // qs = R_ItoG * (R_rtoi * q + t_rtoi) + t_ItoG
            Eigen::Matrix<double, 3, 1> e1 = v.mu_ - qs;
            // Pos observed noise and doppler noise, So the noise matrix should be（ sigma_xyz     0_3*1
            //                                                                         0_1*3    sigma_doppler)
    
            Eigen::Matrix<double, 3, 3> info_dndt = Eigen::Matrix<double, 3, 3>::Zero();
            info_dndt = v.info_;

            // check chi2 th
            double res = e1.transpose() * v.info_ * e1;
            if (std::isnan(res) || res > options_.res_outlier_th_) {
                #pragma omp critical
                {
                    effect_pts[idx] = false;
                    big_ndterror++;
                }
                continue;
            }

            // build residual
            Eigen::Matrix<double, 3, 21> J;
            J.setZero();

            // res1 to pIinG
            J.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); 
            // res1 to RItoG
            Eigen::Matrix<double, 3, 1> so3_R_ = ItoG_pose.so3().log();
            Eigen::Matrix<double, 3, 1> so3_R_norm = so3_R_/so3_R_.norm();
            double so3_norm = so3_R_.norm();
            double cot_so3norminv2 = 1.0/tan(so3_norm/2);
            Eigen::Matrix<double, 3, 3> Jaco_R_inv = so3_norm/2 * cot_so3norminv2 * Eigen::Matrix<double, 3, 3>::Identity() + 
                                        (1 - so3_norm/2 * cot_so3norminv2) * so3_R_norm * so3_R_norm.transpose() +
                                        so3_norm/2 * Sophus::SO3d::hat(so3_R_norm);
            J.block<3, 3>(0, 6) = -ItoG_pose.so3().matrix() * Sophus::SO3d::hat(RtoI_pose.so3().matrix() * q + RtoI_pose.translation()) * Jaco_R_inv; 
            // res1 to Rrtoi
            Eigen::Matrix<double, 3, 1> so3_R_rtoi_ = RtoI_pose.so3().log();
            Eigen::Matrix<double, 3, 1> so3_R_rtoi_norm = so3_R_rtoi_/so3_R_rtoi_.norm();
            double so3_normrtoi_ = so3_R_rtoi_.norm();
            double cot_so3normrtoi_inv2 = 1.0/tan(so3_normrtoi_/2);
            Eigen::Matrix<double, 3, 3> Jaco_R_rtoi_inv = so3_normrtoi_/2 * cot_so3normrtoi_inv2 * Eigen::Matrix<double, 3, 3>::Identity() + 
                                                (1-so3_normrtoi_/2 * cot_so3normrtoi_inv2)*so3_R_rtoi_norm*so3_R_rtoi_norm.transpose()+
                                                so3_normrtoi_/2 * Sophus::SO3d::hat(so3_R_rtoi_norm);
            J.block<3 ,3>(0, 15) = -ItoG_pose.so3().matrix() * RtoI_pose.so3().matrix() * Sophus::SO3d::hat(q) * Jaco_R_rtoi_inv; 
            // res1 to prini
            J.block<3, 3>(0, 18) = ItoG_pose.so3().matrix(); 


            #pragma omp critical
            {
                jacobians[idx] = J;
                errors[idx] = e1;
                weights[idx] = wi;
                infos[idx] = info_dndt;
                effect_pts[idx] = true;
            }
        } else {
            #pragma omp critical
            {
                effect_pts[idx] = false;
            }
        }
    }

    // accumulate Jacobian and error
    double total_res = 0;
    int effective_num = 0;

    double weight_sum = std::accumulate(weights.begin(), weights.end(), 0.0);
    double mean_weight = weight_sum / weights.size();

    HTVH.setZero();
    HTVr.setZero();

    auto H_and_err = std::accumulate(
        index.begin(), index.end(), std::pair<Eigen::Matrix<double,21 ,21>, Eigen::Matrix<double,21,1>>(
        Eigen::Matrix<double,21,21>::Zero(), Eigen::Matrix<double,21,1>::Zero()),
        [&jacobians, &errors, &infos, &effect_pts, &total_res, &effective_num, &weights, &mean_weight](const std::pair<Eigen::Matrix<double,21,21>, 
        Eigen::Matrix<double,21,1>>& pre, int idx) -> std::pair<Eigen::Matrix<double,21,21>, Eigen::Matrix<double,21,1>> {
            if (!effect_pts[idx]) {
                return pre;
            } else { 
                total_res += errors[idx].transpose() * infos[idx] * errors[idx];
                effective_num++;
                return std::pair<Eigen::Matrix<double,21,21>, Eigen::Matrix<double,21,1>>(
                        pre.first + jacobians[idx].transpose() * infos[idx] * jacobians[idx] * weights[idx] / mean_weight,
                        pre.second + jacobians[idx].transpose() * infos[idx] * errors[idx] * weights[idx] / mean_weight);
                        // * weights[idx] / mean_weight
            }
        });
    
    std::cout<< " total res: " <<total_res<< ", res_num: " << effective_num<<"(all_num: "<<index.size()
            <<", fail_num: "<<big_ndterror<<")"
            << ", mean res: " << total_res/effective_num<<std::endl;

    HTVH = H_and_err.first;
    HTVr = H_and_err.second;

}


void FastPDNDT::AlignPDNDT(const Sophus::SE3d& ItoG_pose, const Sophus::SE3d& RtoI_pose, const Eigen::Matrix<double,3,1>& wI_bg, 
                    const Eigen::Matrix<double,3,1>& velG, Eigen::Matrix<double, 21, 21>& HTVH, Eigen::Matrix<double, 21, 1>& HTVr) {

    assert(grids_.empty() == false);
    std::cout << " pts: " << source_->size() << ", grids: " << grids_.size();

    // cal SE3 T_RtoG
    Sophus::SE3d pose = ItoG_pose * RtoI_pose;

    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    std::vector<bool> effect_pts(index.size(), false);
    std::vector<Eigen::Matrix<double, 4, 21>> jacobians(index.size());
    std::vector<Eigen::Matrix<double, 4, 1>> errors(index.size());
    std::vector<Eigen::Matrix<double, 4, 4>> infos(index.size());
    std::vector<double> weights(index.size());

    int big_ndterror = 0, big_doperror = 0, no_ndt = 0;
    #pragma omp parallel for
    for(int idx = 0;idx < source_->size();idx++){
        Eigen::Vector3d q = Eigen::Vector3d(source_->at(idx).x, source_->at(idx).y, source_->at(idx).z);
        Eigen::Vector3d q_vector = q/q.norm();  // direction vector
        Eigen::Matrix<double,3,1> qs = pose * q;  

        double esi_doppler = q_vector.dot(RtoI_pose.so3().matrix().transpose() * 
                        (ItoG_pose.so3().matrix().transpose() * velG + Sophus::SO3d::hat(wI_bg) * RtoI_pose.translation()));
        double obs_doppler = -source_->at(idx).doppler;

        double ii = source_->at(idx).intensity;
        double wi = std::exp(-(source_->at(idx).covariance(0,0) + source_->at(idx).covariance(1,1) + source_->at(idx).covariance(2,2)));
        // std::cout<<ii<<" "<<wi<<std::endl;

        // Cal qs grid and its nearest neighbor lattice
        Eigen::Matrix<int,3,1> key = (qs * options_.inv_voxel_size_).cast<int>();
        auto it = grids_.find(key);
        
        /// Here check whether the Gaussian distribution has been estimated
        if (it != grids_.end() && it->second->second.ndt_estimated_) {
            auto& v = it->second->second;  // voxel
            // qs = R_ItoG * (R_rtoi * q + t_rtoi) + t_ItoG
            Eigen::Matrix<double, 3, 1> e1 = v.mu_ - qs;
            // doppler = q/q_vector * R_rtoi.transpose() * (R_ItoG.transpose() * vinG + So3::hat(w_I - bg) * p_rtoi)
            double e2 = obs_doppler - esi_doppler;
            Eigen::Matrix<double, 4, 1> e(e1(0,0), e1(1,0), e1(2,0), e2);
            

            // WPNDT info and dopplernoise are independent，So the noise matrix should be（ sigma_xyz    0_3*1
            //                                                                              0_1*3    sigma_doppler)
            double info_doppler = 1.0/pow(options_.doppler_cov_, 2);
            Eigen::Matrix<double, 4, 4> info_dndt = Eigen::Matrix<double, 4, 4>::Zero();
            info_dndt.block(0,0,3,3) = v.info_;
            info_dndt(3,3) = info_doppler;

            
            //std::cout<<"-------------"<<std::endl;
            //#pragma omp critical
            //{std::cout<<e.transpose()<<std::endl;}
            //{std::cout<<info_dndt<<std::endl;}
            

            // check chi2 th
            double res = e1.transpose() * v.info_ * e1;
            if (std::isnan(res) || res > options_.res_outlier_th_) {
                #pragma omp critical
                {
                    effect_pts[idx] = false;
                    big_ndterror++;
                }
                continue;
            }
            if(fabs(e2) > options_.dop_outlier_th_){
                #pragma omp critical
                {
                    effect_pts[idx] = false;
                    big_doperror++;
                }
                continue;
            }

            // build residual
            Eigen::Matrix<double, 4, 21> J;
            J.setZero();

            // res1 to pIinG
            J.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); 
            // res1 to RItoG
            Eigen::Matrix<double, 3, 1> so3_R_ = ItoG_pose.so3().log();
            Eigen::Matrix<double, 3, 1> so3_R_norm = so3_R_/so3_R_.norm();
            double so3_norm = so3_R_.norm();
            double cot_so3norminv2 = 1.0/tan(so3_norm/2);
            Eigen::Matrix<double, 3, 3> Jaco_R_inv = so3_norm/2 * cot_so3norminv2 * Eigen::Matrix<double, 3, 3>::Identity() + 
                                        (1 - so3_norm/2 * cot_so3norminv2) * so3_R_norm * so3_R_norm.transpose() +
                                        so3_norm/2 * Sophus::SO3d::hat(so3_R_norm);
            J.block<3, 3>(0, 6) = -ItoG_pose.so3().matrix() * Sophus::SO3d::hat(RtoI_pose.so3().matrix() * q + RtoI_pose.translation()) * Jaco_R_inv; 
            // res1 to Rrtoi
            Eigen::Matrix<double, 3, 1> so3_R_rtoi_ = RtoI_pose.so3().log();
            Eigen::Matrix<double, 3, 1> so3_R_rtoi_norm = so3_R_rtoi_/so3_R_rtoi_.norm();
            double so3_normrtoi_ = so3_R_rtoi_.norm();
            double cot_so3normrtoi_inv2 = 1.0/tan(so3_normrtoi_/2);
            Eigen::Matrix<double, 3, 3> Jaco_R_rtoi_inv = so3_normrtoi_/2 * cot_so3normrtoi_inv2 * Eigen::Matrix<double, 3, 3>::Identity() + 
                                                (1-so3_normrtoi_/2 * cot_so3normrtoi_inv2)*so3_R_rtoi_norm*so3_R_rtoi_norm.transpose()+
                                                so3_normrtoi_/2 * Sophus::SO3d::hat(so3_R_rtoi_norm);
            J.block<3 ,3>(0, 15) = -ItoG_pose.so3().matrix() * RtoI_pose.so3().matrix() * Sophus::SO3d::hat(q) * Jaco_R_rtoi_inv; 
            // res1 to prini
            J.block<3, 3>(0, 18) = ItoG_pose.so3().matrix(); 

            // res2 to vinG
            J.block<1, 3>(3, 3) = q_vector.transpose() * (RtoI_pose.so3().matrix().transpose() * ItoG_pose.so3().matrix().transpose());
            // res2 to RItoG
            J.block<1, 3>(3, 6) = q_vector.transpose() * (RtoI_pose.so3().matrix().transpose() 
                                  * Sophus::SO3d::hat(ItoG_pose.so3().matrix().transpose() * velG)) * Jaco_R_inv;
            // res2 to bg
            J.block<1, 3>(3, 9) = q_vector.transpose() * RtoI_pose.so3().matrix().transpose() * Sophus::SO3d::hat(RtoI_pose.translation());
            // res2 to Rrtoi
            J.block<1, 3>(3, 15) = q_vector.transpose() * Sophus::SO3d::hat(RtoI_pose.so3().matrix().transpose() 
                                  * (ItoG_pose.so3().matrix().transpose() * velG + Sophus::SO3d::hat(wI_bg) * RtoI_pose.translation())) * Jaco_R_rtoi_inv;
            // res2 to prtoi
            J.block<1, 3>(3, 18) = q_vector.transpose() * RtoI_pose.so3().matrix().transpose() * Sophus::SO3d::hat(wI_bg);

            #pragma omp critical
            {
                jacobians[idx] = J;
                errors[idx] = e;
                weights[idx] = wi;
                infos[idx] = info_dndt;
                effect_pts[idx] = true;
            }
        } else {
            #pragma omp critical
            {
                no_ndt++;
            }
            double e2 = obs_doppler - esi_doppler;
            if (fabs(e2) > options_.dop_outlier_th_) {
                #pragma omp critical
                {
                    effect_pts[idx] = false;
                    big_doperror++;
                }
                continue;
            }
            Eigen::Matrix<double, 4, 1> e(0.0, 0.0, 0.0, e2);

            double info_doppler = 1.0/pow(options_.doppler_cov_, 2);
            Eigen::Matrix<double, 4, 4> info_dndt = Eigen::Matrix<double, 4, 4>::Zero();
            //info_dndt.block(0,0,3,3) = std::numeric_limits<double>::min() * Eigen::Matrix3d::Identity();
            info_dndt.block(0,0,3,3) = Eigen::Matrix3d::Zero();
            info_dndt(3,3) = info_doppler;

            Eigen::Matrix<double, 4, 21> J;
            J.setZero();
            // res2 to vinG
            J.block<1, 3>(3, 3) = q_vector.transpose() * (RtoI_pose.so3().matrix().transpose() * ItoG_pose.so3().matrix().transpose());
            // res2 to RItoG
            Eigen::Matrix<double, 3, 1> so3_R_ = ItoG_pose.so3().log();
            Eigen::Matrix<double, 3, 1> so3_R_norm = so3_R_/so3_R_.norm();
            double so3_norm = so3_R_.norm();
            double cot_so3norminv2 = 1.0/tan(so3_norm/2);
            Eigen::Matrix<double, 3, 3> Jaco_R_inv = so3_norm/2 * cot_so3norminv2 * Eigen::Matrix<double, 3, 3>::Identity() + 
                                        (1 - so3_norm/2 * cot_so3norminv2) * so3_R_norm * so3_R_norm.transpose() +
                                        so3_norm/2 * Sophus::SO3d::hat(so3_R_norm);
            J.block<1, 3>(3, 6) = q_vector.transpose() * (RtoI_pose.so3().matrix().transpose() 
                                  * Sophus::SO3d::hat(ItoG_pose.so3().matrix().transpose() * velG)) * Jaco_R_inv;
            // res2 to bg
            J.block<1, 3>(3, 9) = q_vector.transpose() * RtoI_pose.so3().matrix().transpose() * Sophus::SO3d::hat(RtoI_pose.translation());
            // res2 to Rrtoi
            Eigen::Matrix<double, 3, 1> so3_R_rtoi_ = RtoI_pose.so3().log();
            Eigen::Matrix<double, 3, 1> so3_R_rtoi_norm = so3_R_rtoi_/so3_R_rtoi_.norm();
            double so3_normrtoi_ = so3_R_rtoi_.norm();
            double cot_so3normrtoi_inv2 = 1.0/tan(so3_normrtoi_/2);
            Eigen::Matrix<double, 3, 3> Jaco_R_rtoi_inv = so3_normrtoi_/2 * cot_so3normrtoi_inv2 * Eigen::Matrix<double, 3, 3>::Identity() + 
                                                (1-so3_normrtoi_/2 * cot_so3normrtoi_inv2)*so3_R_rtoi_norm*so3_R_rtoi_norm.transpose()+
                                                so3_normrtoi_/2 * Sophus::SO3d::hat(so3_R_rtoi_norm);
            J.block<1, 3>(3, 15) = q_vector.transpose() * Sophus::SO3d::hat(RtoI_pose.so3().matrix().transpose() 
                                  * (ItoG_pose.so3().matrix().transpose() * velG + Sophus::SO3d::hat(wI_bg) * RtoI_pose.translation())) * Jaco_R_rtoi_inv;
            // res2 to prtoi
            J.block<1, 3>(3, 18) = q_vector.transpose() * RtoI_pose.so3().matrix().transpose() * Sophus::SO3d::hat(wI_bg);
            
            #pragma omp critical
            {
                jacobians[idx] = J;
                errors[idx] = e;
                weights[idx] = wi;
                infos[idx] = info_dndt;
                effect_pts[idx] = true;
            }
        }
    }
    //);

    // accumulate Jacobian and error
    double total_res = 0;
    int effective_num = 0;

    double weight_sum = std::accumulate(weights.begin(), weights.end(), 0.0);
    double mean_weight = weight_sum / weights.size();

    HTVH.setZero();
    HTVr.setZero();

    Eigen::Matrix<double,21,1> grad = Eigen::Matrix<double,21,1>::Zero();

    auto H_and_err = std::accumulate(
        index.begin(), index.end(), std::pair<Eigen::Matrix<double,21 ,21>, Eigen::Matrix<double,21,1>>(
        Eigen::Matrix<double,21,21>::Zero(), Eigen::Matrix<double,21,1>::Zero()),
        [&jacobians, &errors, &infos, &effect_pts, &total_res, &effective_num, &weights, &mean_weight, &grad](const std::pair<Eigen::Matrix<double,21,21>, 
        Eigen::Matrix<double,21,1>>& pre, int idx) -> std::pair<Eigen::Matrix<double,21,21>, Eigen::Matrix<double,21,1>> {
            if (!effect_pts[idx]) {
                return pre;
            } else { 
                total_res += errors[idx].transpose() * infos[idx] * errors[idx];
                grad = grad + jacobians[idx].transpose() * errors[idx];
                effective_num++;
                return std::pair<Eigen::Matrix<double,21,21>, Eigen::Matrix<double,21,1>>(
                        pre.first + jacobians[idx].transpose() * infos[idx] * jacobians[idx] * weights[idx] / mean_weight,
                        //pre.first + jacobians[idx].transpose() * infos[idx] * jacobians[idx],
                        pre.second + jacobians[idx].transpose() * infos[idx] * errors[idx] * weights[idx] / mean_weight);
                        //pre.second + jacobians[idx].transpose() * infos[idx] * errors[idx]);
                        
            }
        });
    
    std::cout<< " total res: " <<total_res<< ", res_num: " << effective_num<<"(all_num: "<<index.size()
            <<", fail_num: "<<big_ndterror<<"-"<<big_doperror<<"-"<<no_ndt<<")"
            << ", mean res: " << total_res/effective_num<<std::endl;

    HTVH = H_and_err.first;
    HTVr = H_and_err.second;

}

void FastPDNDT::calGrad(const Sophus::SE3d& ItoG, const Sophus::SE3d& RtoI_pose, const Eigen::Matrix<double,3,1>& wI_bg, 
                    const Eigen::Matrix<double,3,1>& velG){

    double true_x = 45.76843918992841;
    double true_y = 55.65619012772704;
    double true_z = 6.128080739272892;
    double true_qx = -0.02979663161985175;
    double true_qy = -0.028729381300031265;
    double true_qz = 0.7723629037811935;
    double true_qw = 0.6338314667614027;

    Eigen::Quaterniond qgt(true_qw, true_qx, true_qy, true_qz);
    Eigen::Matrix3d rgt = qgt.toRotationMatrix();
    Eigen::Vector3d egt = rgt.eulerAngles(2, 1, 0);
    std::cout<<true_x<<" "<<egt(0)/3.1415926*180<<std::endl;

    // Sampling space range
    double x_range = 20.0; // ±2 meters
    double yaw_range_deg = 5.0; // ±10 degrees
    double yaw_range_rad = yaw_range_deg / 180 * 3.141592653;

    // The number of sampling points
    int num_steps = 500;

    // Calculating step size
    double x_step = 2 * x_range / (num_steps - 1);
    double yaw_step =  2 * yaw_range_rad / (num_steps - 1);

    // Generate sampling point
    std::vector<Sophus::SE3d> samples;
    for (int i = 0; i < num_steps; ++i) {
        for (int j = 0; j < num_steps; ++j) {
            double sampled_x = true_x - x_range + i * x_step;
            double sampled_y = true_y;
            double sampled_z = true_z;
            double sampled_yaw = -yaw_range_rad + j * yaw_step;

            // Generating rotation matrix
            Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
            Eigen::Quaterniond quat(0.6338314667614027, -0.02979663161985175, -0.028729381300031265, 0.7723629037811935);
            Eigen::Matrix3d rotation_z_matrix = Eigen::AngleAxisd(sampled_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            rotation = rotation_z_matrix * quat.toRotationMatrix();

            // Generating transformation matrix
            Sophus::SE3d sample_transform(rotation, Eigen::Vector3d(sampled_x, sampled_y, sampled_z));
            samples.push_back(sample_transform);
        }
    }
    
    std::cout<<ItoG.matrix()<<std::endl;
    Sophus::SE3d ii = samples.at(0) * RtoI_pose.inverse();
    std::cout<<ii.matrix()<<std::endl;


    for(int i = 0;i<samples.size();i++){
        Sophus::SE3d pose = samples.at(i);
        Sophus::SE3d ItoG_pose = samples.at(i) * RtoI_pose.inverse();
        
        std::vector<int> index(source_->points.size());
        for (int i = 0; i < index.size(); ++i) {
            index[i] = i;
        }

        std::vector<bool> effect_pts(index.size(), false);
        std::vector<Eigen::Matrix<double, 4, 21>> jacobians(index.size());
        std::vector<Eigen::Matrix<double, 4, 1>> errors(index.size());

        #pragma omp parallel for
        for(int idx = 0;idx < source_->size();idx++){
            Eigen::Vector3d q = Eigen::Vector3d(source_->at(idx).x, source_->at(idx).y, source_->at(idx).z);
            Eigen::Vector3d q_vector = q/q.norm();  
            Eigen::Matrix<double,3,1> qs = pose * q;  

            double esi_doppler = q_vector.dot(RtoI_pose.so3().matrix().transpose() * 
                            (ItoG_pose.so3().matrix().transpose() * velG + Sophus::SO3d::hat(wI_bg) * RtoI_pose.translation()));
            double obs_doppler = -source_->at(idx).doppler;

            Eigen::Matrix<int,3,1> key = (qs * options_.inv_voxel_size_).cast<int>();
            auto it = grids_.find(key);
        
            if (it != grids_.end() && it->second->second.ndt_estimated_) {
                auto& v = it->second->second;  // voxel
                // qs = R_ItoG * (R_rtoi * q + t_rtoi) + t_ItoG
                Eigen::Matrix<double, 3, 1> e1 = v.mu_ - qs;
                // doppler = q/q_vector * R_rtoi.transpose() * (R_ItoG.transpose() * vinG + So3::hat(w_I - bg) * p_rtoi)
                double e2 = obs_doppler - esi_doppler;
                Eigen::Matrix<double, 4, 1> e(e1(0,0), e1(1,0), e1(2,0), e2);
                

                double info_doppler = 1.0/pow(options_.doppler_cov_, 2);
                Eigen::Matrix<double, 4, 4> info_dndt = Eigen::Matrix<double, 4, 4>::Zero();
                info_dndt.block(0,0,3,3) = v.info_;
                info_dndt(3,3) = info_doppler;
            

                // check chi2 th
                double res = e1.transpose() * v.info_ * e1;
                if (std::isnan(res) || res > options_.res_outlier_th_) {
                    #pragma omp critical
                    {
                        effect_pts[idx] = false;
                    }
                    continue;
                }
                if(fabs(e2) > options_.dop_outlier_th_){
                    #pragma omp critical
                    {
                        effect_pts[idx] = false;
                    }
                    continue;
                }

                // build residual
                Eigen::Matrix<double, 4, 21> J;
                J.setZero();

                // res1 to pIinG
                J.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); 
                // res1 to RItoG
                Eigen::Matrix<double, 3, 1> so3_R_ = ItoG_pose.so3().log();
                Eigen::Matrix<double, 3, 1> so3_R_norm = so3_R_/so3_R_.norm();
                double so3_norm = so3_R_.norm();
                double cot_so3norminv2 = 1.0/tan(so3_norm/2);
                Eigen::Matrix<double, 3, 3> Jaco_R_inv = so3_norm/2 * cot_so3norminv2 * Eigen::Matrix<double, 3, 3>::Identity() + 
                                            (1 - so3_norm/2 * cot_so3norminv2) * so3_R_norm * so3_R_norm.transpose() +
                                            so3_norm/2 * Sophus::SO3d::hat(so3_R_norm);
                J.block<3, 3>(0, 6) = -ItoG_pose.so3().matrix() * Sophus::SO3d::hat(RtoI_pose.so3().matrix() * q + RtoI_pose.translation()) * Jaco_R_inv; 
                // res1 to Rrtoi
                Eigen::Matrix<double, 3, 1> so3_R_rtoi_ = RtoI_pose.so3().log();
                Eigen::Matrix<double, 3, 1> so3_R_rtoi_norm = so3_R_rtoi_/so3_R_rtoi_.norm();
                double so3_normrtoi_ = so3_R_rtoi_.norm();
                double cot_so3normrtoi_inv2 = 1.0/tan(so3_normrtoi_/2);
                Eigen::Matrix<double, 3, 3> Jaco_R_rtoi_inv = so3_normrtoi_/2 * cot_so3normrtoi_inv2 * Eigen::Matrix<double, 3, 3>::Identity() + 
                                                    (1-so3_normrtoi_/2 * cot_so3normrtoi_inv2)*so3_R_rtoi_norm*so3_R_rtoi_norm.transpose()+
                                                    so3_normrtoi_/2 * Sophus::SO3d::hat(so3_R_rtoi_norm);
                J.block<3 ,3>(0, 15) = -ItoG_pose.so3().matrix() * RtoI_pose.so3().matrix() * Sophus::SO3d::hat(q) * Jaco_R_rtoi_inv; 
                // res1 to prini
                J.block<3, 3>(0, 18) = ItoG_pose.so3().matrix(); 

                // res2 to vinG
                J.block<1, 3>(3, 3) = q_vector.transpose() * (RtoI_pose.so3().matrix().transpose() * ItoG_pose.so3().matrix().transpose());
                // res2 to RItoG
                J.block<1, 3>(3, 6) = q_vector.transpose() * (RtoI_pose.so3().matrix().transpose() 
                                    * Sophus::SO3d::hat(ItoG_pose.so3().matrix().transpose() * velG)) * Jaco_R_inv;
                // res2 to bg
                J.block<1, 3>(3, 9) = q_vector.transpose() * RtoI_pose.so3().matrix().transpose() * Sophus::SO3d::hat(RtoI_pose.translation());
                // res2 to Rrtoi
                J.block<1, 3>(3, 15) = q_vector.transpose() * Sophus::SO3d::hat(RtoI_pose.so3().matrix().transpose() 
                                    * (ItoG_pose.so3().matrix().transpose() * velG + Sophus::SO3d::hat(wI_bg) * RtoI_pose.translation())) * Jaco_R_rtoi_inv;
                // res2 to prtoi
                J.block<1, 3>(3, 18) = q_vector.transpose() * RtoI_pose.so3().matrix().transpose() * Sophus::SO3d::hat(wI_bg);

                #pragma omp critical
                {
                    jacobians[idx] = J;
                    errors[idx] = e;
                    effect_pts[idx] = true;
                }
                } else {
                    double e2 = obs_doppler - esi_doppler;
                    if (fabs(e2) > options_.dop_outlier_th_) {
                        #pragma omp critical
                        {
                            effect_pts[idx] = false;
                        }
                        continue;
                    }
                    Eigen::Matrix<double, 4, 1> e(0.0, 0.0, 0.0, e2);

                    double info_doppler = 1.0/pow(options_.doppler_cov_, 2);
                    Eigen::Matrix<double, 4, 4> info_dndt = Eigen::Matrix<double, 4, 4>::Zero();
                    //info_dndt.block(0,0,3,3) = std::numeric_limits<double>::min() * Eigen::Matrix3d::Identity();
                    info_dndt.block(0,0,3,3) = Eigen::Matrix3d::Zero();
                    info_dndt(3,3) = info_doppler;

                    Eigen::Matrix<double, 4, 21> J;
                    J.setZero();
                    // res2 to vinG
                    J.block<1, 3>(3, 3) = q_vector.transpose() * (RtoI_pose.so3().matrix().transpose() * ItoG_pose.so3().matrix().transpose());
                    // res2 to RItoG
                    Eigen::Matrix<double, 3, 1> so3_R_ = ItoG_pose.so3().log();
                    Eigen::Matrix<double, 3, 1> so3_R_norm = so3_R_/so3_R_.norm();
                    double so3_norm = so3_R_.norm();
                    double cot_so3norminv2 = 1.0/tan(so3_norm/2);
                    Eigen::Matrix<double, 3, 3> Jaco_R_inv = so3_norm/2 * cot_so3norminv2 * Eigen::Matrix<double, 3, 3>::Identity() + 
                                                (1 - so3_norm/2 * cot_so3norminv2) * so3_R_norm * so3_R_norm.transpose() +
                                                so3_norm/2 * Sophus::SO3d::hat(so3_R_norm);
                    J.block<1, 3>(3, 6) = q_vector.transpose() * (RtoI_pose.so3().matrix().transpose() 
                                        * Sophus::SO3d::hat(ItoG_pose.so3().matrix().transpose() * velG)) * Jaco_R_inv;
                    // res2 to bg
                    J.block<1, 3>(3, 9) = q_vector.transpose() * RtoI_pose.so3().matrix().transpose() * Sophus::SO3d::hat(RtoI_pose.translation());
                    // res2 to Rrtoi
                    Eigen::Matrix<double, 3, 1> so3_R_rtoi_ = RtoI_pose.so3().log();
                    Eigen::Matrix<double, 3, 1> so3_R_rtoi_norm = so3_R_rtoi_/so3_R_rtoi_.norm();
                    double so3_normrtoi_ = so3_R_rtoi_.norm();
                    double cot_so3normrtoi_inv2 = 1.0/tan(so3_normrtoi_/2);
                    Eigen::Matrix<double, 3, 3> Jaco_R_rtoi_inv = so3_normrtoi_/2 * cot_so3normrtoi_inv2 * Eigen::Matrix<double, 3, 3>::Identity() + 
                                                        (1-so3_normrtoi_/2 * cot_so3normrtoi_inv2)*so3_R_rtoi_norm*so3_R_rtoi_norm.transpose()+
                                                        so3_normrtoi_/2 * Sophus::SO3d::hat(so3_R_rtoi_norm);
                    J.block<1, 3>(3, 15) = q_vector.transpose() * Sophus::SO3d::hat(RtoI_pose.so3().matrix().transpose() 
                                        * (ItoG_pose.so3().matrix().transpose() * velG + Sophus::SO3d::hat(wI_bg) * RtoI_pose.translation())) * Jaco_R_rtoi_inv;
                    // res2 to prtoi
                    J.block<1, 3>(3, 18) = q_vector.transpose() * RtoI_pose.so3().matrix().transpose() * Sophus::SO3d::hat(wI_bg);
                    
                    #pragma omp critical
                    {
                        jacobians[idx] = J;
                        errors[idx] = e;
                        effect_pts[idx] = true;
                    }
                }
            }

        auto grad = std::accumulate(
            index.begin(), index.end(), Eigen::Matrix<double,21,1>(Eigen::Matrix<double,21,1>::Zero()),
            [&jacobians, &errors, &effect_pts](const Eigen::Matrix<double,21,1> pre, int idx) -> Eigen::Matrix<double,21,1> 
            {
                if (!effect_pts[idx]) {
                    return pre;
                } else { 
                    return pre + jacobians[idx].transpose() * errors[idx];
                }
            });

        out_printf.precision(6);
        out_printf.setf(std::ios::fixed, std::ios::floatfield);
        Eigen::Matrix3d rotation_matrix = pose.so3().matrix();
        // Euler angles are extracted from the rotation matrix(ZYX))
        Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
        Eigen::Vector3d grad_r = grad.block(6,0,3,1);
        out_printf<<pose.translation()(0)<<" "<<euler_angles(0)/3.1415926*180<<" "<<grad(0)<<" "<<grad_r.norm()<<std::endl;
    }

    
    
}