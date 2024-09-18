#include "estimate_vel.h"

using namespace rise;

void Estimate_Vel::velocity_estimate(double time, pcl::PointCloud<RadarPointCloudType>::Ptr &raw, Eigen::Vector3d& v_radar, Eigen::Vector3d& sigma_v_radar){

    pcl::PointCloud<RadarPointCloudType>::Ptr radar_scan(new pcl::PointCloud<RadarPointCloudType>);
    radar_scan = raw;
    //set Matrix and Vector to save radar point normalize coordinates and doppler
    Eigen::MatrixXd H_normlize(radar_scan->size(), 3);
    Eigen::VectorXd b_doppler(radar_scan->size());

    for(size_t i = 0;i < radar_scan->size();i++){

        const double range = Eigen::Vector3d(radar_scan->at(i).x, radar_scan->at(i).y, radar_scan->at(i).z).norm();

        H_normlize.row(i) = Eigen::Vector3d(radar_scan->at(i).x/range, radar_scan->at(i).y/range, radar_scan->at(i).z/range);
        b_doppler(i) = -radar_scan->at(i).doppler;
    }

    // when this point remain stationary 
    // (normx,normy,normz).dot(vx_radar,vy_radar,vz_radar) = v_doppler

    const Eigen::MatrixXd HTH = H_normlize.transpose() * H_normlize;
    const Eigen::VectorXd b = b_doppler;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(HTH);
    double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);

    if(cond > 1000)v_radar = H_normlize.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    else v_radar = (HTH).ldlt().solve(H_normlize.transpose() * b);

    const Eigen::VectorXd e = H_normlize * v_radar - b;
    const Eigen::MatrixXd C = (e.transpose() * e).x() * (HTH).inverse() / (H_normlize.rows() - 3);
    sigma_v_radar  = Eigen::Vector3d(C(0, 0), C(1, 1), C(2, 2));
    sigma_v_radar  = sigma_v_radar.array();
    if (sigma_v_radar.x() >= 0.0 && sigma_v_radar.y() >= 0.0 && sigma_v_radar.z() >= 0.0)
        sigma_v_radar = sigma_v_radar.array().sqrt();
    else
        {sigma_v_radar  = Eigen::Vector3d(-1, -1, -1);printf("Failure at solve vel, Sigma of vel is wrong!\n");}

    std::cout << std::fixed << std::setprecision(5);
    std::cout << "\033[34m";
    std::cout<<"[EST_VEL]"<<time<<"-radar vel: "<<v_radar.transpose()<<"-sigma: "<<sigma_v_radar.transpose();  
    std::cout << "\033[0m" << std::endl;     
    out_risevel.precision(9);
    out_risevel.setf(std::ios::fixed, std::ios::floatfield);
    out_risevel<<time<<" "<<v_radar(0)<<" "<<v_radar(1)<<" "<<v_radar(2)<<" "<<v_radar.norm()<<" "
               <<Eigen::Vector2d(v_radar(0),v_radar(1)).norm()<<" "<<sigma_v_radar(0)<<" "<<sigma_v_radar(1)
               <<" "<<sigma_v_radar(2)<<std::endl;                      

}

void Estimate_Vel::velocity_estimate_Ransac(pcl::PointCloud<RadarPointCloudType>::Ptr &raw, pcl::PointCloud<RadarPointCloudType>::Ptr &output, 
                    Eigen::Vector3d& v_radar_inliner, Eigen::Vector3d& sigma_v_radar_inliner){

    pcl::PointCloud<RadarPointCloudType>::Ptr radar_scan(new pcl::PointCloud<RadarPointCloudType>);
    radar_scan = raw;
    //set Matrix and Vector to save radar point normalize coordinates and doppler
    Eigen::MatrixXd H_normlize(radar_scan->size(), 3);
    Eigen::VectorXd b_doppler(radar_scan->size());

    for(size_t i = 0;i < radar_scan->size();i++){

        const double range = Eigen::Vector3d(radar_scan->at(i).x, radar_scan->at(i).y, radar_scan->at(i).z).norm();

        H_normlize.row(i) = Eigen::Vector3d(radar_scan->at(i).x/range, radar_scan->at(i).y/range, radar_scan->at(i).z/range);
        b_doppler(i) = -radar_scan->at(i).doppler;
    }

    // when this point remain stationary 
    // (normx,normy,normz).dot(vx_radar,vy_radar,vz_radar) = v_doppler
    // (cos(phi)cos(theta),cos(phi)sin(theta),sin(phi)).dot(vx_radar,vy_radar,vz_radar) = v_doppler

    // Do ransac
    std::vector<uint> inlier_idx_best;
    std::vector<uint> idx(H_normlize.rows());
    for (uint k = 0; k < H_normlize.rows(); ++k) idx[k] = k;

    std::random_device rd;
    std::mt19937 g(rd());

    uint N_ransac_points = (uint)H_normlize.rows() * 0.05;
    uint ransac_iter_ = uint((std::log(1.0 - 0.995))/std::log(1.0 - std::pow(1.0 - 0.005, N_ransac_points)));
    printf("ransac_iter num is %d\n", ransac_iter_);

    double init_th_inliner = 0.05;
    
    if (H_normlize.rows() >= N_ransac_points){

        for (uint k = 0; k < ransac_iter_; ++k){

            std::shuffle(idx.begin(), idx.end(), g);

            Eigen::MatrixXd H_normlize_ransac(N_ransac_points, 3);
            Eigen::VectorXd b_doppler_ransac(N_ransac_points);

            Eigen::Vector3d v_radar_ransac, sigma_v_radar_ransac;

            for (uint i = 0; i < N_ransac_points; ++i) {
                H_normlize_ransac.row(i) = H_normlize.row(idx.at(i));
                b_doppler_ransac(i) = b_doppler(idx.at(i));
            }

            const Eigen::MatrixXd HTH = H_normlize_ransac.transpose() * H_normlize_ransac;
            const Eigen::VectorXd b = b_doppler_ransac;

            Eigen::JacobiSVD<Eigen::MatrixXd> svd(HTH);
            double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);

            if(cond > 1000)v_radar_ransac = H_normlize_ransac.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
            else v_radar_ransac = (HTH).ldlt().solve(H_normlize_ransac.transpose() * b);

            const Eigen::VectorXd e = H_normlize_ransac * v_radar_ransac - b;
            const Eigen::MatrixXd C = (e.transpose() * e).x() * (HTH).inverse() / (H_normlize_ransac.rows() - 3);
            sigma_v_radar_ransac  = Eigen::Vector3d(C(0, 0), C(1, 1), C(2, 2));
            sigma_v_radar_ransac  = sigma_v_radar_ransac.array();
            if (sigma_v_radar_ransac.x() >= 0.0 && sigma_v_radar_ransac.y() >= 0.0 && sigma_v_radar_ransac.z() >= 0.0){
                sigma_v_radar_ransac = sigma_v_radar_ransac.array().sqrt();

                // Calculate the inliner and outliner points
                const Eigen::VectorXd err = (b_doppler - H_normlize * v_radar_ransac).array().abs();
                std::vector<uint> inlier_idx;
                std::vector<uint> outlier_idx;
                for (uint j = 0; j < err.rows(); ++j){
                    
                    if (fabs(err(j)) <= fabs(init_th_inliner * b_doppler(j)))
                        inlier_idx.emplace_back(j);
                    else
                        outlier_idx.emplace_back(j);
                    
                }

                if ( float(outlier_idx.size())/(inlier_idx.size()+outlier_idx.size()) > 0.10)continue;

                printf("Inlier number: %ld, Outlier number: %ld, outlier Ratio: %.2f\n", 
                        inlier_idx.size(), outlier_idx.size(), float(outlier_idx.size())/(inlier_idx.size()+outlier_idx.size()));

                if (inlier_idx.size() > inlier_idx_best.size()){
                    inlier_idx_best = inlier_idx;
                }

            }
            else{
                sigma_v_radar_ransac  = Eigen::Vector3d(-1, -1, -1);
                printf("Failure at solve vel Ransac 1 time\n");
            }
        }
    }
    if(inlier_idx_best.size() > 0.9 * H_normlize.rows()){
        Eigen::MatrixXd H_normlize_inliner(inlier_idx_best.size(), 3);
        Eigen::VectorXd b_doppler_inliner(inlier_idx_best.size());

        
        RadarPointCloudType radarp;
        for (uint i = 0; i < inlier_idx_best.size(); ++i) {
            H_normlize_inliner.row(i) = H_normlize.row(inlier_idx_best.at(i));
            b_doppler_inliner(i) = b_doppler(inlier_idx_best.at(i));
            radarp.x = (*raw)[inlier_idx_best.at(i)].x;
            radarp.y = (*raw)[inlier_idx_best.at(i)].y;
            radarp.z = (*raw)[inlier_idx_best.at(i)].z;
            radarp.intensity = (*raw)[inlier_idx_best.at(i)].intensity;
            radarp.doppler = (*raw)[inlier_idx_best.at(i)].doppler;
            output->points.push_back(radarp);
        }

        const Eigen::MatrixXd HTH_inliner = H_normlize_inliner.transpose() * H_normlize_inliner;
        const Eigen::VectorXd b_inliner = b_doppler_inliner;

        Eigen::JacobiSVD<Eigen::MatrixXd> svd_inliner(HTH_inliner);
        double cond_inliner = svd_inliner.singularValues()(0) / svd_inliner.singularValues()(svd_inliner.singularValues().size() - 1);

        if(cond_inliner > 1000)v_radar_inliner = H_normlize_inliner.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_inliner);
        else v_radar_inliner = (HTH_inliner).ldlt().solve(H_normlize_inliner.transpose() * b_inliner);

        const Eigen::VectorXd e_inliner = H_normlize_inliner * v_radar_inliner - b_inliner;
        const Eigen::MatrixXd C_inliner = (e_inliner.transpose() * e_inliner).x() * (HTH_inliner).inverse() / (H_normlize_inliner.rows() - 3);
        sigma_v_radar_inliner  = Eigen::Vector3d(C_inliner(0, 0), C_inliner(1, 1), C_inliner(2, 2));
        sigma_v_radar_inliner  = sigma_v_radar_inliner.array();
        if (sigma_v_radar_inliner.x() >= 0.0 && sigma_v_radar_inliner.y() >= 0.0 && sigma_v_radar_inliner.z() >= 0.0){
            sigma_v_radar_inliner = sigma_v_radar_inliner.array().sqrt();
        }
        else{
            sigma_v_radar_inliner  = Eigen::Vector3d(-1, -1, -1);
            printf("Failure at solve vel Ransac, Sigma of vel is wrong!\n");
        }

        std::cout << std::fixed << std::setprecision(5);
        std::cout << "\033[33m";
        std::cout<<"[EST_VEL_RANSAC]radar vel: "<<v_radar_inliner.transpose();  
        std::cout << "\033[0m" << std::endl;     
    }else{
        output = raw;
        v_radar_inliner = Eigen::Vector3d(-1, -1, -1);
        sigma_v_radar_inliner = Eigen::Vector3d(-1, -1, -1);
    }                    

}