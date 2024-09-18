#ifndef RISE_ESKF_H
#define RISE_ESKF_H

#include <iomanip>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <boost/filesystem.hpp>

#include "rise_state.h"
#include "estimate_vel.h"
#include "fast_pdndt.h"
#include "radarpcl_view.h"

namespace rise {

class Eskf_Rise {

   public:

    struct Options {

        /// IMU Measurement with zero bias parameters
        double imu_dt_;  // IMU Measuring interval

        /// Radar initialization speed
        double min_radar_vel_zero_; 

        /// Parameters related to initialization
        double max_static_gyro_var;
        double max_static_acce_var;

        Options() : imu_dt_(0.005), min_radar_vel_zero_(0.01),
                    max_static_gyro_var(0.5), max_static_acce_var(0.5){}
    };

    /**
     * Initializing the estimator
     */
    Eskf_Rise(std::shared_ptr<Estimate_Vel> est_v, std::string config_path, std::string output_path, std::string gt_path = ""): radarv_est(est_v){

        Options opts = Options();
        SetInitialConditionsAndNoises(opts, config_path);
        SetOutput(output_path);

        fast_pdndt = std::make_shared<FastPDNDT>(options_pdndt);
        radar_pclviewer = std::make_shared<Radar_PCLView>();
    }

    /**
     * Save imu readings
     */
    void feed_imu_inf(const ImuData imu_data){
        
        imu_deque_.push_back(imu_data);
        std::sort(imu_deque_.begin(),imu_deque_.end());

        auto it0 = imu_deque_.begin();
            while (it0 != imu_deque_.end()) {
            if (imu_deque_.back().timestamp - (*it0).timestamp > 30) {
                it0 = imu_deque_.erase(it0);
            } else {
                it0++;
            }
        }

        std::sort(imu_deque_.begin(),imu_deque_.end());
    }

    /**
     * Store millimeter wave radar readings and start filtering them
     */
    void feed_radar_pointcloud(const double time, const pcl::PointCloud<RadarPointCloudType>::Ptr &radar_scan_raw, const pcl::PointCloud<RadarPointCloudType>::Ptr &radar_scan){

        // estimate egovel and set radardata 
        Eigen::Vector3d vel, velsigma;
        pcl::PointCloud<RadarPointCloudType>::Ptr radarcloud( new pcl::PointCloud<RadarPointCloudType>);
        radarcloud = radar_scan_raw;
        pcl::PointCloud<RadarPointCloudType>::Ptr raw_radarcloud( new pcl::PointCloud<RadarPointCloudType>);
        raw_radarcloud = radar_scan;
        radarv_est->velocity_estimate(time, radarcloud, vel, velsigma);
        RadarData rd;
        rd.timestamp = time;
        rd.radar_scan = radarcloud;
        rd.radar_scan_raw = raw_radarcloud;
        rd.velo = vel;
        rd.velo_sigma = velsigma;
        radar_deque_.push_back(rd);

        std::sort(radar_deque_.begin(),radar_deque_.end());

        /// select IMU data
        if(!radarvel_deque_.empty() && !imu_deque_.empty())
            std::cout<< std::fixed<<std::setprecision(3)<<"r("<<radar_deque_.front().timestamp
            <<","<<radar_deque_.back().timestamp<<")--i( "<< imu_deque_.front().timestamp <<","<<
            imu_deque_.back().timestamp<<")"<<std::endl;
        while(!radar_deque_.empty() && !imu_deque_.empty()&& radar_deque_.at(0).timestamp < imu_deque_.back().timestamp){

            assert(last_radar_time < time);

            if(last_radar_time < 0){
                std::vector<ImuData> imu_data = select_imu_readings(imu_deque_.front().timestamp, radar_deque_.at(0).timestamp);
                bool init_success = ImuInit(radar_deque_.at(0).velo, imu_data, radar_deque_.at(0));
                last_radar_time = radar_deque_.at(0).timestamp;
                if(!init_success){radar_deque_.pop_front();return;}
            }
            
            std::vector<ImuData> imu_data = select_imu_readings(last_radar_time, radar_deque_.at(0).timestamp);

            if(!rise_init_){
                bool init_success = ImuInit(radar_deque_.at(0).velo, imu_data, radar_deque_.at(0));
                last_radar_time = radar_deque_.at(0).timestamp;
                if(!init_success){radar_deque_.pop_front();return;}
            }

            Predict(imu_data);

            //ObserveSpeed(radar_deque_.at(0).timestamp, radar_deque_.at(0).velo, radar_deque_.at(0).velo_sigma, imu_data);
            ObserveDWPNDT(radar_deque_.at(0).timestamp, radar_deque_.at(0), imu_data);

            last_radar_time = radar_deque_.at(0).timestamp;
            radar_deque_.pop_front();

        }

    }

    void saveRadarMap(std::string output_path){
        std::string map_path = output_path + "/radarmap.pcd";
        radar_pclviewer->SaveMap(map_path);
    }

    protected:

    /// Use IMU to initialization, and use Radar to make a rest judgment
    bool ImuInit(const Eigen::Vector3d& vel_radar, const std::vector<ImuData>& imu_data, const RadarData radar_pointcloud);

    /// IMU propagate state
    void Predict(const std::vector<ImuData>& imu_data);

    /// Screening matches the time IMU readings
    std::vector<ImuData> select_imu_readings(double time0, double time1);

    /// Using radar speed observations
    void ObserveSpeed(const double time, const Eigen::Vector3d& vel_radar, const Eigen::Vector3d& vel_radar_sigma, const std::vector<ImuData>& imu_data);
    
    /// filter use pndt and doppler
    void ObserveDWPNDT(const double time, const RadarData radar_pointcloud, const std::vector<ImuData>& imu_data);

    /// Get full state
    RiseState GetRiseState() const { return RiseState(current_time_, R_, p_, v_, bg_, ba_, R_rtoi_,t_rtoi_); }

    /// Obtain SE3 state TGI，TItoG
    Sophus::SE3d GetNominalSE3() const { return Sophus::SE3d(R_, p_); }

    /// Get SE3 IMUextrin state TIR，TRtoI
    Sophus::SE3d GetTimuradarSE3() const { return Sophus::SE3d(R_rtoi_, t_rtoi_); }

    /// Get SE3 radar transformation in the world coordinate system  TGR，TRtoG
    Sophus::SE3d GetTworldradarSE3() const{ return Sophus::SE3d(R_*R_rtoi_, R_*t_rtoi_+p_); }

    private:

    /// Set output path
    void SetOutput(std::string path_traj){

        std::string file_traj = path_traj + "/rise_odom.txt";
        if (boost::filesystem::exists(file_traj)) {
            printf("Traj file exists, deleting old file....\n");
            boost::filesystem::remove(file_traj);
        }
        out_rise.open(file_traj.c_str());
        if (out_rise.fail()) {
            printf("Unable to open output file %s\n",file_traj.c_str());
            std::exit(EXIT_FAILURE);
        }

        std::string cov_diag = path_traj + "/cov_diag.txt";
        if (boost::filesystem::exists(cov_diag)) {
            printf("Traj file exists, deleting old file....\n");
            boost::filesystem::remove(cov_diag);
        }
        out_diag.open(cov_diag.c_str());
        if (out_diag.fail()) {
            printf("Unable to open output file %s\n",cov_diag.c_str());
            std::exit(EXIT_FAILURE);
        }
    }

    // Set parameters and noise
    void SetInitialConditionsAndNoises(Options options, std::string config_path) {

        options_ = options;
        cv::FileStorage fs(config_path, cv::FileStorage::READ);

        // Check whether the file is successfully opened
        if (!fs.isOpened()) {
            std::cout << "Failed to open config file!" << std::endl;
            std::exit(EXIT_FAILURE);
        }
        // Initialization parameter
        double min_radar_vel_zero;
        fs["imuInitVel"]>>min_radar_vel_zero;
        options.min_radar_vel_zero_ = min_radar_vel_zero;

        // Imu noise
        double ea, ew, eba, ebw;
        fs["imuAccNoise"]>>ea;
        fs["imuGyrNoise"]>>ew;
        fs["imuAccBias"]>>eba;
        fs["imuGyrBias"]>>ebw;
        Q_.diagonal() << std::pow(ea,2), std::pow(ea,2), std::pow(ea,2), std::pow(ew,2), std::pow(ew,2), std::pow(ew,2), 
                        std::pow(ebw,2), std::pow(ebw,2), std::pow(ebw,2), std::pow(eba,2), std::pow(eba,2), std::pow(eba,2);

        // Gravity
        double gravity;
        fs["imuGravity"]>>gravity;
        g_<<0, 0, -gravity;

        // 4dradar noise
        double range_cov, azimuth_cov, elevation_cov, doppler_cov;
        fs["rangeCov"]>>range_cov;
        fs["azimuthCov"]>>azimuth_cov;
        fs["elevationCov"]>>elevation_cov;
        fs["dopplerCov"]>>doppler_cov;

        // pdndt settings
        double voxel_size, res_outlier_th, dop_outlier_th, kf_distance, kf_angle_deg, eps;
        int max_iteration, capacity, kf_frame;
        
        fs["voxel_size"]>>voxel_size;
        fs["res_outlier_th"]>>res_outlier_th;
        fs["dop_outlier_th"]>>dop_outlier_th;
        fs["kf_distance"]>>kf_distance;
        fs["kf_angle_deg"]>>kf_angle_deg;
        fs["kf_frame"]>>kf_frame;
        fs["eps"]>>eps;
        fs["max_iteration"]>>max_iteration;
        fs["capacity"]>>capacity;

        options_pdndt.voxel_size_ = voxel_size;
        options_pdndt.res_outlier_th_ = res_outlier_th;
        options_pdndt.dop_outlier_th_ = dop_outlier_th;
        options_pdndt.capacity_ = capacity;
        options_pdndt.kf_distance_ = kf_distance;
        options_pdndt.kf_angle_deg_ = kf_angle_deg;
        options_pdndt.kf_frame_ = kf_frame;
        options_pdndt.eps_ = eps;
        options_pdndt.max_iteration_ = max_iteration;
        options_pdndt.range_cov_ = range_cov;
        options_pdndt.azimuth_cov_ = azimuth_cov;
        options_pdndt.elevation_cov_ = elevation_cov;
        options_pdndt.doppler_cov_ = doppler_cov;

        // Point Update
        double pointupdate_th;
        fs["PointUpdateTh"]>>pointupdate_th;
        options_pdndt.pointupdate_th_ = pointupdate_th;

        // Set the initial covariance
        cov_ = Eigen::Matrix<double,21,21>::Identity() * 1e-6;
        fs["Pos"]>>pos_init_cov;
        fs["Vel"]>>vel_init_cov;
        fs["RollPitch"]>>grav_init_cov;
        fs["Yaw"]>>azimuth_init_cov;
        fs["BiasGyr"]>>bg_init_cov;
        fs["BiasAcc"]>>ba_init_cov;
        fs["RotExt"]>>extrot_init_cov;
        fs["PosExt"]>>extpos_init_cov;

        // Set external parameters
        cv::Mat Trtoi(4, 4, CV_64F);;
        fs["T_Radar_to_Imu"]>>Trtoi;
        Eigen::Matrix<double,4,4> Trtoi_eigen;
        Trtoi_eigen<<Trtoi.at<double>(0,0),Trtoi.at<double>(0,1),Trtoi.at<double>(0,2),Trtoi.at<double>(0,3),
                     Trtoi.at<double>(1,0),Trtoi.at<double>(1,1),Trtoi.at<double>(1,2),Trtoi.at<double>(1,3),
                     Trtoi.at<double>(2,0),Trtoi.at<double>(2,1),Trtoi.at<double>(2,2),Trtoi.at<double>(2,3),
                     Trtoi.at<double>(3,0),Trtoi.at<double>(3,1),Trtoi.at<double>(3,2),Trtoi.at<double>(3,3);
        Eigen::Matrix<double, 3, 3> Rrti = Trtoi_eigen.block(0,0,3,3);
        Eigen::Matrix<double, 3, 1> trti = Trtoi_eigen.block(0,3,3,1);
        Eigen::Quaterniond q_rti = Eigen::Quaterniond(Rrti);
        R_rtoi_ = Sophus::SO3d(q_rti.normalized().toRotationMatrix());
        t_rtoi_ = trti;

        fs.release();
    }

    /// Update the nominal state variable, reset error state
    void UpdateAndReset() {
        p_ += dx_.block<3, 1>(0, 0);
        v_ += dx_.block<3, 1>(3, 0);
        R_ = R_ * Sophus::SO3d::exp(dx_.block<3, 1>(6, 0));

        bg_ += dx_.block<3, 1>(9, 0);
        ba_ += dx_.block<3, 1>(12, 0);
        
        R_rtoi_ = R_rtoi_ * Sophus::SO3d::exp(dx_.block<3, 1>(15, 0));
        t_rtoi_ += dx_.block<3, 1>(18, 0);
        
        ProjectCov();
        dx_.setZero();
    }

    /// Update the nominal state variable
    void Update() {
        p_ += dx_.block<3, 1>(0, 0);
        v_ += dx_.block<3, 1>(3, 0);
        R_ = R_ * Sophus::SO3d::exp(dx_.block<3, 1>(6, 0));

        bg_ += dx_.block<3, 1>(9, 0);
        ba_ += dx_.block<3, 1>(12, 0);
        
        R_rtoi_ = R_rtoi_ * Sophus::SO3d::exp(dx_.block<3, 1>(15, 0));
        t_rtoi_ += dx_.block<3, 1>(18, 0);

    }

    /// The covariance is projected
    void ProjectCov() {
        Eigen::Matrix<double,21,21> J = Eigen::Matrix<double,21,21>::Identity();
        J.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - 0.5 * Sophus::SO3d::hat(dx_.block<3, 1>(6, 0));
        J.block<3, 3>(15, 15) = Eigen::Matrix3d::Identity() - 0.5 * Sophus::SO3d::hat(dx_.block<3, 1>(15, 0));
        cov_ = J * cov_ * J.transpose();
    }

    /// Output the diagonal value of the covariance
    void Output_cov_diag(){
        Eigen::VectorXd diagonal = cov_.diagonal();
        for (int i = 0; i < diagonal.size(); ++i) {
            out_diag << diagonal(i);
            if (i != diagonal.size() - 1) {
                out_diag <<" "; // Add Spaces between elements
            }
        }
        out_diag<<std::endl;
    }

    /// IMU reading interpolate
    static ImuData interpolate_data(const ImuData &imu_1, const ImuData &imu_2, double timestamp) {
        // time-distance lambda
        double lambda = (timestamp - imu_1.timestamp) / (imu_2.timestamp - imu_1.timestamp);
        ImuData data;
        data.timestamp = timestamp;
        data.am = (1 - lambda) * imu_1.am + lambda * imu_2.am;
        data.wm = (1 - lambda) * imu_1.wm + lambda * imu_2.wm;
        return data;
    }

    /// Member variables: Variables that are associated with each instance of a class and accessible from all of its methods.
    /// Velocity estimator
    std::shared_ptr<Estimate_Vel> radarv_est; 

    /// DWPNDT smart pointer
    std::shared_ptr<FastPDNDT> fast_pdndt;
    FastPDNDT::Options options_pdndt;

    /// Point cloud visualization
    std::shared_ptr<Radar_PCLView> radar_pclviewer;

    /// Current state time
    double current_time_ = 0.0; 

    /// Last frame time record
    double last_radar_time = -1;

    /// Nominal state
    Eigen::Vector3d p_ = Eigen::Vector3d::Zero(); //p IinG
    Eigen::Vector3d v_ = Eigen::Vector3d::Zero(); //v IinG
    Sophus::SO3d R_; //R ItoG
    Eigen::Vector3d bg_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d g_{0, 0, -9.8};
    Sophus::SO3d R_rtoi_;
    Eigen::Vector3d t_rtoi_ = Eigen::Vector3d::Zero();

    /// Error state
    Eigen::Matrix<double, 21, 1>  dx_ = Eigen::Matrix<double, 21, 1>::Zero();

    /// Covariance matrix
    Eigen::Matrix<double, 21, 21>  cov_ = Eigen::Matrix<double, 21, 21>::Identity();
    double pos_init_cov, vel_init_cov, azimuth_init_cov, grav_init_cov, ba_init_cov, bg_init_cov, extrot_init_cov, extpos_init_cov;

    /// Process noise and observation noise
    Eigen::Matrix<double, 12, 12>  Q_ = Eigen::Matrix<double, 12, 12>::Zero();
    Eigen::Matrix3d V_obsvel = Eigen::Matrix3d::Zero();

    /// Initialize flag 
    bool rise_init_ = false;  // Whether the initialization succeeds

    /// Data storage
    std::deque<ImuData> imu_deque_;
    std::deque<ImuData> imu_init_; // Store for Imu initialization
    std::deque<RadarVelData> radarvel_deque_;
    std::deque<RadarData> radar_deque_;
    pcl::PointCloud<RadarPointCloudType>::Ptr radar_init_{new pcl::PointCloud<RadarPointCloudType>};
    RiseState last_State;
    std::vector<RiseState> imu_states_;

    /// Configuration in estimator
    Options options_;

    /// Output file stream
    std::ofstream out_rise, out_diag;
};

} // namespace rise

#endif 
