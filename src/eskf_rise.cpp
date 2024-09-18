#include "eskf_rise.h"
#include "math_ops.h"

using namespace rise;
using namespace rise::math;

bool Eskf_Rise::ImuInit(const Eigen::Vector3d& vel_radar, const std::vector<ImuData>& imu_data, const RadarData radar_pointcloud){

    
    std::vector<ImuData> imudata_ = imu_data;
    std::sort(imudata_.begin(),imudata_.end());

    if(vel_radar.norm() < options_.min_radar_vel_zero_){
        imu_init_.insert(imu_init_.end(), imu_data.begin(), imu_data.end());
        *radar_init_ += *radar_pointcloud.radar_scan;
        std::sort(imu_init_.begin(),imu_init_.end());
        printf("[Init]Radar vel: %.3f, insert new imu readings....\n",vel_radar.norm());
        return false;
    }

    if (imu_init_.size() < 200) {
        printf("[Init]Radar vel: %.3f, imu readings is not enough....\n",vel_radar.norm());
        return false;
    }

    // Calculate the mean and variance
    Eigen::Vector3d mean_gyro, mean_acce;
    Eigen::Vector3d cov_gyro, cov_acce;
    ComputeMeanAndCovDiag(imu_init_, mean_gyro, cov_gyro, [](const ImuData& imu) { return imu.wm; });
    ComputeMeanAndCovDiag(imu_init_, mean_acce, cov_acce, [](const ImuData& imu) { return imu.am; });

    // Get z axis, which aligns with -g (z_in_G=0,0,1)
    Eigen::Vector3d z_axis = mean_acce / mean_acce.norm();

    // Create an x_axis
    Eigen::Vector3d e_1(1, 0, 0);

    // Make x_axis perpendicular to z
    Eigen::Vector3d x_axis = e_1 - z_axis * z_axis.transpose() * e_1;
    x_axis = x_axis / x_axis.norm();

    // Get z from the cross product of these two
    Eigen::Vector3d y_axis = SKEW_SYM_MATRIX(z_axis) * x_axis;

    // From these axes get rotation
    Eigen::Matrix3d R_GtoI;
    R_GtoI.block(0, 0, 3, 1) = x_axis;
    R_GtoI.block(0, 1, 3, 1) = y_axis;
    R_GtoI.block(0, 2, 3, 1) = z_axis;

    R_ = Sophus::SO3d(R_GtoI.transpose());
    

    // Recalculate the accelerometer covariance
    ComputeMeanAndCovDiag(imu_init_, mean_acce, cov_acce, [=](const ImuData& imu) { return imu.am + R_GtoI * g_; });
    
    // Inspect IMU noise
    if (cov_gyro.norm() > options_.max_static_gyro_var) {
        printf("[Init]Gyroscope measurement noise %.3f is too high.\n",cov_gyro.norm());
        std::exit(EXIT_FAILURE);
        return false;
    }

    if (cov_acce.norm() > options_.max_static_acce_var) {
        printf("[Init]Accelerometer measurement noise %.3f is too high.\n",cov_acce.norm());
        std::exit(EXIT_FAILURE);
        return false;
    }

    // Estimate measurement noise and zero bias
    bg_ = mean_gyro;
    ba_ = mean_acce;
    // v_ = R_.matrix() * R_rtoi_.matrix() * vel_radar;

    cov_.block(0,0,3,3) = pos_init_cov * Eigen::Matrix3d::Identity();
    cov_.block(3,3,3,3) = vel_init_cov * Eigen::Matrix3d::Identity();
    cov_.block(6,6,3,3) = grav_init_cov * Eigen::Matrix3d::Identity();
    cov_(8,8) = azimuth_init_cov;
    // cov_.block(6,6,3,3) = cov_gyro(2) * Eigen::Matrix3d::Identity();
    
    cov_.block(9,9,3,3) = bg_init_cov * Eigen::Matrix3d::Identity();
    cov_.block(12,12,3,3) = ba_init_cov * Eigen::Matrix3d::Identity();
    cov_.block(15,15,3,3) = extrot_init_cov * Eigen::Matrix3d::Identity();
    cov_.block(18,18,3,3) = extpos_init_cov * Eigen::Matrix3d::Identity();

    current_time_ = imu_init_.back().timestamp;

    Eskf_Rise::Output_cov_diag();
    std::cout<<"[ESKF_INIT]init radar point size: "<<radar_init_->size()<<std::endl;
   
    Eskf_Rise::fast_pdndt->setSource(radar_init_);
    Eskf_Rise::fast_pdndt->IsKeyframe(GetTworldradarSE3());
    // Eskf_Rise::radar_pclviewer->UpdateMap(GetTworldradarSE3(), radar_init_);
    last_State = GetRiseState();

    printf("Imu init success, cost_time = %.3f\n",imu_init_.back().timestamp - imu_init_.front().timestamp);
    std::cout<<"[ESKF_INIT]"<<GetRiseState()<<std::endl;
    std::cout<<"[INIT_TmatrixRtoG]"<<std::endl;
    std::cout<<GetTworldradarSE3().matrix()<<std::endl;
    std::cout<<"[INIT_TmatrixItoG]"<<std::endl;
    std::cout<<GetNominalSE3().matrix()<<std::endl;

    out_rise.precision(12);
    out_rise.setf(std::ios::fixed, std::ios::floatfield);
    out_rise<<current_time_<<" "<<GetTworldradarSE3().translation()(0)<<" "<<GetTworldradarSE3().translation()(1)<<" "
           <<GetTworldradarSE3().translation()(2)<<" "<<GetTworldradarSE3().so3().unit_quaternion().coeffs()(0)<<" "
           <<GetTworldradarSE3().so3().unit_quaternion().coeffs()(1)<<" "<<GetTworldradarSE3().so3().unit_quaternion().coeffs()(2)<<" "
           <<GetTworldradarSE3().so3().unit_quaternion().coeffs()(3)<<std::endl;

    rise_init_ = true;
    return true;
}


void Eskf_Rise::Predict(const std::vector<ImuData>& imu_data){

    /// IMU State prediction
    std::vector<ImuData> imudata_ = imu_data;
    std::sort(imudata_.begin(),imudata_.end());

    /// Store as Result of propagation
    imu_states_.clear();

    for (auto &id : imu_data) {

        assert(id.timestamp >= current_time_);

        double dt = id.timestamp - current_time_;
        if (dt > (5 * options_.imu_dt_) || dt < 0) {
            // Wrong time interval
            printf("\033[31mError time %.3f(dt), %.3f(imutime) - %.3f(statetime)\033[0m\n", dt, id.timestamp, current_time_);
            // continue;
        }

        // Nominal state propagate
        Eigen::Vector3d new_p = p_ + v_ * dt + 0.5 * (R_ * (id.am - ba_)) * dt * dt + 0.5 * g_ * dt * dt;
        Eigen::Vector3d new_v = v_ + R_ * (id.am - ba_) * dt + g_ * dt;
        Sophus::SO3d new_R = R_ * Sophus::SO3d::exp((id.wm - bg_) * dt);

        R_ = new_R;
        v_ = new_v;
        p_ = new_p;
        // The remaining state dimensions remain unchanged

        // Error state propagate
        // Computes the motion process Jacobian matrix F
        Eigen::Matrix<double,21,21> F = Eigen::Matrix<double,21,21>::Identity();        
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;                         // p to v
        F.block<3, 3>(3, 6) = -R_.matrix() * Sophus::SO3d::hat(id.am - ba_) * dt;       // v to theta
        F.block<3, 3>(3, 12) = -R_.matrix() * dt;                                       // v to ba
        F.block<3, 3>(6, 6) = Sophus::SO3d::exp(-(id.wm - bg_) * dt).matrix();          // theta to theta
        F.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity() * dt;                        // theta to bg

        // mean and cov prediction
        dx_ = F * dx_;  // You don't have to count in this line of work，dx_ should be zero after the reset

        // Computes the process noise matrix G
        Eigen::Matrix<double, 21, 12> G = Eigen::Matrix<double, 21, 12>::Zero();
        G.block<3, 3>(3, 0) =  -R_.matrix() * dt;
        G.block<3, 3>(6, 3) =  -Eigen::Matrix3d::Identity() * dt;
        G.block<3, 3>(9, 9) =  Eigen::Matrix3d::Identity() * dt;
        G.block<3, 3>(12, 6) =  Eigen::Matrix3d::Identity() * dt;

        cov_ = F * cov_.eval() * F.transpose() + G * Q_.eval() * G.transpose();


        current_time_ = id.timestamp;

        imu_states_.emplace_back(GetRiseState());
    }
}

void Eskf_Rise::ObserveSpeed(const double time, const Eigen::Vector3d& vel_radar, const Eigen::Vector3d& vel_radar_sigma, 
                const std::vector<ImuData>& imu_data){

    assert(time == current_time_);
    assert(imu_data.back().timestamp == current_time_);

    Eigen::Matrix<double, 3, 1> wi = imu_data.back().wm;
    Eigen::Matrix<double, 3, 1> wI_bg = wi - bg_;

    // Use 3D speed，H(3x21)
    // res 2 vinG
    Eigen::Matrix<double, 3, 21> H = Eigen::Matrix<double, 3, 21>::Zero();
    H.block<3, 3>(0, 3) = R_rtoi_.matrix().transpose() * R_.matrix().transpose();
    // res 2 RItoG
    Eigen::Matrix<double, 3, 1> so3_R_ = R_.log();
    Eigen::Matrix<double, 3, 1> so3_R_norm = so3_R_/so3_R_.norm();
    double so3_norm = so3_R_.norm();
    double cot_so3norminv2 = 1.0/tan(so3_norm/2);
    Eigen::Matrix<double, 3, 3> Jaco_R_inv = so3_norm/2 * cot_so3norminv2 * Eigen::Matrix<double, 3, 3>::Identity() + 
                                        (1 - so3_norm/2 * cot_so3norminv2) * so3_R_norm * so3_R_norm.transpose() +
                                        so3_norm/2 * Sophus::SO3d::hat(so3_R_norm);
    H.block<3, 3>(0, 6) = R_rtoi_.matrix().transpose() * Sophus::SO3d::hat(R_.matrix().transpose() * v_) * Jaco_R_inv;
    // res 2 bg
    H.block<3, 3>(0, 9) = R_rtoi_.matrix().transpose() * Sophus::SO3d::hat(t_rtoi_);
    // res 2 RRtoI
    Eigen::Matrix<double, 3, 1> so3_R_rtoi_ = R_rtoi_.log();
    Eigen::Matrix<double, 3, 1> so3_R_rtoi_norm = so3_R_rtoi_/so3_R_rtoi_.norm();
    double so3_normrtoi_ = so3_R_rtoi_.norm();
    double cot_so3normrtoi_inv2 = 1.0/tan(so3_normrtoi_/2);
    Eigen::Matrix<double, 3, 3> Jaco_R_rtoi_inv = so3_normrtoi_/2 * cot_so3normrtoi_inv2 * Eigen::Matrix<double, 3, 3>::Identity() + 
                                                (1-so3_normrtoi_/2 * cot_so3normrtoi_inv2)*so3_R_rtoi_norm*so3_R_rtoi_norm.transpose()+
                                                so3_normrtoi_/2 * Sophus::SO3d::hat(so3_R_rtoi_norm);
    H.block<3, 3>(0, 15) = Sophus::SO3d::hat(R_rtoi_.matrix().transpose() * (R_.matrix().transpose() * v_ + Sophus::SO3d::hat(wI_bg) * t_rtoi_))
                           * Jaco_R_rtoi_inv;
    // res 2 trtoi
    H.block<3, 3>(0, 18) = R_rtoi_.matrix().transpose() * Sophus::SO3d::hat(wI_bg);

    /// Kalman Gain
    V_obsvel.diagonal() = Eigen::Vector3d(0.001, 0.001, 0.001);
    Eigen::Matrix<double, 21, 3> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + V_obsvel).inverse();

    /// update dx
    dx_ = K * (vel_radar - R_rtoi_.matrix().transpose() * (R_.matrix().transpose() * v_ + Sophus::SO3d::hat(wI_bg) * t_rtoi_));

    /// update cov
    cov_ = (Eigen::Matrix<double,21,21>::Identity() - K * H) * cov_;

    Eskf_Rise::UpdateAndReset();


    dx_.setZero();

    std::cout << "\033[32m";
    std::cout << std::fixed << std::setprecision(3);
    std::cout<<"[ESKF_VEL]"<<GetRiseState()<<std::endl;
    std::cout << "\033[0m" << std::endl; 

    out_rise.precision(12);
    out_rise.setf(std::ios::fixed, std::ios::floatfield);
    out_rise<<current_time_<<" "<<GetTworldradarSE3().translation()(0)<<" "<<GetTworldradarSE3().translation()(1)<<" "
           <<GetTworldradarSE3().translation()(2)<<" "<<GetTworldradarSE3().so3().unit_quaternion().coeffs()(0)<<" "
           <<GetTworldradarSE3().so3().unit_quaternion().coeffs()(1)<<" "<<GetTworldradarSE3().so3().unit_quaternion().coeffs()(2)<<" "
           <<GetTworldradarSE3().so3().unit_quaternion().coeffs()(3)<<std::endl;

    Eskf_Rise::Output_cov_diag();
}


std::vector<ImuData> Eskf_Rise::select_imu_readings(double time0, double time1){
    // Our vector imu readings
  std::vector<ImuData> prop_data;

  printf("\033[33m[Select Imu]time0: %.3f, time1: %.3f, Imu seqtime(%.3f--%.3f)\n\033[0m", time0,time1,imu_deque_.front().timestamp,imu_deque_.back().timestamp);

  // Ensure we have some measurements in the first place!
  if (imu_deque_.empty()) {
      printf("select_imu_readings(): No IMU measurements!!!\n");
    return prop_data;
  }

  // Loop through and find all the needed measurements to propagate with
  // Note we split measurements based on the given state time, and the update timestamp
  for (size_t i = 0; i < imu_deque_.size() - 1; i++) {

    // START OF THE INTEGRATION PERIOD
    if (imu_deque_.at(i + 1).timestamp > time0 && imu_deque_.at(i).timestamp < time0) {
      ImuData data = Eskf_Rise::interpolate_data(imu_deque_.at(i), imu_deque_.at(i + 1), time0);
      prop_data.push_back(data);
      continue;
    }

    // MIDDLE OF INTEGRATION PERIOD
    if (imu_deque_.at(i).timestamp >= time0 && imu_deque_.at(i + 1).timestamp <= time1) {
      prop_data.push_back(imu_deque_.at(i));
      continue;
    }

    // END OF THE INTEGRATION PERIOD
    if (imu_deque_.at(i + 1).timestamp > time1) {
      if (imu_deque_.at(i).timestamp > time1 && i == 0) {
        break;
      } else if (imu_deque_.at(i).timestamp > time1) {
        ImuData data = Eskf_Rise::interpolate_data(imu_deque_.at(i - 1), imu_deque_.at(i), time1);
        prop_data.push_back(data);
      } else {
        prop_data.push_back(imu_deque_.at(i));
      }
      if (prop_data.at(prop_data.size() - 1).timestamp != time1) {
        ImuData data = Eskf_Rise::interpolate_data(imu_deque_.at(i), imu_deque_.at(i + 1), time1);
        prop_data.push_back(data);
      }
      break;
    }
  }

  // Check that we have at least one measurement to propagate with
  if (prop_data.empty()) {
      printf("select_imu_readings(): No IMU measurements to propagate with (%d of 2)!!!\n",(int)prop_data.size());
    return prop_data;
  }

  // Loop through and ensure we do not have an zero dt values
  // This would cause the noise covariance to be Infinity
  for (size_t i = 0; i < prop_data.size() - 1; i++) {
    if (std::abs(prop_data.at(i + 1).timestamp - prop_data.at(i).timestamp) < 1e-12) {
        printf("select_imu_readings(): Zero DT between IMU reading %d and %d, removing it!\n", (int)i,(int)(i + 1));
      prop_data.erase(prop_data.begin() + i);
      i--;
    }
  }

  // Check that we have at least one measurement to propagate with
  if (prop_data.size() < 2) {
      printf("select_imu_readings(): No enough IMU measurements to propagate with (%d of 2)!!!\n" ,(int)prop_data.size());
    return prop_data;
  }

  std::sort(prop_data.begin(),prop_data.end());
  // Success
  printf("prop data time0: %.3f(%.3f),time1: %.3f(%.3f)\n",time0,prop_data.front().timestamp,time1,prop_data.back().timestamp);
  return prop_data;
}



void Eskf_Rise::ObserveDWPNDT(const double time, const RadarData radar_pointcloud, const std::vector<ImuData>& imu_data){

    assert(time == current_time_);
    assert(imu_data.back().timestamp == current_time_);

    Eigen::Matrix<double, 3, 1> vr = radar_pointcloud.velo;
    Eigen::Matrix<double, 3, 1> wi = imu_data.back().wm;
    Sophus::SO3d start_R = R_;
    Sophus::SO3d start_Rrtoi = R_rtoi_;
    Eigen::Matrix<double, 21, 1> HTVr;
    Eigen::Matrix<double, 21, 21> HTVH;
    Eigen::Matrix<double, 21, 21> Pk, Qk;

    Eskf_Rise::fast_pdndt->setSource(radar_pointcloud.radar_scan);
    
    for(int iter = 0;iter < Eskf_Rise::fast_pdndt->options_.max_iteration_;iter++){

      std::cout << std::fixed << std::setprecision(7);
      std::cout<<"ieskf_pdndt iter: "<<iter;
      Eigen::Matrix<double, 3, 1> wI_bg = wi - bg_;
      Eskf_Rise::fast_pdndt->AlignPDNDT(GetNominalSE3(), GetTimuradarSE3(), wI_bg, v_, HTVH, HTVr);
      //Eskf_Rise::fast_pdndt->AlignPNDT(GetNominalSE3(), GetTimuradarSE3(), wI_bg, v_, HTVH, HTVr);
      
      // Project cov (Projection when the linearized point changes during iteration)
      Eigen::Matrix<double, 21, 21> J = Eigen::Matrix<double, 21, 21>::Identity();
      J.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - 0.5 * Sophus::SO3d::hat((start_R.inverse() * R_).log());
      J.block<3, 3>(15, 15) = Eigen::Matrix3d::Identity() - 0.5 * Sophus::SO3d::hat((start_Rrtoi.inverse() * R_rtoi_).log());
      Pk = J * cov_ * J.transpose() + (1e-15 * Eigen::Matrix<double ,21 ,21>::Identity());

      // dx = K * res = (P.inv + H.trans * V.inv *H).inv * H.trans * V.inv * res
      Qk = (Pk.inverse() + HTVH + (1e-15 * Eigen::Matrix<double ,21 ,21>::Identity())).inverse(); 
      dx_ = Qk * HTVr;

      if (dx_.hasNaN()) {
        std::cout << " dx_ Matrix contains NaN values." << std::endl;
        std::cout << "Qk: " << std::endl;
        std::cout << Qk << std::endl;
        std::cout << "HTVH: " << std::endl;
        std::cout << HTVH << std::endl;
        std::cout << "HTVr: " << std::endl;
        std::cout << HTVr << std::endl;
        std::cout << "Pk: " << std::endl;
        std::cout << Pk << std::endl;
        std::exit(EXIT_FAILURE);
      } 

      // xt = xk-1 + dxk
      RiseState x_k_1 = GetRiseState();
      Eskf_Rise::Update();
      RiseState x_k = GetRiseState();

      // Update radar point uncertain
      bool update = Eskf_Rise::fast_pdndt->SourceUpdateUncertain(x_k_1, x_k, wi, cov_, vr);

      // convergence
      if (dx_.norm() < Eskf_Rise::fast_pdndt->options_.eps_) {
            std::cout << std::fixed << std::setprecision(7);
            std::cout<<", dx norm: "<< dx_.norm()<<std::endl;
            break;
      }else{std::cout<<std::endl;}

    }

    /// add pcl to submap
    Eskf_Rise::fast_pdndt->IsKeyframe(GetTworldradarSE3());

    /// visualization
    Eskf_Rise::radar_pclviewer->UpdateMap(GetTworldradarSE3(), radar_pointcloud.radar_scan_raw);
    // Eskf_Rise::radar_pclviewer->UpdateMap(GetTworldradarSE3(), Eskf_Rise::fast_pdndt->getSource());

    /// update cov
    cov_ = (Eigen::Matrix<double, 21, 21>::Identity() - Qk * HTVH) * Pk;

    /// project cov(Projection when resetting error state)
    Eigen::Matrix<double, 21, 21> J = Eigen::Matrix<double, 21, 21>::Identity();
    J.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - 0.5 * Sophus::SO3d::hat((start_R.inverse() * R_).log());
    J.block<3, 3>(15, 15) = Eigen::Matrix3d::Identity() - 0.5 * Sophus::SO3d::hat((start_Rrtoi.inverse() * R_rtoi_).log());
    cov_ = J * cov_ * J.transpose();

    dx_.setZero();

    std::cout << "\033[32m";
    std::cout << std::fixed << std::setprecision(3);
    std::cout<<"[ESKF_DWPNDT]"<<GetRiseState()<<std::endl;
    std::cout << "\033[0m" << std::endl;

    out_rise.precision(12);
    out_rise.setf(std::ios::fixed, std::ios::floatfield);
    out_rise<<current_time_<<" "<<GetTworldradarSE3().translation()(0)<<" "<<GetTworldradarSE3().translation()(1)<<" "
           <<GetTworldradarSE3().translation()(2)<<" "<<GetTworldradarSE3().so3().unit_quaternion().coeffs()(0)<<" "
           <<GetTworldradarSE3().so3().unit_quaternion().coeffs()(1)<<" "<<GetTworldradarSE3().so3().unit_quaternion().coeffs()(2)<<" "
           <<GetTworldradarSE3().so3().unit_quaternion().coeffs()(3)<<std::endl;

    Eskf_Rise::Output_cov_diag();
}