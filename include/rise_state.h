#ifndef RISE_STATE_H
#define RISE_STATE_H

#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>


namespace rise {

class RiseState {

    public:

    RiseState() = default;

    // from time, R, p, v, bg, ba
    explicit RiseState(double time, const Sophus::SO3d& R = Sophus::SO3d(), const Eigen::Matrix<double, 3, 1>& t = Eigen::Matrix<double, 3, 1>::Zero(), const Eigen::Matrix<double, 3, 1>& v = Eigen::Matrix<double, 3, 1>::Zero(),
                        const Eigen::Matrix<double, 3, 1>& bg = Eigen::Matrix<double, 3, 1>::Zero(), const Eigen::Matrix<double, 3, 1>& ba = Eigen::Matrix<double, 3, 1>::Zero(),
                        const Sophus::SO3d& R_rtoi = Sophus::SO3d(),const Eigen::Matrix<double, 3, 1>& t_rtoi = Eigen::Matrix<double, 3, 1>::Zero())
        : timestamp_(time), R_(R), p_(t), v_(v), bg_(bg), ba_(ba), R_rtoi_(R_rtoi),t_rtoi_(t_rtoi) {}

    // from pose and vel
    RiseState(double time, const Sophus::SE3d& pose, const Eigen::Matrix<double, 3, 1>& vel = Eigen::Matrix<double, 3, 1>::Zero())
        : timestamp_(time), R_(pose.so3()), p_(pose.translation()), v_(vel) {}

    /// Change to Sophus
    Sophus::SE3d GetSE3() const { return Sophus::SE3d(R_, p_); }
    Sophus::SE3d GetextSE3() const { return Sophus::SE3d(R_rtoi_, t_rtoi_); }

    /// Get state
    double GetZ() const { return p_(2);}
    Eigen::Matrix<double, 3, 1> GetVel() const { return v_; }
    Eigen::Matrix<double, 3, 1> GetBiasACC() const { return ba_; }
    Eigen::Matrix<double, 3, 1> GetBiasGYR() const { return bg_; }

    /// Print state
    friend std::ostream& operator<<(std::ostream& os, const RiseState& s) {
        os <<"Time: "<<s.timestamp_<< ", p: " << s.p_.transpose() << ", v: " << s.v_.transpose()
           << ", q: " << s.R_.unit_quaternion().coeffs().transpose() << ", bg: " << s.bg_.transpose()
           << ", ba: " << s.ba_.transpose()<< ", q_rtoi: " << s.R_rtoi_.unit_quaternion().coeffs().transpose()
           << ", p_rini: " << s.t_rtoi_.transpose();
        return os;
    }



    protected:

    // Time
    double timestamp_ = 0;  
    // Rotation  
    Sophus::SO3d R_; 
    // Translation 
    Eigen::Matrix<double, 3, 1> p_ = Eigen::Matrix<double, 3, 1>::Zero();
    // Velocity   
    Eigen::Matrix<double, 3, 1> v_ = Eigen::Matrix<double, 3, 1>::Zero(); 
    // Bias of gyro   
    Eigen::Matrix<double, 3, 1> bg_ = Eigen::Matrix<double, 3, 1>::Zero();  
    // Bias of acce
    Eigen::Matrix<double, 3, 1> ba_ = Eigen::Matrix<double, 3, 1>::Zero();  
    // Transform matrix from radar to imu
    Sophus::SO3d R_rtoi_;              
    Eigen::Matrix<double, 3, 1> t_rtoi_ = Eigen::Matrix<double, 3, 1>::Zero();
};

}  // namespace rise

#endif