// RTK 模块按 IMU_Processing.hpp 的方式组织：配置、坐标转换、初始化和观测更新放在一个头文件。
// 地图和滤波状态仍在 camera_init，只有发布点云时转换到 ENU。
#pragma once

#include <GeographicLib/LocalCartesian.hpp>
#include <Eigen/Eigenvalues>
#include <sensor_msgs/NavSatFix.h>
#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>
#include "use-ikfom.hpp"

class RtkProcess
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Filter = esekfom::esekf<state_ikfom, 12, input_ikfom>;
    bool enable = false;
    std::string topic;

    bool load_parameters(ros::NodeHandle& nh)
    {
        nh.param("rtk/enable", enable, false);
        if (!enable) return true;
        nh.param<std::string>("rtk/topic", topic, "/rtk/fix");
        nh.param("rtk/align_distance", align_distance_, 5.0);
        nh.param("rtk/align_min_samples", align_min_samples_, 10);
        nh.param("rtk/max_std", max_std_, 1.0);
        nh.param("rtk/wait_sec", wait_sec, 0.1);
        std::vector<double> translation, rotation;
        nh.getParam("rtk/extrinsic_T", translation);
        nh.getParam("rtk/extrinsic_R", rotation);
        if (translation.size() != 3 || rotation.size() != 9) return false;
        for (int i = 0; i < 3; ++i) lever_arm_[i] = translation[i];
        Eigen::Matrix3d R;
        for (int i = 0; i < 9; ++i) R(i/3, i%3) = rotation[i];
        // 位置观测只用杆臂；旋转外参仍读取并检查，不能将其当成绝对航向。
        return lever_arm_.allFinite() && R.allFinite() &&
               (R.transpose()*R-Eigen::Matrix3d::Identity()).norm() < 1e-6 &&
               std::abs(R.determinant()-1.0) < 1e-6 && !topic.empty() &&
               std::isfinite(align_distance_) && align_distance_ > 0.0 && align_min_samples_ >= 3 &&
               std::isfinite(max_std_) && max_std_ > 0.0 && std::isfinite(wait_sec) && wait_sec >= 0.0;
    }

    // 上游提供测量时刻、经纬度、椭球高和 ENU 协方差；不在此解析 F9P 串口协议。
    void rtk_cbk(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX || msg->header.stamp.isZero() ||
            !std::isfinite(msg->latitude) || std::abs(msg->latitude) >= 90.0 ||
            !std::isfinite(msg->longitude) || std::abs(msg->longitude) > 180.0 ||
            !std::isfinite(msg->altitude)) return;
        if (!origin_inited_)
        {
            local_.Reset(msg->latitude, msg->longitude, msg->altitude);
            origin_inited_ = true;
            ROS_INFO("RTK ENU origin: latitude=%.9f longitude=%.9f altitude=%.3f",
                     msg->latitude, msg->longitude, msg->altitude);
        }
        if (msg->position_covariance_type == sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN) return;
        Measurement m;
        m.stamp = msg->header.stamp.toSec();
        if (!buffer_.empty() && m.stamp <= buffer_.back().stamp) return;
        Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> covariance(msg->position_covariance.data());
        if (!covariance.allFinite() || (covariance-covariance.transpose()).norm() > 1e-6) return;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance);
        if (eigen.info() != Eigen::Success || eigen.eigenvalues().minCoeff() <= 0.0 ||
            eigen.eigenvalues().maxCoeff() > max_std_*max_std_) return;
        std::vector<double> rotation(9);
        local_.Forward(msg->latitude, msg->longitude, msg->altitude,
                       m.position.x(), m.position.y(), m.position.z(), rotation);
        Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> M(rotation.data());
        // NavSatFix 协方差属于测量点的局部 ENU，先旋转到首点 ENU。
        m.covariance = M * covariance * M.transpose();
        for (int i = 0; i < 3; ++i) m.covariance(i,i) = std::max(m.covariance(i,i), 0.0004);
        buffer_.push_back(m);
        if (buffer_.size() > 200) buffer_.pop_front();
    }

    // 当前扫描需要的 RTK 时刻交给原 IMU 传播循环，历史或重复测量不会再次使用。
    std::deque<double> measurement_times(double begin, double end)
    {
        while (!buffer_.empty() && buffer_.front().stamp <= begin) buffer_.pop_front();
        std::deque<double> times;
        for (const auto& m : buffer_)
        {
            if (m.stamp > end) break;
            times.push_back(m.stamp);
        }
        return times;
    }

    void update(double stamp, Filter& kf, bool map_ready)
    {
        if (buffer_.empty() || std::abs(buffer_.front().stamp-stamp) > 1e-6) return;
        const Measurement m = buffer_.front();
        buffer_.pop_front();
        if (!map_ready) return;
        const auto& state = kf.get_x();
        if (!aligned_)
        {
            // 必须用天线轨迹对齐天线轨迹，不能直接拿 IMU 位置替代天线位置。
            Pair pair;
            pair.world = state.pos + state.rot * lever_arm_;
            pair.enu = m.position;
            pair.weight = 1.0 / m.covariance.trace();
            pairs_.push_back(pair);
            if (pairs_.size() > 200) pairs_.pop_front();
            align(Eigen::Vector3d(state.grav[0], state.grav[1], state.grav[2]));
            return;
        }
        z_world_ = R_EW_.transpose() * (m.position-t_EW_);
        R_world_ = R_EW_.transpose() * m.covariance * R_EW_;
        const auto H = jacobian(state);
        const Eigen::Vector3d residual = z_world_ - (state.pos + state.rot*lever_arm_);
        const Eigen::Matrix3d S = H*kf.get_P()*H.transpose() + R_world_;
        Eigen::LDLT<Eigen::Matrix3d> solver(S);
        if (solver.info() != Eigen::Success || !solver.isPositive()) return;
        const double nis = residual.dot(solver.solve(residual));
        if (!std::isfinite(nis) || nis < 0.0 || nis > 11.345) return;
        // 与 LiDAR 更新共享同一个 x/P，通用接口支持三维各向异性噪声。
        updating_ = true;
        kf.update_iterated_dyn_share();
        updating_ = false;
        if (!fusion_started_)
        {
            ROS_INFO("RTK position fusion started");
            fusion_started_ = true;
        }
    }

    void h_model(state_ikfom& state, esekfom::dyn_share_datastruct<double>& data)
    {
        data.valid = true;
        data.z = z_world_;
        data.h = state.pos + state.rot*lever_arm_;
        data.h_x = jacobian(state);
        data.h_v = Eigen::Matrix3d::Identity();
        data.R = R_world_;
    }

    bool aligned() const { return aligned_; }
    bool updating() const { return updating_; }
    Eigen::Vector3d to_enu(const Eigen::Vector3d& world) const { return R_EW_*world + t_EW_; }
    double wait_sec = 0.1;

private:
    struct Measurement
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double stamp;
        Eigen::Vector3d position;
        Eigen::Matrix3d covariance;
    };
    struct Pair
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d world, enu;
        double weight;
    };

    Eigen::Matrix<double,3,23> jacobian(const state_ikfom& state) const
    {
        Eigen::Matrix<double,3,23> H = Eigen::Matrix<double,3,23>::Zero();
        Eigen::Matrix3d lever_skew;
        lever_skew << 0, -lever_arm_.z(), lever_arm_.y(), lever_arm_.z(), 0, -lever_arm_.x(),
                      -lever_arm_.y(), lever_arm_.x(), 0;
        H.block<3,3>(0,0).setIdentity();
        // 右扰动 R_new=R*Exp(delta_theta)，这里求预测观测 h 的导数。
        H.block<3,3>(0,3) = -state.rot.toRotationMatrix()*lever_skew;
        return H;
    }

    void align(const Eigen::Vector3d& gravity)
    {
        if (pairs_.size() < static_cast<std::size_t>(align_min_samples_) || gravity.norm() < 1.0) return;
        // 原工程不保证 camera_init 的 z 轴朝上，先用重力消除倾斜，再求水平偏航。
        const Eigen::Matrix3d level = Eigen::Quaterniond::FromTwoVectors(gravity.normalized(),
            -Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Vector3d mean_world = Eigen::Vector3d::Zero(), mean_enu = Eigen::Vector3d::Zero();
        double total = 0.0, motion_world = 0.0, motion_enu = 0.0;
        for (const auto& pair : pairs_)
        {
            mean_world += pair.weight*pair.world;
            mean_enu += pair.weight*pair.enu;
            total += pair.weight;
            motion_world = std::max(motion_world, (level*(pair.world-pairs_.front().world)).head<2>().norm());
            motion_enu = std::max(motion_enu, (pair.enu-pairs_.front().enu).head<2>().norm());
        }
        if (motion_world < align_distance_ || motion_enu < align_distance_) return;
        mean_world /= total; mean_enu /= total;
        double cross = 0.0, dot = 0.0;
        for (const auto& pair : pairs_)
        {
            const Eigen::Vector3d a = level*(pair.world-mean_world), b = pair.enu-mean_enu;
            cross += pair.weight*(a.x()*b.y()-a.y()*b.x());
            dot += pair.weight*(a.x()*b.x()+a.y()*b.y());
        }
        if (std::hypot(cross,dot) < 1e-9) return;
        const Eigen::Matrix3d rotation = Eigen::AngleAxisd(std::atan2(cross,dot),
            Eigen::Vector3d::UnitZ()).toRotationMatrix()*level;
        const Eigen::Vector3d translation = mean_enu-rotation*mean_world;
        double error = 0.0;
        for (const auto& pair : pairs_)
            error += pair.weight*(pair.enu-rotation*pair.world-translation).squaredNorm();
        if (std::sqrt(error/total) > 0.3) return;
        R_EW_ = rotation; t_EW_ = translation; aligned_ = true;
        pairs_.clear();
        ROS_INFO_STREAM("RTK alignment complete. R_EW=\n" << R_EW_ << "\nt_EW=" << t_EW_.transpose());
    }

    bool origin_inited_ = false, aligned_ = false, updating_ = false, fusion_started_ = false;
    double align_distance_ = 5.0, max_std_ = 1.0;
    int align_min_samples_ = 10;
    Eigen::Vector3d lever_arm_ = Eigen::Vector3d::Zero(), t_EW_ = Eigen::Vector3d::Zero(), z_world_;
    Eigen::Matrix3d R_EW_ = Eigen::Matrix3d::Identity(), R_world_;
    GeographicLib::LocalCartesian local_;
    std::deque<Measurement, Eigen::aligned_allocator<Measurement>> buffer_;
    std::deque<Pair, Eigen::aligned_allocator<Pair>> pairs_;
};
