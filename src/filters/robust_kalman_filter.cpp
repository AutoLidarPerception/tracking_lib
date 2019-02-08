/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "tracking/filters/robust_kalman_filter.h"

#include <ros/ros.h>

#include "common/bounding_box.hpp"
#include "common/types/type.h"

namespace autosense {
namespace tracking {

/********* KF滤波器默认初始化值 *********/
// As weighted correction for KF
bool RobustKalmanFilter::s_use_adaptive_ = true;
/**
 * @brief 预测噪声协方差矩阵
 * | 10 0   0 |
 * | 0  10  0 |
 * | 0  0   10|
 */
Eigen::Matrix3d RobustKalmanFilter::s_propagation_noise_ =
    10 * Eigen::Matrix3d::Identity();
double RobustKalmanFilter::s_measurement_noise_ = 0.4;
double RobustKalmanFilter::s_initial_velocity_noise_ = 5;
// Some kinds of threshold
size_t RobustKalmanFilter::s_measurement_cached_history_size_minimum_ = 3;
size_t RobustKalmanFilter::s_measurement_cached_history_size_maximum_ = 6;
double RobustKalmanFilter::s_association_score_maximum_ = 1.0;
double RobustKalmanFilter::s_breakdown_threshold_maximum_ = 10;

RobustKalmanFilter::RobustKalmanFilter() {
    name_ = "RobustKalmanFilter";
    age_ = 0;
    measurement_cached_history_size_ =
        s_measurement_cached_history_size_minimum_;

    velocity_covariance_ =
        s_initial_velocity_noise_ * Eigen::Matrix3d::Identity();

    // states
    update_quality_ = 1.0;
    breakdown_threshold_ = s_breakdown_threshold_maximum_;

    belief_velocity_ = Eigen::Vector3d::Zero();
    belief_acceleration_gain_ = Eigen::Vector3d::Zero();
    belief_acceleration_ = Eigen::Vector3d::Zero();
}

void RobustKalmanFilter::setUseAdaptive(const bool &use_adaptive) {
    s_use_adaptive_ = use_adaptive;
    ROS_INFO_STREAM("use adaptive of RobustKalmanFilter is "
                    << s_use_adaptive_);
}

bool RobustKalmanFilter::setAssociationScoreMaximum(
    const double &association_score_maximum) {
    if (association_score_maximum > 0) {
        s_association_score_maximum_ = association_score_maximum;
        ROS_INFO_STREAM("association score maximum of RobustKalmanFilter is "
                        << s_association_score_maximum_ << ".");
        return true;
    }
    ROS_ERROR("invalid association score maximum of RobustKalmanFilter!");
    return false;
}

bool RobustKalmanFilter::setBreakdownThresholdMaximum(
    const double &breakdown_threshold_maximum) {
    if (breakdown_threshold_maximum > 0) {
        s_breakdown_threshold_maximum_ = breakdown_threshold_maximum;
        ROS_INFO_STREAM("breakdown threshold maximum of RobustKalmanFilter is "
                        << s_breakdown_threshold_maximum_ << ".");
        return true;
    }
    ROS_ERROR("invalid breakdown threshold maximum of RobustKalmanFilter!");
    return false;
}

/*
 * 初始化KF噪声
 *	initial_velocity_noise 状态初始化噪声
 *	s_propagation_noise_ 预测噪声协方差矩阵
 *		| xy_propagation_noise 		0 			0
 *|
 *		|		0		xy_propagation_noise
 *|
 *		| 		0					0
 *z_propagation_noise		|
 *	s_measurement_noise_ 观测噪声
 */
bool RobustKalmanFilter::initNoiseParams(const double &measurement_noise,
                                         const double &initial_velocity_noise,
                                         const double &xy_propagation_noise,
                                         const double &z_propagation_noise) {
    if (measurement_noise < 0) {
        ROS_ERROR("invalid measurement noise of RobustKalmanFilter!");
        return false;
    }
    if (initial_velocity_noise < 0) {
        ROS_ERROR("invalid intial velocity noise of RobustKalmanFilter!");
        return false;
    }
    if (xy_propagation_noise < 0) {
        ROS_ERROR("invalid xy propagation noise of RobustKalmanFilter!");
        return false;
    }
    if (z_propagation_noise < 0) {
        ROS_ERROR("invalid z propagation noise of RobustKalmanFilter!");
        return false;
    }
    s_measurement_noise_ = measurement_noise;
    s_initial_velocity_noise_ = initial_velocity_noise;
    s_propagation_noise_(0, 0) = xy_propagation_noise;
    s_propagation_noise_(1, 1) = xy_propagation_noise;
    s_propagation_noise_(2, 2) = z_propagation_noise;
    ROS_INFO_STREAM("measurment noise of RobustKalmanFilter is "
                    << s_measurement_noise_);
    ROS_INFO_STREAM("initial velocity noise of RobustKalmanFilter is "
                    << s_initial_velocity_noise_);
    ROS_INFO_STREAM("propagation noise of RobustKalmanFilter is\n"
                    << s_propagation_noise_);
    return true;
}

// @brief initialize the state of filter with trackable object's anchor point
// and velocity 速度初始化为零
// @params[IN] anchor_point: initial anchor point for filtering
// @params[IN] velocity: initial velocity for filtering
// @return nothing
void RobustKalmanFilter::initState(const Eigen::Vector3f &anchor_point,
                                   const Eigen::Vector3f &velocity) {
    update_quality_ = 1.0;
    breakdown_threshold_ = s_breakdown_threshold_maximum_;
    belief_anchor_point_ = anchor_point.cast<double>();
    belief_velocity_ = velocity.cast<double>();
    belief_acceleration_gain_ = Eigen::Vector3d::Zero();
    belief_acceleration_ = Eigen::Vector3d::Zero();
}
//--------------------------------- init -------------------------------------//

//-------------------------- predit & update ---------------------------------//
/// @brief 基于CVM的状态预测 + KF更新预测协方差矩阵(用于更新...)
/// @retval 返回六维状态向量: 箭头锚点(belief_anchor_point_(x, y, z));
//    箭头方向大小(belief_velocity_(x, y, z))
Eigen::VectorXf RobustKalmanFilter::execPredict(const double &time_diff) {
    // Compute predict states 采用 Constant Velocity Model
    Eigen::VectorXf predicted_state;
    predicted_state.resize(6);
    predicted_state(0) =
        belief_anchor_point_(0) + belief_velocity_(0) * time_diff;
    predicted_state(1) =
        belief_anchor_point_(1) + belief_velocity_(1) * time_diff;
    predicted_state(2) =
        belief_anchor_point_(2) + belief_velocity_(2) * time_diff;
    predicted_state(3) = belief_velocity_(0);
    predicted_state(4) = belief_velocity_(1);
    predicted_state(5) = belief_velocity_(2);

    // Compute predicted covariance 更新预测协方差矩阵
    execPropagateNoiseUpdate(time_diff);

    return predicted_state;
}

// 预测协方差矩阵更新(传播)
void RobustKalmanFilter::execPropagateNoiseUpdate(const double &time_diff) {
    // Only propagate tracked motion？？？
    if (age_ <= 0) {
        return;
    }
    velocity_covariance_ += s_propagation_noise_ * time_diff * time_diff;
}

/**
 * @note
 *
 * @param new_object
 * @param old_object
 * @param time_diff
 */
void RobustKalmanFilter::updateWithObservation(
    TrackableObjectConstPtr new_object,
    TrackableObjectConstPtr old_object,
    const double &time_diff) {
    if (time_diff <= DBL_EPSILON) {
        ROS_WARN(
            "[RobustKalmanFilter] Time diff is too limited to updating Robust "
            "Kalman Filter!");
        return;
    }
    // A. Compute update quality if needed
    if (s_use_adaptive_) {
        calcUpdateQuality(new_object, old_object);
    } else {
        update_quality_ = 1.0;
    }

    // B. Compute measurements
    // position
    Eigen::Vector3f measured_anchor_point = new_object->anchor_point;
    // velocity
    Eigen::Vector3f measured_velocity =
        calcMeasuredVelocity(new_object, old_object, time_diff);
    // accleration
    Eigen::Vector3f measured_acceleration =
        calcMeasuredAcceleration(measured_velocity, time_diff);

    // C. Update model, velocity and acceleration 卡尔曼公式
    execVelocityUpdate(measured_anchor_point, measured_velocity, time_diff);
    execAccelerationUpdate(measured_acceleration);

    // D. Maintain measurement history
    // Cache measurement history
    if (history_measured_velocity_.size() >= measurement_cached_history_size_) {
        history_measured_velocity_.pop_front();
        history_time_diff_.pop_front();
    }
    history_measured_velocity_.push_back(measured_velocity);
    history_time_diff_.push_back(time_diff);

    evaluateOnlineCovariance();

    age_++;
}

/**
 * @note
 *  Constant Velocity Model on "belief_velocity_"
 *  Update position: "belief_anchor_point_" += "belief_velocity_" * time_diff
 * @param time_diff
 */
void RobustKalmanFilter::updateWithoutObservation(const double &time_diff) {
    // Only update belief anchor point
    belief_anchor_point_ += belief_velocity_ * time_diff;
    age_++;
}

//----------------------- Adaptive KF update quality -------------------------//
/**
 * @brief update "update_quality_" for adaptive filtering
 * 评估卡尔曼修正质量:[0,1]
 * @note choose the lower one between following 2 strategies
 *  <1> according to Tracker-Observation match association score
 *  <2> according Tracked Object's point number change
 *  原始卡尔曼滤波器更新其状态不区分其测量的质量
 *  然而,质量是滤波噪声的有益提示,可以估计
 *  例如,在关联步骤中计算的距离可以是一个合理的测量质量估计
 *  根据关联质量更新过滤算法的状态,增强了运动估计问题的鲁棒性和平滑度
 */
void RobustKalmanFilter::calcUpdateQuality(TrackableObjectConstPtr new_object,
                                           TrackableObjectConstPtr old_object) {
    // Strategy A: according to association score
    float update_quality_according_association_score =
        calcUpdateQualityAccordingAssociationScore(new_object);
    // Strategy B: according to point number change
    float update_quality_according_point_num_change =
        calcUpdateQualityAccordingPointNumChange(new_object, old_object);
    // Pick a smaller one to control possible filter distraction of noises
    update_quality_ = update_quality_according_association_score;
    if (update_quality_according_association_score >
        update_quality_according_point_num_change) {
        update_quality_ = update_quality_according_point_num_change;
    }
}

// Compute update quality according Tracker-Observation match association score
float RobustKalmanFilter::calcUpdateQualityAccordingAssociationScore(
    TrackableObjectConstPtr new_object) {
    float association_score = new_object->association_score;
    float update_quality = 1;
    if (s_association_score_maximum_ == 0) {
        return update_quality;
    }
    update_quality = 1 - (association_score / s_association_score_maximum_);
    update_quality = update_quality > 1 ? 1 : update_quality;
    update_quality = update_quality < 0 ? 0 : update_quality;
    update_quality = update_quality * update_quality;
    return update_quality;
}

// Compute update quality according point number change
float RobustKalmanFilter::calcUpdateQualityAccordingPointNumChange(
    TrackableObjectConstPtr new_object, TrackableObjectConstPtr old_object) {
    int new_pt_num = new_object->object_ptr->cloud->size();
    int old_pt_num = old_object->object_ptr->cloud->size();
    if (new_pt_num <= 0 || old_pt_num <= 0) {
        return 0;
    }
    float update_quality =
        1 - fabs(new_pt_num - old_pt_num) / std::max(new_pt_num, old_pt_num);
    return update_quality;
}
//----------------------- Adaptive KF update quality -------------------------//

//------ compute measurement(trackable object) velocity&acceleration ---------//
// Compute 2D velocity measurment for filtering
// Obtain robust measurment via observation redundency
/**
 * 观察冗余: 在一系列重复观测中选择速度测量,即滤波算法的输入
 *  包括锚点移位,边界框中心偏移,边界框角点移位等
 *  冗余观测将为滤波测量带来额外的鲁棒性,因为所有观察失败的概率远远小于单次观察失败的概率
 */
Eigen::VectorXf RobustKalmanFilter::calcMeasuredVelocity(
    TrackableObjectConstPtr new_object,
    TrackableObjectConstPtr old_object,
    const double &time_diff) {
    // Observation I: anchor point velocity measurment
    Eigen::Vector3f measured_anchor_point_velocity =
        calcMeasuredAnchorPointVelocity(new_object, old_object, time_diff);
    // Observation II: bbox center velocity measurment
    Eigen::Vector3f measured_bbox_center_velocity =
        calcMeasuredBboxCenterVelocity(new_object, old_object, time_diff);
    // Observation III: bbox corner velocity measurment
    Eigen::Vector3f measured_bbox_corner_velocity =
        calcMeasuredBboxCornerVelocity(new_object, old_object, time_diff);

    std::vector<Eigen::Vector3f> measured_candidates;
    measured_candidates.push_back(measured_anchor_point_velocity);
    measured_candidates.push_back(measured_bbox_center_velocity);
    measured_candidates.push_back(measured_bbox_corner_velocity);
    Eigen::Vector3f measured_velocity =
        selectMeasuredVelocity(measured_candidates);

    return measured_velocity;
}

// Compute 2D anchor point velocity measurment 质心估计速度
Eigen::VectorXf RobustKalmanFilter::calcMeasuredAnchorPointVelocity(
    TrackableObjectConstPtr new_object,
    TrackableObjectConstPtr old_object,
    const double &time_diff) {
    Eigen::Vector3f measured_anchor_point_velocity =
        new_object->anchor_point - old_object->anchor_point;
    measured_anchor_point_velocity /= time_diff;
    measured_anchor_point_velocity(2) = 0.0;
    return measured_anchor_point_velocity;
}

// TODO(gary): Compute 2D bbox center velocity measurment 边框中心估计速度
/// @note predict object size and ground center based on object cloud and
/// previous direction
Eigen::VectorXf RobustKalmanFilter::calcMeasuredBboxCenterVelocity(
    TrackableObjectConstPtr new_object,
    TrackableObjectConstPtr old_object,
    const double &time_diff) {
    Eigen::Vector3d old_dir = old_object->direction.cast<double>();
    Eigen::Vector3d old_size = old_object->size.cast<double>();
    Eigen::Vector3d old_center = old_object->ground_center.cast<double>();

    // predict object size and ground center based on object cloud and previous
    // direction
    Eigen::Vector3d new_size = old_size;
    Eigen::Vector3d new_center = old_center;
    common::bbox::computeBboxSizeCenter<PointICloudPtr>(
        new_object->object_ptr->cloud, old_dir, &new_size, &new_center);

    Eigen::Vector3f measured_bbox_center_velocity_with_old_dir =
        (new_center - old_center).cast<float>();
    measured_bbox_center_velocity_with_old_dir /= time_diff;
    measured_bbox_center_velocity_with_old_dir(2) = 0.0;

    Eigen::Vector3f measured_bbox_center_velocity =
        measured_bbox_center_velocity_with_old_dir;
    Eigen::Vector3f project_dir =
        new_object->anchor_point - old_object->anchor_point;
    if (measured_bbox_center_velocity.dot(project_dir) <= 0) {
        measured_bbox_center_velocity = Eigen::Vector3f::Zero();
    }
    return measured_bbox_center_velocity;
}

// TODO(gary): Compute 2D bbox corner velocity measurment 边框顶点估计速度
Eigen::VectorXf RobustKalmanFilter::calcMeasuredBboxCornerVelocity(
    TrackableObjectConstPtr new_object,
    TrackableObjectConstPtr old_object,
    const double &time_diff) {
    Eigen::Vector3f project_dir =
        new_object->anchor_point - old_object->anchor_point;
    project_dir.normalize();

    Eigen::Vector3d old_dir = old_object->direction.cast<double>();
    Eigen::Vector3d old_size = old_object->size.cast<double>();
    Eigen::Vector3d old_center = old_object->ground_center.cast<double>();

    // predict object size and ground center based on object cloud and previous
    // direction
    Eigen::Vector3d new_size = old_size;
    Eigen::Vector3d new_center = old_center;
    common::bbox::computeBboxSizeCenter<PointICloudPtr>(
        new_object->object_ptr->cloud, old_dir, &new_size, &new_center);
    Eigen::Vector3d ortho_old_dir(-old_dir(1), old_dir(0), 0.0);

    /// @note old and new observation's 4 corners
    Eigen::Vector3d old_bbox_corner_list[4];
    Eigen::Vector3d new_bbox_corner_list[4];
    // corner Forward-Left
    Eigen::Vector3d old_bbox_corner = old_center + old_dir * old_size(0) * 0.5 +
                                      ortho_old_dir * old_size(1) * 0.5;
    Eigen::Vector3d new_bbox_corner = new_center + old_dir * new_size(0) * 0.5 +
                                      ortho_old_dir * new_size(1) * 0.5;
    old_bbox_corner_list[0] = old_bbox_corner;
    new_bbox_corner_list[0] = new_bbox_corner;
    // corner Backward-Left
    old_bbox_corner = old_center - old_dir * old_size(0) * 0.5 +
                      ortho_old_dir * old_size(1) * 0.5;
    new_bbox_corner = new_center - old_dir * new_size(0) * 0.5 +
                      ortho_old_dir * new_size(1) * 0.5;
    old_bbox_corner_list[1] = old_bbox_corner;
    new_bbox_corner_list[1] = new_bbox_corner;
    // corner Forward-Right
    old_bbox_corner = old_center + old_dir * old_size(0) * 0.5 -
                      ortho_old_dir * old_size(1) * 0.5;
    new_bbox_corner = new_center + old_dir * new_size(0) * 0.5 -
                      ortho_old_dir * new_size(1) * 0.5;
    old_bbox_corner_list[2] = old_bbox_corner;
    new_bbox_corner_list[2] = new_bbox_corner;
    // corner Backward-Right
    old_bbox_corner = old_center - old_dir * old_size(0) * 0.5 -
                      ortho_old_dir * old_size(1) * 0.5;
    new_bbox_corner = new_center - old_dir * new_size(0) * 0.5 -
                      ortho_old_dir * new_size(1) * 0.5;
    old_bbox_corner_list[3] = old_bbox_corner;
    new_bbox_corner_list[3] = new_bbox_corner;

    Eigen::Vector3f min_bbox_corner_velocity_on_project_dir =
        Eigen::Vector3f(100, 100, 0);
    float min_bbox_corner_velocity_on_project_dir_gain_norm =
        min_bbox_corner_velocity_on_project_dir.norm();
    for (size_t i = 0u; i < 4; ++i) {
        old_bbox_corner = old_bbox_corner_list[i];
        new_bbox_corner = new_bbox_corner_list[i];
        Eigen::Vector3f bbox_corner_velocity =
            ((new_bbox_corner - old_bbox_corner) / time_diff).cast<float>();
        float bbox_corner_velocity_project_dir_inner_product =
            bbox_corner_velocity(0) * project_dir(0) +
            bbox_corner_velocity(1) * project_dir(1);
        float bbox_corner_velocity_project_dir_angle_cos =
            bbox_corner_velocity_project_dir_inner_product /
            (bbox_corner_velocity.head(2).norm() * project_dir.head(2).norm());
        float bbox_corner_velocity_norm_on_project_dir =
            bbox_corner_velocity.head(2).norm() *
            bbox_corner_velocity_project_dir_angle_cos;
        Eigen::Vector3f bbox_corner_velocity_on_project_dir =
            project_dir * bbox_corner_velocity_norm_on_project_dir;

        bbox_corner_velocity_on_project_dir(2) = 0.0;
        if (bbox_corner_velocity_on_project_dir(0) * project_dir(0) <= 0) {
            bbox_corner_velocity_on_project_dir = Eigen::Vector3f::Zero();
        }
        Eigen::Vector3f bbox_corner_velocity_on_project_dir_gain =
            bbox_corner_velocity_on_project_dir -
            belief_velocity_.cast<float>();
        if (bbox_corner_velocity_on_project_dir_gain.norm() <
            min_bbox_corner_velocity_on_project_dir_gain_norm) {
            min_bbox_corner_velocity_on_project_dir =
                bbox_corner_velocity_on_project_dir;
            min_bbox_corner_velocity_on_project_dir_gain_norm =
                bbox_corner_velocity_on_project_dir_gain.norm();
        }
    }

    return min_bbox_corner_velocity_on_project_dir;
}

// Select measured velocity among candidates(according motion consistency)
Eigen::Vector3f RobustKalmanFilter::selectMeasuredVelocity(
    const std::vector<Eigen::Vector3f> &candidates) {
    // Strategy I: accoridng motion consistency
    return selectMeasuredVelocityAccordingMotionConsistency(candidates);
}

// Select measured velocity among candidates according motion consistency
// 与历史速度预估最接近的值作为速度观测值
/// @note choose candidate with least measured_velocity_gain
Eigen::Vector3f
RobustKalmanFilter::selectMeasuredVelocityAccordingMotionConsistency(
    const std::vector<Eigen::Vector3f> &candidates) {
    if (candidates.size() <= 0) {
        return Eigen::Vector3f::Zero();
    }
    // choose candidate with least measured_velocity_gain
    Eigen::Vector3f measured_velocity = candidates[0];
    Eigen::Vector3f measured_velocity_gain =
        measured_velocity - belief_velocity_.cast<float>();
    for (size_t i = 1; i < candidates.size(); ++i) {
        Eigen::Vector3f candidate_velocity_gain =
            candidates[i] - belief_velocity_.cast<float>();
        if (candidate_velocity_gain.norm() < measured_velocity_gain.norm()) {
            measured_velocity = candidates[i];
            measured_velocity_gain = candidate_velocity_gain;
        }
    }
    return measured_velocity;
}

/**
 * @brief 通过多观测冗余得到的速度量,结合历史速度信息计算加速度
 * @param measured_velocity
 * @param time_diff
 * @return
 */
Eigen::Vector3f RobustKalmanFilter::calcMeasuredAcceleration(
    const Eigen::Vector3f &measured_velocity, const double &time_diff) {
    if (history_measured_velocity_.size() < 3) {
        return Eigen::Vector3f::Zero();
    }
    int history_index = history_measured_velocity_.size() - 3;
    Eigen::Vector3f history_measurement =
        history_measured_velocity_[history_index];
    double accumulated_time_diff = time_diff;
    for (int i = history_index + 1; i < history_measured_velocity_.size();
         ++i) {
        accumulated_time_diff += history_time_diff_[i];
    }
    Eigen::Vector3f measured_acceleration =
        measured_velocity - history_measurement;
    measured_acceleration /= accumulated_time_diff;
    return measured_acceleration;
}
//-------------- compute measurement(trackable object) velocity --------------//

//-------------------------------- Update model ------------------------------//
/**
 * @brief KF update by Velocity and Acceleration separately
 * @param measured_anchor_point
 * @param measured_velocity
 * @param time_diff
 * @result
 *  breakdown_threshold_
 *  belief_anchor_point_, belief_velocity_, belief_acceleration_gain_
 *  velocity_covariance_
 */
void RobustKalmanFilter::execVelocityUpdate(
    const Eigen::VectorXf &measured_anchor_point,
    const Eigen::VectorXf &measured_velocity,
    const double &time_diff) {
    // Compute kalman gain
    Eigen::Matrix3d mat_c = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d mat_q = s_measurement_noise_ * Eigen::Matrix3d::Identity();
    Eigen::Matrix3d mat_k =
        velocity_covariance_ * mat_c.transpose() *
        (mat_c * velocity_covariance_ * mat_c.transpose() + mat_q).inverse();

    // Compute posterior belief
    Eigen::Vector3d measured_anchor_point_d =
        measured_anchor_point.cast<double>();
    Eigen::Vector3d measured_velocity_d = measured_velocity.cast<double>();
    Eigen::Vector3d priori_velocity =
        belief_velocity_ + belief_acceleration_gain_ * time_diff;
    // TODO(gary): 速度卡尔曼系数
    Eigen::Vector3d velocity_gain =
        mat_k * (measured_velocity_d - mat_c * priori_velocity);

    // Breakdown
    calcBreakdownThreshold();
    if (velocity_gain.norm() > breakdown_threshold_) {
        velocity_gain.normalize();
        velocity_gain *= breakdown_threshold_;
    }
    // TODO(gary): 非自适应
    belief_anchor_point_ = measured_anchor_point_d;
    belief_velocity_ = priori_velocity + velocity_gain;
    belief_acceleration_gain_ = velocity_gain / time_diff;
    // Adaptive 自适应更新
    if (s_use_adaptive_) {
        belief_velocity_ -= belief_acceleration_gain_ * time_diff;
        belief_acceleration_gain_ *= update_quality_;
        belief_velocity_ += belief_acceleration_gain_ * time_diff;
    }

    // Compute posterior covariance 更新最佳估计值的噪声分布
    velocity_covariance_ =
        (Eigen::Matrix3d::Identity() - mat_k * mat_c) * velocity_covariance_;
}

// Set breakdown threshold as 2 times of velocity covariance,
//   and smooth it somehow.
void RobustKalmanFilter::calcBreakdownThreshold() {
    breakdown_threshold_ += velocity_covariance_(0, 0) * 2;
    breakdown_threshold_ /= 2;
    if (breakdown_threshold_ > s_breakdown_threshold_maximum_) {
        breakdown_threshold_ = s_breakdown_threshold_maximum_;
    }
}

/**
 * @brief 融合更新质量/故障阈值控制更新增益过度估计的加速度更新 ==>
 * 通过加速度估计更新速度估计
 * @param measured_acceleration
 * @result belief_acceleration_
 */
void RobustKalmanFilter::execAccelerationUpdate(
    const Eigen::VectorXf &measured_acceleration) {
    // Compute kalman gain
    Eigen::Matrix3d mat_c = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d mat_q =
        s_measurement_noise_ * Eigen::Matrix3d::Identity() * 3;
    Eigen::Matrix3d mat_k =
        velocity_covariance_ * mat_c.transpose() *
        (mat_c * velocity_covariance_ * mat_c.transpose() + mat_q).inverse();
    // Compute posterior belief
    Eigen::Vector3d measured_acceleration_d =
        measured_acceleration.cast<double>();
    Eigen::Vector3d acceleration_gain =
        mat_k * (measured_acceleration_d - mat_c * belief_acceleration_);
    // Adaptive
    acceleration_gain *= update_quality_;
    // Breakdown
    float breakdown_threshold = 2;
    if (acceleration_gain.norm() > breakdown_threshold) {
        acceleration_gain.normalize();
        acceleration_gain *= breakdown_threshold;
    }
    // simple add
    belief_acceleration_ += acceleration_gain;
}
//------------------------------- Update model -------------------------------//

/**
 * @brief online velocity "belief_velocity_"'s covariance
 */
void RobustKalmanFilter::evaluateOnlineCovariance() {
    Eigen::Matrix3d online_covariance = Eigen::Matrix3d::Zero();
    int evaluate_window_size =
        history_measured_velocity_.size() >
                s_measurement_cached_history_size_maximum_
            ? history_measured_velocity_.size()
            : s_measurement_cached_history_size_maximum_;
    for (int i = 0; i < evaluate_window_size; ++i) {
        int history_index = history_measured_velocity_.size() - i - 1;
        //残差
        Eigen::Vector3d velocity_residual = Eigen::Vector3d(5, 5, 0);
        if (history_index >= 0) {
            velocity_residual =
                history_measured_velocity_[history_index].cast<double>() -
                belief_velocity_;
        }
        online_covariance(0, 0) += velocity_residual(0) * velocity_residual(0);
        online_covariance(0, 1) += velocity_residual(0) * velocity_residual(1);
        online_covariance(1, 0) += velocity_residual(1) * velocity_residual(0);
        online_covariance(1, 1) += velocity_residual(1) * velocity_residual(1);
    }
    online_velocity_covariance_ = online_covariance / evaluate_window_size;
}
//----------------------------- predit & update ------------------------------//

//------------------------------- get status ---------------------------------//
void RobustKalmanFilter::getState(Eigen::Vector3f *anchor_point,
                                  Eigen::Vector3f *velocity) {
    (*anchor_point) = belief_anchor_point_.cast<float>();
    (*velocity) = belief_velocity_.cast<float>();
}

void RobustKalmanFilter::getState(Eigen::Vector3f *anchor_point,
                                  Eigen::Vector3f *velocity,
                                  Eigen::Vector3f *acceleration) {
    (*anchor_point) = belief_anchor_point_.cast<float>();
    (*velocity) = belief_velocity_.cast<float>();
    (*acceleration) = belief_acceleration_.cast<float>();
}

void RobustKalmanFilter::getAccelerationGain(
    Eigen::Vector3f *acceleration_gain) {
    (*acceleration_gain) = belief_acceleration_gain_.cast<float>();
}

void RobustKalmanFilter::getOnlineCovariance(
    Eigen::Matrix3d *online_covariance) {
    *online_covariance = online_velocity_covariance_;
}
//--------------------------------- get status -------------------------------//
}  // namespace tracking
}  // namespace autosense
