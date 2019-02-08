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
#ifndef TRACKING_INCLUDE_TRACKING_FILTERS_ROBUST_KALMAN_FILTER_H_
#define TRACKING_INCLUDE_TRACKING_FILTERS_ROBUST_KALMAN_FILTER_H_

#include <Eigen/Core>
#include <deque>  // std::deque
#include <vector>

#include "tracking/filters/base_filter.hpp"

namespace autosense {
namespace tracking {

class RobustKalmanFilter : public BaseFilter {
 public:
    RobustKalmanFilter();

    ~RobustKalmanFilter() {}

    //------------------------------ init ------------------------------------//
    // @brief set use adaptive for all the filter objects
    // @params[IN] use_adaptive: flag of whether use adaptive version or not
    // @return nothing
    static void setUseAdaptive(const bool& use_adaptive);

    // @brief set association score maximum for computing update qaulity
    // @params[IN] association_score_maximum: association score maximum
    // @return true if set successfully, otherwise return false
    static bool setAssociationScoreMaximum(
        const double& association_score_maximum);

    // @brief set breakdown threshold maximum for computing breakdown ratio
    // @params[IN] breakdown_threshold_maximum: breakdown threshold maximum
    // @return true if set successfully, otherwise return false
    /**
     * 高斯滤波算法 （Gaussian Filter algorithms）
     *  总是假设它们的高斯分布产生噪声
     *  然而,这种假设可能在运动预估问题中失败,因为其测量的噪声可能来自直方分布
     *  为了克服更新增益的过度估计,在过滤过程中使用故障阈值
     * @param breakdown_threshold_maximum
     * @return
     */
    static bool setBreakdownThresholdMaximum(
        const double& breakdown_threshold_maximum);

    // @brief init initialize parameters for kalman filter
    // @params[IN] measurement_noise: noise of measurement
    // @params[IN] initial_velocity_noise: initial uncertainty of velocity
    // @params[IN] xy_propagation_noise: propagation uncertainty of xy
    // @params[IN] z_propagation_noise: propagation uncertainty of z
    // @return true if set successfully, otherwise return false
    static bool initNoiseParams(const double& measurement_noise,
                                const double& initial_velocity_noise,
                                const double& xy_propagation_noise,
                                const double& z_propagation_noise);

    // @brief initialize the state of filter
    // @params[IN] anchor_point: initial anchor point for filtering
    // @params[IN] velocity: initial velocity for filtering
    // @return nothing
    void initState(const Eigen::Vector3f& anchor_point,
                   const Eigen::Vector3f& velocity);
    //------------------------------ init ------------------------------------//

    //--------------------------- predit & update ----------------------------//
    // @brief predict the state of filter
    // @params[IN] time_diff: time interval for predicting
    // @return predicted states of filtering
    Eigen::VectorXf execPredict(const double& time_diff);

    // @brief update filter with object
    // @params[IN] new_object: new object for current updating
    // @params[IN] old_object: old object for last updating
    // @params[IN] time_diff: time interval from last updating
    // @return nothing
    void updateWithObservation(TrackableObjectConstPtr new_object,
                               TrackableObjectConstPtr old_object,
                               const double& time_diff);

    // @brief update filter without object
    // @params[IN] time_diff: time interval from last updating
    // @return nothing
    void updateWithoutObservation(const double& time_diff);
    //--------------------------- predit & update ----------------------------//

    //--------------------------- get status ---------------------------------//
    // @brief get state of filter
    // @params[OUT] anchor_point: anchor point of current state
    // @params[OUT] velocity: velocity of current state
    // @return nothing
    void getState(Eigen::Vector3f* anchor_point, Eigen::Vector3f* velocity);

    // @brief get state of filter with accelaration
    // @params[OUT] anchor_point: anchor point of current state
    // @params[OUT] velocity: velocity of current state
    // @params[OUT] velocity_accelaration: accelaration of current state
    // @return nothing
    void getState(Eigen::Vector3f* anchor_point,
                  Eigen::Vector3f* velocity,
                  Eigen::Vector3f* accelaration);

    void getAccelerationGain(Eigen::Vector3f* acceleration_gain);
    //--------------------------- get status ---------------------------------//

 protected:
    //---------------------------- predit & update ---------------------------//
    // @brief propagate covariance of filter
    // @params[IN] time_diff: time interval from last updating
    // @return nothing
    void execPropagateNoiseUpdate(const double& time_diff);

    // @brief update filter
    // @params[IN] measured_anchor_point: anchor point of given measurement
    // @params[IN] measured_velocity: velocity of given measurement
    // @params[IN] time_diff: time interval from last updating
    // @return nothing
    void execVelocityUpdate(const Eigen::VectorXf& measured_anchor_point,
                            const Eigen::VectorXf& measured_velocity,
                            const double& time_diff);

    void evaluateOnlineCovariance();

    void execAccelerationUpdate(const Eigen::VectorXf& measured_acceleration);

    // @brief compute breakdown threshold
    // @return nothing
    void calcBreakdownThreshold();

    //---------- compute measurement(trackable object) velocity --------------//
    // @brief compute measured velocity
    // @params[IN] new_object: new object for current updating
    // @params[IN] old_object: old object for last updating
    // @params[IN] time_diff: time interval from last updating
    // @return measured velocity
    Eigen::VectorXf calcMeasuredVelocity(TrackableObjectConstPtr new_object,
                                         TrackableObjectConstPtr old_object,
                                         const double& time_diff);

    // @brief compute measured anchor point velocity
    // @params[IN] new_object: new object for current updating
    // @params[IN] old_object: old object for last updating
    // @params[IN] time_diff: time interval from last updating
    // @return measured anchor point elocity
    Eigen::VectorXf calcMeasuredAnchorPointVelocity(
        TrackableObjectConstPtr new_object,
        TrackableObjectConstPtr old_object,
        const double& time_diff);

    // @brief compute measured bbox center velocity
    // @params[IN] new_object: new object for current updating
    // @params[IN] old_object: old object for last updating
    // @params[IN] time_diff: time interval from last updating
    // @return measured bbox center velocity
    Eigen::VectorXf calcMeasuredBboxCenterVelocity(
        TrackableObjectConstPtr new_object,
        TrackableObjectConstPtr old_object,
        const double& time_diff);

    // @brief compute measured bbox corner velocity
    // @params[IN] new_object: new object for current updating
    // @params[IN] old_object: old object for last updating
    // @params[IN] time_diff: time interval from last updating
    // @return measured bbox corner velocity
    Eigen::VectorXf calcMeasuredBboxCornerVelocity(
        TrackableObjectConstPtr new_object,
        TrackableObjectConstPtr old_object,
        const double& time_diff);

    // @brief select measured velocity among candidates
    // @params[IN] candidates: candidates of measured velocity
    // @return selected measurement of velocity
    Eigen::Vector3f selectMeasuredVelocity(
        const std::vector<Eigen::Vector3f>& candidates);

    // @brief select measured velocity among candidates according motion
    // consistency
    // @params[IN] candidates: candidates of measured velocity
    // @return selected measurement of velocity
    Eigen::Vector3f selectMeasuredVelocityAccordingMotionConsistency(
        const std::vector<Eigen::Vector3f>& candidates);

    Eigen::Vector3f calcMeasuredAcceleration(
        const Eigen::Vector3f& measured_velocity, const double& time_diff);
    //---------- compute measurement(trackable object) velocity --------------//

    //---------------------- Adaptive KF update quality ----------------------//
    // @brief compute update quality for adaptive filtering
    // @params[IN] new_object: new object for current updating
    // @params[IN] old_object: old object for last updating
    // @reutrn nothing
    void calcUpdateQuality(TrackableObjectConstPtr new_object,
                           TrackableObjectConstPtr old_object);

    // @brief compute update quality by using association score
    // @params[IN] new_object: new object for current updating
    // @return upate quality according association score
    float calcUpdateQualityAccordingAssociationScore(
        TrackableObjectConstPtr new_object);

    // @brief compute update quality by using association score
    // @params[IN] old_object: old object for last updaitng
    // @params[IN] new_object: new object for current updating
    // @return update quality according point number change
    float calcUpdateQualityAccordingPointNumChange(
        TrackableObjectConstPtr new_object, TrackableObjectConstPtr old_object);
    //---------------------- Adaptive KF update quality ----------------------//
    //---------------------------- predit & update ---------------------------//

    // @brief get online covariance of filter
    // @params[OUT] online_covariance: online covariance
    // @return noting
    void getOnlineCovariance(Eigen::Matrix3d* online_covariance);

 protected:
    // adaptive
    static bool s_use_adaptive_;

    static double s_association_score_maximum_;

    // parameters
    // 预测不确定性的噪声协方差矩阵
    static Eigen::Matrix3d s_propagation_noise_;
    // 观测噪声
    static double s_measurement_noise_;
    // 状态初始化噪声
    static double s_initial_velocity_noise_;
    static double s_breakdown_threshold_maximum_;

    static size_t s_measurement_cached_history_size_minimum_;
    static size_t s_measurement_cached_history_size_maximum_;

    size_t measurement_cached_history_size_;

    // 滤波器存活帧数
    int age_;

    // filter history
    std::deque<Eigen::Vector3f> history_measured_velocity_;
    std::deque<double> history_time_diff_;

    // filter covariances
    // 速度协方差=噪声协方差*(time_diff^2) 预测噪声协方差矩阵,通过
    Eigen::Matrix3d velocity_covariance_;
    /// @note "belief_velocity_" online convariance
    Eigen::Matrix3d online_velocity_covariance_;

    /// @note Adaptive on accleration "belief_acceleration_gain_"
    double update_quality_;
    /// @note Adaptive on Kalman gain's norm(square root of Euclidean) threshold
    double breakdown_threshold_;

    // filter states
    Eigen::Vector3d belief_anchor_point_;
    Eigen::Vector3d belief_velocity_;
    Eigen::Vector3d belief_acceleration_;
    Eigen::Vector3d belief_acceleration_gain_;
};  // class RobustKalmanFilter

}  // namespace tracking
}  // namespace autosense

#endif  // TRACKING_INCLUDE_TRACKING_FILTERS_ROBUST_KALMAN_FILTER_H_
