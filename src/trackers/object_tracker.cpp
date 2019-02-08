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
#include "tracking/trackers/object_tracker.hpp"
#include <ros/ros.h>

#include "common/bounding_box.hpp"
#include "common/geometry.hpp"
#include "tracking/filters/robust_kalman_filter.h"

namespace autosense {
namespace tracking {

FilterType ObjectTracker::s_filter_method_ = ROBUST_KALMAN_FILTER;
int ObjectTracker::s_tracker_cached_history_size_maximum_ = 5;
double ObjectTracker::s_acceleration_noise_maximum_ = 5;
double ObjectTracker::s_speed_noise_maximum_ = 0.4;

/**
 * @brief 基于可追踪障碍物创建并初始化障碍物追踪器
 *      初始化锚点为观测锚点
 *      初始化速度,加速度为0
 *  Initialize filter
 *  Initialize tracker info
 * @param obj
 * @note Apollo's doc
 *  obj: 经过CNN检测-->Min_Box包裹-->ConstructTrackedObjects
 * 构造的当前帧的一个可追踪障碍物
 */
ObjectTracker::ObjectTracker(TrackableObjectPtr obj, IdType tracker_id) {
    // Initialize filter
    Eigen::Vector3f initial_anchor_point = obj->anchor_point;
    Eigen::Vector3f initial_velocity = Eigen::Vector3f::Zero();
    if (s_filter_method_ == ROBUST_KALMAN_FILTER) {
        filter_ = new RobustKalmanFilter();
    } else {
        filter_ = new RobustKalmanFilter();
        ROS_WARN(
            "invalid filter method! default filter (RobustKalmanFilter) in "
            "use!");
    }
    filter_->initState(initial_anchor_point, initial_velocity);

    // Initialize tracker info
    idx_ = tracker_id;
    age_ = 1;
    total_visible_count_ = 1;
    consecutive_invisible_count_ = 0;
    period_ = 0.0;
    current_object_ = obj;

    is_static_hypothesis_ = false;
    belief_anchor_point_ = initial_anchor_point;
    belief_velocity_ = initial_velocity;
    belief_velocity_accelaration_ = Eigen::Vector3f::Zero();
    // NEED TO NOTICE: All the states would be collected mainly based on states
    // of tracked object. Thus, update tracked object when you update the state
    // of track !!!!!
    obj->velocity = belief_velocity_;

    // TODO(gary): Initialize object direction with its lane direction
    // obj->direction = obj->lane_direction;
}

ObjectTracker::~ObjectTracker() {
    if (filter_) {
        delete filter_;
        filter_ = nullptr;
    }
}

bool ObjectTracker::setFilterMethod(const std::string &filter_method_name) {
    if (filter_method_name == "robust_kalman_filter") {
        s_filter_method_ = ROBUST_KALMAN_FILTER;
        ROS_INFO_STREAM("filter method of object tracker is "
                        << filter_method_name);
        return true;
    }
    ROS_ERROR("invalid filter method name of object tracker!");
    return false;
}

bool ObjectTracker::setTrackerCachedHistorySizeMaximum(
    const int &tracker_cached_history_size_maximum) {
    if (tracker_cached_history_size_maximum > 0) {
        s_tracker_cached_history_size_maximum_ =
            tracker_cached_history_size_maximum;
        ROS_INFO_STREAM(
            "tracker cached history size maximum of object tracker is "
            << s_tracker_cached_history_size_maximum_);
        return true;
    }
    ROS_ERROR("invalid tracker cached history size maximum of object tracker!");
    return false;
}

bool ObjectTracker::setSpeedNoiseMaximum(const double &speed_noise_maximum) {
    if (speed_noise_maximum > 0) {
        s_speed_noise_maximum_ = speed_noise_maximum;
        ROS_INFO_STREAM("speed noise maximum of object tracker is "
                        << s_speed_noise_maximum_);
        return true;
    }
    ROS_ERROR("invalid speed noise maximum of object tracker!");
    return false;
}

bool ObjectTracker::setAccelerationNoiseMaximum(
    const double &acceleration_noise_maximum) {
    if (acceleration_noise_maximum > 0) {
        s_acceleration_noise_maximum_ = acceleration_noise_maximum;
        ROS_INFO_STREAM("acceleration noise maximum of object tracker is "
                        << s_acceleration_noise_maximum_);
        return true;
    }
    ROS_ERROR("invalid acceleration noise maximum of object tracker!");
    return false;
}

//--------------------------- predict & update -------------------------------//
// 每一个物体追踪器->Predict 包括对应的KF预测
// @brief predict the state of track
// @params[IN] time_diff: time interval for predicting
// @return predicted states of track
Eigen::VectorXf ObjectTracker::execPredict(const double &time_diff) {
    // Get the predict of filter
    // 返回六维状态向量: 箭头锚点(belief_anchor_point_(x, y, z));
    // 箭头方向大小(belief_velocity_(x, y, z))
    // 基于CVM的状态预测 + KF更新: 预测协方差矩阵 [KF预测]
    Eigen::VectorXf filter_predict = filter_->execPredict(time_diff);

    // TODO(gary): filter_->execPredict()里面也进行了相似的更新
    // Get the predict of track (update tracked object when you update the state
    // of track !!!!!) [追踪器预测]
    Eigen::VectorXf tracker_predict = filter_predict;
    tracker_predict(0) =
        belief_anchor_point_(0) + belief_velocity_(0) * time_diff;
    tracker_predict(1) =
        belief_anchor_point_(1) + belief_velocity_(1) * time_diff;
    tracker_predict(2) =
        belief_anchor_point_(2) + belief_velocity_(2) * time_diff;
    tracker_predict(3) = belief_velocity_(0);
    tracker_predict(4) = belief_velocity_(1);
    tracker_predict(5) = belief_velocity_(2);

    return tracker_predict;
}

/**
 * @brief
 * @param time_diff
 * @return
 */
Eigen::VectorXf ObjectTracker::getExpectedState(const double &time_diff) {
    /// @note 采用 Constant Velocity Model, also in Robust Kalman Filter
    Eigen::VectorXf expected_state;
    expected_state.resize(6);
    expected_state(0) =
        belief_anchor_point_(0) + belief_velocity_(0) * time_diff;
    expected_state(1) =
        belief_anchor_point_(1) + belief_velocity_(1) * time_diff;
    expected_state(2) =
        belief_anchor_point_(2) + belief_velocity_(2) * time_diff;
    expected_state(3) = belief_velocity_(0);
    expected_state(4) = belief_velocity_(1);
    expected_state(5) = belief_velocity_(2);

    return expected_state;
}

/**
 * @param new_object
 * @param time_diff
 * @result
 *  update object's state:
 *      new_object->anchor_point
 *      new_object->velocity
 *  update object's history list: history_objects_
 */
void ObjectTracker::updateWithObservation(TrackableObjectPtr object_obsved,
                                          const double &time_diff) {
    // A. update object tracker
    // A.1 update filter
    filter_->updateWithObservation(object_obsved, current_object_, time_diff);

    filter_->getState(&belief_anchor_point_, &belief_velocity_);

    // NEED TO NOTICE: All the states would be collected mainly based on states
    // of tracked object. Thus, update tracked object when you update the state
    // of track !!!!!
    object_obsved->anchor_point = belief_anchor_point_;
    object_obsved->velocity = belief_velocity_;

    belief_velocity_accelaration_ =
        (object_obsved->velocity - current_object_->velocity) / time_diff;
    // A.2 update track info
    age_++;
    total_visible_count_++;
    consecutive_invisible_count_ = 0;
    period_ += time_diff;
    // A.3 update history
    int history_size = history_objects_.size();
    if (history_size >= s_tracker_cached_history_size_maximum_) {
        history_objects_.pop_front();
    }
    history_objects_.push_back(current_object_);
    current_object_ = object_obsved;

    // B. smooth tracking object
    // B.1 smooth velocity
    smoothTrackerVelocity(time_diff);
    // B.2 smooth orientation
    smoothTrackerOrientation();
}

/**
 * @brief update object tracker without observation but CVM
 * @param time_diff
 */
void ObjectTracker::updateWithoutObservation(const double &time_diff) {
    TrackableObjectPtr new_obj(new TrackableObject());
    new_obj->clone(*current_object_);

    // based on CVM
    Eigen::Vector3f predicted_shift = belief_velocity_ * time_diff;

    // A. update tracking object
    new_obj->anchor_point = current_object_->anchor_point + predicted_shift;
    new_obj->barycenter = current_object_->barycenter + predicted_shift;
    new_obj->ground_center = current_object_->ground_center + predicted_shift;

    // B. update cloud & polygon
    PointICloudPtr cloud = new_obj->object_ptr->cloud;
    for (size_t j = 0u; j < cloud->points.size(); ++j) {
        cloud->points[j].x += predicted_shift[0];
        cloud->points[j].y += predicted_shift[1];
        cloud->points[j].z += predicted_shift[2];
    }
    PolygonDType &polygon = new_obj->object_ptr->polygon;
    for (size_t j = 0u; j < polygon.points.size(); ++j) {
        polygon.points[j].x += predicted_shift[0];
        polygon.points[j].y += predicted_shift[1];
        polygon.points[j].z += predicted_shift[2];
    }

    // C. update filter
    filter_->updateWithoutObservation(time_diff);

    // D. update states of track
    belief_anchor_point_ = new_obj->anchor_point;
    // NEED TO NOTICE: All the states would be collected mainly based on states
    // of tracked object. Thus, update tracked object when you update the state
    // of track !!!!
    new_obj->velocity = belief_velocity_;

    // E. update track info
    age_++;
    consecutive_invisible_count_++;
    period_ += time_diff;

    // F. update history
    int history_size = history_objects_.size();
    if (history_size >= s_tracker_cached_history_size_maximum_) {
        history_objects_.pop_front();
    }
    history_objects_.push_back(current_object_);

    current_object_ = new_obj;
}

/**
 * @brief update object tracker without observation but predict velocity in
 * "predict_state"
 * @param predict_state
 * @param time_diff
 */
void ObjectTracker::updateWithoutObservation(
    const Eigen::VectorXf &predict_state, const double &time_diff) {
    // A. update object of track
    TrackableObjectPtr new_obj(new TrackableObject());
    new_obj->clone(*current_object_);

    Eigen::Vector3f predicted_shift = predict_state.tail(3) * time_diff;

    new_obj->anchor_point = current_object_->anchor_point + predicted_shift;
    new_obj->barycenter = current_object_->barycenter + predicted_shift;
    new_obj->ground_center = current_object_->ground_center + predicted_shift;

    // B. update cloud & polygon
    PointICloudPtr cloud = new_obj->object_ptr->cloud;
    for (size_t j = 0u; j < cloud->points.size(); ++j) {
        cloud->points[j].x += predicted_shift[0];
        cloud->points[j].y += predicted_shift[1];
        cloud->points[j].z += predicted_shift[2];
    }
    PolygonDType &polygon = new_obj->object_ptr->polygon;
    for (size_t j = 0u; j < polygon.points.size(); ++j) {
        polygon.points[j].x += predicted_shift[0];
        polygon.points[j].y += predicted_shift[1];
        polygon.points[j].z += predicted_shift[2];
    }

    // C. update filter without object
    filter_->updateWithoutObservation(time_diff);

    // D. update states of track
    belief_anchor_point_ = new_obj->anchor_point;
    // NEED TO NOTICE: All the states would be collected mainly based on states
    // of tracked object. Thus, update tracked object when you update the state
    // of track !!!!
    new_obj->velocity = belief_velocity_;

    // E. update track info
    age_++;
    consecutive_invisible_count_++;
    period_ += time_diff;

    // F. update history
    int history_size = history_objects_.size();
    if (history_size >= s_tracker_cached_history_size_maximum_) {
        history_objects_.pop_front();
    }
    history_objects_.push_back(current_object_);

    current_object_ = new_obj;
}

/**
 * @brief smooth velocity as last velocity or static to Zero
 *  keep motion if accelaration of filter is greater than a threshold
 *  update velocity to Zero if is static hypothesis
 * @param time_diff
 */
void ObjectTracker::smoothTrackerVelocity(const double &time_diff) {
    // A. keep motion if accelaration of filter is greater than a threshold
    Eigen::Vector3f filter_acceleration_gain = Eigen::Vector3f::Zero();
    filter_->getAccelerationGain(&filter_acceleration_gain);
    double filter_accelaration = filter_acceleration_gain.norm();
    bool need_keep_motion = filter_accelaration > s_acceleration_noise_maximum_;
    if (need_keep_motion) {
        Eigen::Vector3f last_velocity = Eigen::Vector3f::Zero();
        if (history_objects_.size() > 0) {
            last_velocity =
                history_objects_[history_objects_.size() - 1]->velocity;
        }
        belief_velocity_ = last_velocity;
        belief_velocity_accelaration_ = Eigen::Vector3f::Zero();
        // NEED TO NOTICE: All the states would be collected mainly based on
        // states
        // of tracked object. Thus, update tracked object when you update the
        // state
        // of track !!!!
        current_object_->velocity = belief_velocity_;
        // keep static hypothesis
        return;
    }

    // B. static hypothesis check & claping noise
    is_static_hypothesis_ = checkTrackerStaticHypothesis(time_diff);
    if (is_static_hypothesis_) {
        belief_velocity_ = Eigen::Vector3f::Zero();
        belief_velocity_accelaration_ = Eigen::Vector3f::Zero();
        // NEED TO NOTICE: All the states would be collected mainly based on
        // states
        // of tracked object. Thus, update tracked object when you update the
        // state
        // of track !!!!
        current_object_->velocity = belief_velocity_;
    }
}

/**
 * @brief
 *  smooth tracker orientation as obvious velocity's direction or observation's
 * direction
 *  predict object size and ground center based on object cloud and previous
 * direction
 *  update object size and ground center to predicted ones
 */
void ObjectTracker::smoothTrackerOrientation() {
    Eigen::Vector3f current_dir = current_object_->direction;
    float current_speed = current_object_->velocity.head(2).norm();
    bool velocity_is_obvious = current_speed > (s_speed_noise_maximum_ * 2);
    if (velocity_is_obvious) {
        current_dir = current_object_->velocity;
    } else {
        // TODO(gary): lane_direction needs HD Map
        // current_dir = current_object_->lane_direction;
        current_dir = current_object_->direction;
    }
    current_dir(2) = 0;
    current_dir.normalize();
    Eigen::Vector3d new_size;
    Eigen::Vector3d new_center;
    common::bbox::computeBboxSizeCenter<PointICloudPtr>(
        current_object_->object_ptr->cloud, current_dir.cast<double>(),
        &new_size, &new_center);
    current_object_->direction = current_dir;
    current_object_->ground_center = new_center.cast<float>();
    current_object_->size = new_size.cast<float>();
}

/**
 * @breif cheack whether tracking object is static
 * @note by checking previous/current consecutive velocity's angle change is
 * greater than 45 degree
 * @param time_diff
 * @return is static or not
 */
bool ObjectTracker::checkTrackerStaticHypothesis(const double &time_diff) {
    // A. check whether tracking object's velocity angle changed obviously
    bool is_velocity_angle_change =
        checkTrackerStaticHypothesisByVelocityAngleChange(time_diff);

    // B. evaluate velocity level
    double speed = belief_velocity_.head(2).norm();
    bool velocity_is_noise = speed < (s_speed_noise_maximum_ / 2);
    bool velocity_is_small = speed < (s_speed_noise_maximum_ / 1);
    if (velocity_is_noise) {
        return true;
    }
    // NEED TO NOTICE: claping small velocity may not reasonable when the true
    // velocity of target object is really small. e.g. a moving out vehicle in
    // a parking lot. Thus, instead of clapping all the small velocity, we clap
    // those whose history trajectory or performance is close to a static one.
    if (velocity_is_small && is_velocity_angle_change) {
        return true;
    }
    return false;
}

/**
 * @brief check whether tracking object's velocity angle changed obviously
 * @note by checking previous/current consecutive velocity's angle change is
 * greater than 45 degree
 * @param time_diff
 * @return
 */
bool ObjectTracker::checkTrackerStaticHypothesisByVelocityAngleChange(
    const double &time_diff) {
    Eigen::Vector3f previous_velocity =
        history_objects_[history_objects_.size() - 1]->velocity;
    Eigen::Vector3f current_velocity = current_object_->velocity;
    double velocity_angle_change =
        common::geometry::computeTheta2dXyBetweenVectors<Eigen::Vector3f>(
            previous_velocity, current_velocity);
    if (fabs(velocity_angle_change) > M_PI / 4.0) {
        return true;
    }
    return false;
}
//--------------------------- predict & update -------------------------------//
}  // namespace tracking
}  // namespace autosense
