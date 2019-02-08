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
#ifndef TRACKING_INCLUDE_TRACKING_TRACKERS_OBJECT_TRACKER_HPP_
#define TRACKING_INCLUDE_TRACKING_TRACKERS_OBJECT_TRACKER_HPP_

#include <Eigen/Core>
#include <deque>  // std::deque
#include <memory>
#include <string>

#include "tracking/filters/base_filter.hpp"

#include "common/types/object.hpp"
#include "common/types/trackable_object.hpp"

namespace autosense {
namespace tracking {
//单物体追踪器
class ObjectTracker {
 public:
    explicit ObjectTracker(TrackableObjectPtr obj, IdType tracker_id);

    ~ObjectTracker();

    // @brief set filter method for all the object track objects
    // @params[IN] filter_method: method name of filtering algorithm
    // @return true if set successfully, otherwise return false
    static bool setFilterMethod(const std::string &filter_method_name);

    // @brief set track cached history size maximum
    // @params[IN] track_cached_history_size_maximum: track cached history size
    // maximum
    // @return true if set successfully, otherwise return false
    static bool setTrackerCachedHistorySizeMaximum(
        const int &tracker_cached_history_size_maximum);

    // @brief set acceleration noise maximum
    // @params[IN] acceleration_noise_maximum: acceleration noise maximum
    // @return true if set successfully, otherwise return false
    static bool setAccelerationNoiseMaximum(
        const double &acceleration_noise_maximum);

    // @brief set speed noise maximum
    // @params[IN] speed noise maximum: speed noise maximum
    // @return true if set successfully, otherwise return false
    static bool setSpeedNoiseMaximum(const double &speed_noise_maximum);

    //--------------------------- predict & update ---------------------------//
    // @brief predict the state of track
    // @params[IN] time_diff: time interval for predicting
    // @return predicted states of track
    Eigen::VectorXf execPredict(const double &time_diff);

    Eigen::VectorXf getExpectedState(const double &time_diff);

    void getExpectedObject(const double &time_diff,
                           TrackableObjectPtr expected_obj) const {
        if (expected_obj == nullptr) {
            expected_obj.reset(new TrackableObject);
        }
        expected_obj->clone(*current_object_);

        // based on CVM
        Eigen::Vector3f predicted_shift = belief_velocity_ * time_diff;

        // A. update tracking object
        expected_obj->anchor_point =
            current_object_->anchor_point + predicted_shift;
        expected_obj->barycenter =
            current_object_->barycenter + predicted_shift;
        expected_obj->ground_center =
            current_object_->ground_center + predicted_shift;

        expected_obj->velocity = belief_velocity_;
        expected_obj->direction = current_object_->direction;

        // B. update cloud & polygon
        PointICloudPtr cloud = expected_obj->object_ptr->cloud;
        for (size_t j = 0u; j < cloud->points.size(); ++j) {
            cloud->points[j].x += predicted_shift[0];
            cloud->points[j].y += predicted_shift[1];
            cloud->points[j].z += predicted_shift[2];
        }
        PolygonDType &polygon = expected_obj->object_ptr->polygon;
        for (size_t j = 0u; j < polygon.points.size(); ++j) {
            polygon.points[j].x += predicted_shift[0];
            polygon.points[j].y += predicted_shift[1];
            polygon.points[j].z += predicted_shift[2];
        }
    }

    // @brief update tracker with object
    // @params[IN] new_object: recent detected object for current updating
    // @params[IN] time_diff: time interval from last updating
    // @return nothing
    void updateWithObservation(TrackableObjectPtr object_obsved,
                               const double &time_diff);

    // @brief update track without object
    // @params[IN] time_diff: time interval from last updating
    // @return nothing
    void updateWithoutObservation(const double &time_diff);

    // @brief update track without object with given predicted state
    // @params[IN] predict_state: given predicted state of track
    // @params[IN] time_diff: time interval from last updating
    // @return nothing
    void updateWithoutObservation(const Eigen::VectorXf &predict_state,
                                  const double &time_diff);
    //--------------------------- predict & update ---------------------------//

 protected:
    //------------------- smooth velocity and orientation --------------------//
    // @brief smooth velocity over track history
    // @params[IN] new_object: new detected object for updating
    // @params[IN] time_diff: time interval from last updating
    // @return nothing
    void smoothTrackerVelocity(const double &time_diff);

    // @brief smooth orientation over track history
    // @return nothing
    void smoothTrackerOrientation();

    // @brief check whether track is static or not
    // @params[IN] new_object: new detected object just updated
    // @params[IN] time_diff: time interval between last two updating
    // @return true if track is static, otherwise return false
    bool checkTrackerStaticHypothesis(const double &time_diff);

    // @brief sub strategy of checking whether track is static or not via
    // considering the velocity angle change
    // @params[IN] new_object: new detected object just updated
    // @params[IN] time_diff: time interval between last two updating
    // @return true if track is static, otherwise return false
    bool checkTrackerStaticHypothesisByVelocityAngleChange(
        const double &time_diff);
    //------------------- smooth velocity and orientation --------------------//

 private:
    ObjectTracker();

 public:
    // algorithm setup 滤波器算法
    static FilterType s_filter_method_;
    BaseFilter *filter_;

    // basic info
    // 追踪器唯一标识id
    int idx_;
    // 创建至今存活帧数
    int age_;
    // 追踪物体累计观察帧数
    int total_visible_count_;
    // 追踪物体连续丢失帧数
    int consecutive_invisible_count_;
    // 持续追踪时间间隔(s)
    double period_;
    // 追踪器上一帧观察实例
    TrackableObjectPtr current_object_;

    // tracked objects in history
    std::deque<TrackableObjectPtr> history_objects_;

    // states
    // NEED TO NOTICE: All the states would be collected mainly based on states
    // of tracked object. Thus, update tracked object when you update the state
    // of track !!!!!
    bool is_static_hypothesis_;

    // 追踪器追踪的物体可靠状态(锚点+速度+加速度) ==>
    // 根据追踪器绑定的追踪物体TrackedObject状态更新
    Eigen::Vector3f belief_anchor_point_;
    Eigen::Vector3f belief_velocity_;
    Eigen::Vector3f belief_velocity_accelaration_;

 private:
    // global setup
    static int s_tracker_cached_history_size_maximum_;
    static double s_speed_noise_maximum_;
    static double s_acceleration_noise_maximum_;
};  // class ObjectTracker

typedef std::shared_ptr<ObjectTracker> ObjectTrackerPtr;
typedef std::shared_ptr<const ObjectTracker> ObjectTrackerConstPtr;

}  // namespace tracking
}  // namespace autosense

#endif  // TRACKING_INCLUDE_TRACKING_TRACKERS_OBJECT_TRACKER_HPP_
