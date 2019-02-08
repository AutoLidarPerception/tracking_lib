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
#include "tracking/trackers/multi_object_tracker.hpp"
#include <ros/ros.h>

#include "common/time.hpp"  // common::Clock

namespace autosense {
namespace tracking {

int MultiObjectTracker::s_tracker_idx_ = 0;
int MultiObjectTracker::s_tracker_consecutive_invisible_maximum_ = 1;
float MultiObjectTracker::s_tracker_visible_ratio_minimum_ = 0.6;

/// reserve 1000 trackers space
MultiObjectTracker::MultiObjectTracker() { trackers_.reserve(1000); }

/// call clear()
MultiObjectTracker::~MultiObjectTracker() { clear(); }

void MultiObjectTracker::clear() {
    for (size_t i = 0u; i < trackers_.size(); ++i) {
        if (trackers_[i]) {
            // delete (trackers_[i]);
            trackers_[i] = nullptr;
        }
    }
    trackers_.clear();
}

//------------------------------------ init ----------------------------------//
bool MultiObjectTracker::setTrackerConsecutiveInvisibleMaximum(
    const int &track_consecutive_invisible_maximum) {
    if (track_consecutive_invisible_maximum >= 0) {
        s_tracker_consecutive_invisible_maximum_ =
            track_consecutive_invisible_maximum;
        ROS_INFO_STREAM("consecutive invisible maximum of object tracker is "
                        << s_tracker_consecutive_invisible_maximum_);
        return true;
    }
    ROS_ERROR("invalid consecutive invisible maximum of object tracker!");
    return false;
}

bool MultiObjectTracker::setTrackerVisibleRatioMinimum(
    const float &track_visible_ratio_minimum) {
    if (track_visible_ratio_minimum >= 0 && track_visible_ratio_minimum <= 1) {
        s_tracker_visible_ratio_minimum_ = track_visible_ratio_minimum;
        ROS_INFO_STREAM("visible ratio minimum of object tracker is "
                        << s_tracker_visible_ratio_minimum_);
        return true;
    }
    ROS_ERROR("invalid visible ratio minimum of object track!");
    return false;
}
//------------------------------------ init ----------------------------------//

//---------------------------- predict & update ------------------------------//
/**
 * @brief state predict of maintained trackers
 *  基于上一帧预测当前帧追踪障碍物状态
 * @params[OUT] trackers_predict: predicted states of maintained trackers
 * @params[IN] time_diff: time interval for predicting
 * @return nothing
 */
void MultiObjectTracker::execPredict(
    std::vector<Eigen::VectorXf> *tracker_predicts, const double &time_diff) {
    tracker_predicts->resize(trackers_.size());
    for (size_t i = 0u; i < trackers_.size(); ++i) {
        // 每一个物体追踪器->predict
        (*tracker_predicts)[i] = trackers_[i]->execPredict(time_diff);
    }
}

/**
 * @brief Update assigned trackers with associated observed objects
 * @param trackers_predict
 * @param new_objects
 * @param assignments
 * @param time_diff
 */
void MultiObjectTracker::updateAssignedTrackers(
    std::vector<TrackableObjectPtr> *obsvs_trackable,
    const std::vector<TrackerObsvPair> &assignments,
    const double &time_diff) {
    for (size_t i = 0u; i < assignments.size(); ++i) {
        int tracker_id = assignments[i].first;
        int obsv_id = assignments[i].second;
        trackers_[tracker_id]->updateWithObservation(
            (*obsvs_trackable)[obsv_id], time_diff);
    }
}

/**
 * @brief Update system's trackers without matched observed objects
 * @param trackers_predict
 * @param unassigned_trackers
 * @param time_diff
 * @result
 *  Maintain current unassigned trackers "unassigned_trackers_"
 */
void MultiObjectTracker::updateUnassignedTrackers(
    const std::vector<Eigen::VectorXf> &tracker_predicts,
    const std::vector<int> &trackers_unassigned,
    const double &time_diff) {
    unassigned_trackers_.clear();
    for (size_t i = 0u; i < trackers_unassigned.size(); ++i) {
        int tracker_idx = trackers_unassigned[i];
        trackers_[tracker_idx]->updateWithoutObservation(
            tracker_predicts[tracker_idx], time_diff);
        // update unassigned trackers at current frame
        unassigned_trackers_.push_back(trackers_[tracker_idx]->idx_);
    }
}

/**
 * @brief Create new trackers for observed objects without matched trackers
 * @note Trackers should satisfy some condition to be collected as system
 * @brief 为未关联的新可追踪物体创建绑定的追踪器
 *  将unassigned_objects下标指示的可追踪物体添加进系统多物体追踪器中
 * @param new_objects 经过constructTrackedObjects()构造的当前帧所有可追踪障碍物
 * @param unassigned_objects 需要绑定追踪器的可追踪物体下标集合
 */
void MultiObjectTracker::createNewTrackers(
    const std::vector<TrackableObjectPtr> &obsvs_trackable,
    const std::vector<int> &unassigned_ids) {
    common::Clock clock;
    for (size_t i = 0u; i < unassigned_ids.size(); ++i) {
        int obsv_id = unassigned_ids[i];
        // 为每一个追踪障碍物绑定障碍物追踪器
        ObjectTrackerPtr tracker(
            new ObjectTracker(obsvs_trackable[obsv_id], getNextTrackerId()));
        trackers_.push_back(tracker);
    }
    ROS_INFO_STREAM("HmTrackingWorker::created "
                    << unassigned_ids.size() << " new Trackers. Took "
                    << clock.takeRealTime() << "ms.");
}

/**
 * @brief init trajectory with all current observed segments
 * @param obsvs_trackable
 * @param unassigned_ids
 * @param trajectory_segments
 * @param trajectory_periods
 */
void MultiObjectTracker::createNewTrackers(
    const std::vector<TrackableObjectPtr> &obsvs_trackable,
    const std::vector<int> &unassigned_ids,
    std::map<IdType, std::vector<IdType>> *trajectory_segments,
    std::map<IdType, double> *trajectory_periods) {
    common::Clock clock;
    for (size_t i = 0u; i < unassigned_ids.size(); ++i) {
        int obsv_id = unassigned_ids[i];
        // 为每一个追踪障碍物绑定障碍物追踪器
        ObjectTrackerPtr tracker(
            new ObjectTracker(obsvs_trackable[obsv_id], getNextTrackerId()));
        trackers_.push_back(tracker);
        // initial unassigned obsv
        std::vector<IdType> obsvs;
        obsvs.push_back(obsvs_trackable[obsv_id]->object_ptr->id);
        (*trajectory_segments).insert(std::make_pair(tracker->idx_, obsvs));
        // initial period -1.0
        (*trajectory_periods).insert(std::make_pair(tracker->idx_, -1.));
    }
    ROS_INFO_STREAM("HmTrackingWorker::created "
                    << unassigned_ids.size() << " new Trackers. Took "
                    << clock.takeRealTime() << "ms.");
}

/**
 * @brief remove lost trackers based on invisible ratio
 *          and consecutive invisible count.
 * @return current number of valid tracker
 * @result
 *  remain valid trackers in "trackers_"
 *  collect lost trackers in "lost_trackers_"
 */
int MultiObjectTracker::removeLostTrackers() {
    size_t valid_tracker_num = 0;
    lost_trackers_.clear();
    for (size_t i = 0u; i < trackers_.size(); ++i) {
        // A. remove trackers invisible ratio less than given minimum
        float tracker_visible_ratio =
            trackers_[i]->total_visible_count_ * 1.0f / trackers_[i]->age_;
        if (tracker_visible_ratio < s_tracker_visible_ratio_minimum_) {
            lost_trackers_.push_back(trackers_[i]->idx_);
            continue;
        }

        // B. remove trackers consecutive invisible count > given maximum
        int tracker_consecutive_invisible_count =
            trackers_[i]->consecutive_invisible_count_;
        if (tracker_consecutive_invisible_count >
            s_tracker_consecutive_invisible_maximum_) {
            lost_trackers_.push_back(trackers_[i]->idx_);
            continue;
        }

        // C. keep trackers pass above limitation
        if (i == valid_tracker_num) {
            // no need to move current valid tracker #i
            valid_tracker_num++;
        } else {
            // move current valid tracker #i forward, lost tracker
            // #valid_tracker_num
            // backward
            ObjectTrackerPtr tmp = trackers_[i];
            trackers_[i] = trackers_[valid_tracker_num];
            trackers_[valid_tracker_num] = tmp;
            valid_tracker_num++;
        }
    }

    // D. remove lost tracks all one time
    int num_removed = trackers_.size() - valid_tracker_num;
    for (size_t i = valid_tracker_num; i < trackers_.size(); ++i) {
        if (trackers_[i] != nullptr) {
            // delete (trackers_[i]);
            trackers_[i] = nullptr;
        }
    }
    trackers_.resize(valid_tracker_num);

    return num_removed;
}

int MultiObjectTracker::removeLostTrackers(
    std::map<IdType, Trajectory> *trajectory_poses,
    std::map<IdType, double> *trajectory_periods) {
    size_t valid_tracker_num = 0;
    lost_trackers_.clear();
    for (size_t i = 0u; i < trackers_.size(); ++i) {
        // A. remove trackers invisible ratio less than given minimum
        float tracker_visible_ratio =
            trackers_[i]->total_visible_count_ * 1.0f / trackers_[i]->age_;
        if (tracker_visible_ratio < s_tracker_visible_ratio_minimum_) {
            lost_trackers_.push_back(trackers_[i]->idx_);

            auto iter = (*trajectory_poses).find(trackers_[i]->idx_);
            if (iter != (*trajectory_poses).end()) {
                // TODO(gary): maintain already fixed trajectories for tracking
                auto iter_periods =
                    (*trajectory_periods).find(trackers_[i]->idx_);
                iter_periods->second = trackers_[i]->period_;

                (*trajectory_poses).erase(iter);
            }

            continue;
        }

        // B. remove trackers consecutive invisible count > given maximum
        int tracker_consecutive_invisible_count =
            trackers_[i]->consecutive_invisible_count_;
        if (tracker_consecutive_invisible_count >
            s_tracker_consecutive_invisible_maximum_) {
            lost_trackers_.push_back(trackers_[i]->idx_);

            auto iter = (*trajectory_poses).find(trackers_[i]->idx_);
            if (iter != (*trajectory_poses).end()) {
                // TODO(gary): maintain already fixed trajectories for tracking
                auto iter_periods =
                    (*trajectory_periods).find(trackers_[i]->idx_);
                iter_periods->second = trackers_[i]->period_;

                (*trajectory_poses).erase(iter);
            }

            continue;
        }

        // C. keep trackers pass above limitation
        if (i == valid_tracker_num) {
            // no need to move current valid tracker #i
            valid_tracker_num++;
        } else {
            // move current valid tracker #i forward, lost tracker
            // #valid_tracker_num
            // backward
            ObjectTrackerPtr tmp = trackers_[i];
            trackers_[i] = trackers_[valid_tracker_num];
            trackers_[valid_tracker_num] = tmp;
            valid_tracker_num++;
        }
    }

    // D. remove lost tracks all one time
    int num_removed = trackers_.size() - valid_tracker_num;
    for (size_t i = valid_tracker_num; i < trackers_.size(); ++i) {
        if (trackers_[i] != nullptr) {
            // delete (trackers_[i]);
            trackers_[i] = nullptr;
        }
    }
    trackers_.resize(valid_tracker_num);

    return num_removed;
}
//---------------------------- predict & update ------------------------------//
}  // namespace tracking
}  // namespace autosense
