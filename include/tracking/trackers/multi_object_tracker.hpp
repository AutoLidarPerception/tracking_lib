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
#ifndef TRACKING_INCLUDE_TRACKING_TRACKERS_MULTI_OBJECT_TRACKER_HPP_
#define TRACKING_INCLUDE_TRACKING_TRACKERS_MULTI_OBJECT_TRACKER_HPP_

#include <map>
#include <vector>

#include "object_tracker.hpp"
#include "tracking/matchers/base_tracker_obsv_matcher.h"  // TrackerObsvPair

namespace autosense {
namespace tracking {
//多物体追踪器
class MultiObjectTracker {
 public:
    // 维护追踪器索引
    static int s_tracker_idx_;
    static int s_tracker_consecutive_invisible_maximum_;
    static float s_tracker_visible_ratio_minimum_;

 public:
    MultiObjectTracker();

    ~MultiObjectTracker();

    // @brief clear maintained tracks
    // @return nothing
    void clear();

    // @brief get next avaiable track id
    // 每个追踪器分配一个唯一的id
    // @return next avaiable track id
    static IdType getNextTrackerId() {
        IdType ret_tracker_id = s_tracker_idx_;
        if (s_tracker_idx_ == ID_MAX) {
            s_tracker_idx_ = 0;
        } else {
            s_tracker_idx_++;
        }
        return ret_tracker_id;
    }

    //-------------------------------- init ----------------------------------//
    // @brief set track consecutive invisible maximum
    // @params[IN] track_consecutive_invisible_maximum: track consecutive
    // invisible maximum
    // @return true if set successfully, otherwise return false
    static bool setTrackerConsecutiveInvisibleMaximum(
        const int &tracker_consecutive_invisible_maximum);

    // @brief set track visible ratio minimum
    // @params[IN] track_visible_ratio_minimum: track visible ratio minimum
    // @return true if set successfully, otherwise return false
    static bool setTrackerVisibleRatioMinimum(
        const float &track_visible_ratio_minimum);
    //-------------------------------- init ----------------------------------//

    //------------------------- predict & update -----------------------------//
    void execPredict(std::vector<Eigen::VectorXf> *tracker_predicts,
                     const double &time_diff);

    // @brief update assigned tracks
    // @params[IN] tracks_predict: predicted states of maintained tracks
    // @params[IN] new_objects: recently detected objects
    // @params[IN] assignments: assignment pair of <track, object>
    // @params[IN] time_diff: time interval for updating
    // @return nothing
    void updateAssignedTrackers(
        std::vector<TrackableObjectPtr> *obsvs_trackable,
        const std::vector<TrackerObsvPair> &assignments,
        const double &time_diff);

    // @brief update tracks without matched objects
    // @params[IN] tracks_predict: predicted states of maintained tracks
    // @params[IN] unassigned_tracks: index of unassigned tracks
    // @params[IN] time_diff: time interval for updating
    // @return nothing
    void updateUnassignedTrackers(
        const std::vector<Eigen::VectorXf> &tracker_predicts,
        const std::vector<int> &trackers_unassigned,
        const double &time_diff);

    // @brief create new tracks for objects without matched track
    // @params[IN] obsvs_trackable: recently detected objects after construction
    // @params[IN] unassigned_objects: index of unassigned objects
    // @return nothing
    void createNewTrackers(
        const std::vector<TrackableObjectPtr> &obsvs_trackable,
        const std::vector<int> &unassigned_ids);

    void createNewTrackers(
        const std::vector<TrackableObjectPtr> &obsvs_trackable,
        const std::vector<int> &unassigned_ids,
        std::map<IdType, std::vector<IdType>> *trajectory_segments,
        std::map<IdType, double> *trajectory_periods);

    // @brief remove lost tracks
    // @return number of removed tracks
    int removeLostTrackers();

    int removeLostTrackers(std::map<IdType, Trajectory> *trajectory_poses,
                           std::map<IdType, double> *trajectory_periods);

    //------------------------------ get status ------------------------------//
    // @brief get maintained tracks
    // @return maintained tracks
    inline std::vector<ObjectTrackerPtr> &getTrackers() { return trackers_; }

    // @brief get maintained tracks
    // @return maintained tracks
    inline const std::vector<ObjectTrackerPtr> &getTrackers() const {
        return trackers_;
    }

    inline const std::vector<IdType> &getUnassignedTrackers() const {
        return unassigned_trackers_;
    }

    inline const std::vector<IdType> &getLostTrackers() const {
        return lost_trackers_;
    }

    // @brief get size of maintained tracks
    // @return size of maintained tracks
    inline int size() const { return trackers_.size(); }
    //------------------------------ get status ------------------------------//

 private:
    std::vector<ObjectTrackerPtr> trackers_;

    std::vector<IdType> unassigned_trackers_;
    std::vector<IdType> lost_trackers_;
};  // class ObjectMultiTracker

}  // namespace tracking
}  // namespace autosense

#endif  // TRACKING_INCLUDE_TRACKING_TRACKERS_MULTI_OBJECT_TRACKER_HPP_
