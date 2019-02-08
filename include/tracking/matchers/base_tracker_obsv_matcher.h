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
#ifndef TRACKING_INCLUDE_TRACKING_MATCHERS_BASE_TRACKER_OBSV_MATCHER_H_
#define TRACKING_INCLUDE_TRACKING_MATCHERS_BASE_TRACKER_OBSV_MATCHER_H_

#include <Eigen/Core>
#include <string>
#include <utility>
#include <vector>

#include "common/types//trackable_object.hpp"
#include "tracking/trackers/object_tracker.hpp"

namespace autosense {
namespace tracking {

typedef enum {
    HUNGARIAN_MATCHER = 0,
} MatcherType;

typedef std::pair<int, int> TrackerObsvPair;

class BaseTrackerObsvMatcher {
 public:
    BaseTrackerObsvMatcher() {}

    virtual ~BaseTrackerObsvMatcher() {}

    // @brief match detected objects to maintained tracks
    //   当前帧检测到的追踪障碍物能否与系统追踪器列表中的某个追踪器关联
    // @params[IN] objects: detected objects
    // @params[IN] tracks: maintained tracks
    // @params[IN] tracks_predict: predicted states of maintained tracks
    //                             追踪器预测的追踪障碍物新状态
    // @params[OUT] assignments: matched pair of <track, object>
    // @params[OUT] unassigned_tracks: unmatched tracks 未找到关联障碍物的追踪器
    // @params[OUT] unassigned_objects: unmatched objects 找不到
    // @return nothing
    virtual void match(std::vector<TrackableObjectPtr> *objects_obsved,
                       const std::vector<ObjectTrackerPtr> &trackers,
                       const std::vector<Eigen::VectorXf> &trackers_predict,
                       std::vector<TrackerObsvPair> *assignments,
                       std::vector<int> *unassigned_trackers,
                       std::vector<int> *unassigned_objects_obsved) = 0;

    // @brief get name of matcher
    // @return name of matcher
    virtual std::string Name() const = 0;
};  // class BaseTrackerObsvMatcher

}  // namespace tracking
}  // namespace autosense

#endif  // TRACKING_INCLUDE_TRACKING_MATCHERS_BASE_TRACKER_OBSV_MATCHER_H_
