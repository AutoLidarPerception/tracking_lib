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
#ifndef TRACKING_INCLUDE_TRACKING_MATCHERS_HUNGARIAN_TRACKER_OBSV_MATCHER_HPP_
#define TRACKING_INCLUDE_TRACKING_MATCHERS_HUNGARIAN_TRACKER_OBSV_MATCHER_HPP_

#include <Eigen/Core>
#include <string>
#include <vector>

#include "common/types/trackable_object.hpp"
#include "tracking/matchers/base_tracker_obsv_matcher.h"
#include "tracking/trackers/object_tracker.hpp"

namespace autosense {
namespace tracking {

class HungarianTrackerObsvMatcher : public BaseTrackerObsvMatcher {
 private:
    // threshold of matching
    static float s_match_distance_maximum_;

 public:
    HungarianTrackerObsvMatcher() {}

    ~HungarianTrackerObsvMatcher() {}

    // @brief set match distance maximum for matcher
    // @params[IN] match_distance_maximum: match distance maximum
    // @return true if set successfuly, otherwise return false
    static bool setMatchDistanceMaximum(const float &match_distance_maximum);

    // 当前帧检测到的追踪障碍物能否与系统追踪器列表中的某个追踪器关联
    // @brief match detected objects to tracks
    // @params[IN] objects: new detected objects for matching
    // @params[IN] tracks: maintaining tracks for matching
    // @params[IN] tracks_predict: predicted state of maintained tracks
    // @params[OUT] assignments: assignment pair of object & track
    // @params[OUT] unassigned_tracks: tracks without matched object
    // @params[OUT] unassigned_objects: objects without matched track
    // @return nothing
    void match(std::vector<TrackableObjectPtr> *objects_obsved,
               const std::vector<ObjectTrackerPtr> &trackers,
               const std::vector<Eigen::VectorXf> &trackers_predict,
               std::vector<TrackerObsvPair> *assignments,
               std::vector<int> *unassigned_trackers,
               std::vector<int> *unassigned_objects_obsved);

    std::string Name() const { return "HungarianTrackerObsvMatcher"; }

 protected:
    // @brief compute association matrix
    // @params[IN] tracks: maintained tracks for matching
    // @params[IN] tracks_predict: predicted states of maintained tracks
    // @params[IN] new_objects: recently detected objects
    // @params[OUT] association_mat: matrix of association distance
    // @return nothing
    void computeAssociateMatrix(
        const std::vector<ObjectTrackerPtr> &trackers,
        const std::vector<Eigen::VectorXf> &trackers_predict,
        const std::vector<TrackableObjectPtr> &objects_obsved,
        Eigen::MatrixXf *association_mat);

    // @brief compute connected components within given threshold
    // @params[IN] association_mat: matrix of association distance
    // @params[IN] connected_threshold: threshold of connected components
    // @params[OUT] track_components: connected objects of given tracks
    // @params[OUT] obj_components: connected tracks of given objects
    // @return nothing
    void computeConnectedComponents(
        const Eigen::MatrixXf &association_mat,
        const float &connected_threshold,
        std::vector<std::vector<int>> *tracker_components,
        std::vector<std::vector<int>> *objects_obsved_components);

    // @brief match detected objects to tracks in component level
    // @params[IN] association_mat: association matrix of all objects to tracks
    // @params[IN] track_component: component of track
    // @params[IN] object_component: component of object
    // @params[OUT] sub_assignments: component assignment pair of object & track
    // @params[OUT] sub_unassigned_tracks: component tracks not matched
    // @params[OUT] sub_unasgined_objects: component objects not matched
    // @return nothing
    void matchInComponent(const Eigen::MatrixXf &association_mat,
                          const std::vector<int> &tracker_component,
                          const std::vector<int> &objects_obsved_component,
                          std::vector<TrackerObsvPair> *sub_assignments,
                          std::vector<int> *sub_unassigned_trackers,
                          std::vector<int> *sub_unassigned_objects_obsved);

    // @brief assign objects to tracks using components
    // @params[IN] association_mat: matrix of association distance
    // @params[IN] assign_distance_maximum: threshold distance of assignment
    // @params[OUT] assignments: assignment pair of matched object & track
    // @params[OUT] unassigned_tracks: tracks without matched object
    // @params[OUT] unassigned_objects: objects without matched track
    // @return nothing
    void assignObsvedObjects2Trackers(
        const Eigen::MatrixXf &association_mat,
        const double &assign_distance_maximum,
        std::vector<TrackerObsvPair> *assignments,
        std::vector<int> *unassigned_trackers,
        std::vector<int> *unassigned_objects_obsved);
};  // class HungarianTrackerObsvMatcher

}  // namespace tracking
}  // namespace autosense

#endif  // TRACKING_INCLUDE_TRACKING_MATCHERS_HUNGARIAN_TRACKER_OBSV_MATCHER_HPP_
