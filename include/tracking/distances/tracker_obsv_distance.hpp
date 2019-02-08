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
#ifndef TRACKING_INCLUDE_TRACKING_DISTANCES_TRACKER_OBSV_DISTANCE_HPP_
#define TRACKING_INCLUDE_TRACKING_DISTANCES_TRACKER_OBSV_DISTANCE_HPP_

#include <Eigen/Core>
#include <string>

#include "common/types/trackable_object.hpp"
#include "tracking/trackers/object_tracker.hpp"

namespace autosense {
namespace tracking {
/**
 * @brief 计算关联距离矩阵中每个元素的距离度量
 * @note Tracker->Obsvs Associate Distance Matrix
 *  | Tracker(i)        -->Obsv(j) -->Obsv(j+1) -->Obsv(j+...)  |
 *  | Tracker(i+1)          ...                                 |
 *  | Tracker(i+...)        ...                                 |
 */
class TrackerObsvDistance {
 protected:
    // distance weights
    static double s_location_distance_weight_;
    static double s_direction_distance_weight_;
    static double s_bbox_size_distance_weight_;
    static double s_point_num_distance_weight_;
    static double s_histogram_distance_weight_;

 public:
    //-------------------------------- set weights
    //--------------------------------//
    // @brief set weight of location dist for all the track object distance
    // objects
    // @params[IN] location_distance_weight: weight of location dist
    // @return true if set successfully, otherwise return false
    static bool setLocationDistanceWeight(
        const float& location_distance_weight);

    // @brief set weight of direction dist for all the track object distance
    // objects
    // @params[IN] direction_distance_weight: weight of direction dist
    // @return true if set successfully, otherwise return false
    static bool setDirectionDistanceWeight(
        const float& direction_distance_weight);

    // @brief set weight of bbox size dist for all the track object distance
    // objects
    // @params[IN] bbox_size_distance_weight: weight of bbox size dist
    // @return true if set successfully, otherwise return false
    static bool setBboxSizeDistanceWeight(
        const float& bbox_size_distance_weight);

    // @brief set weight of point num dist for all the track object distance
    // objects
    // @params[IN] point_num_distance_weight: weight of point num dist
    // @return true if set successfully, otherwise return false
    static bool setPointNumDistanceWeight(
        const float& point_num_distance_weight);

    // @brief set weight of histogram dist for all the track object distance
    // objects
    // @params[IN] weight_histogram_dist: weight of histogram dist
    // @return true if set successfully, otherwise return false
    static bool setHistogramDistanceWeight(
        const float& histogram_distance_weight);
    //-------------------------------- set weights
    //--------------------------------//

    // @brief compute distance for given track & object
    // @params[IN] track: track for <track, object> distance computing
    // @params[IN] track_predict: predicted state of given track
    // @params[IN] new_object: recently detected object
    // @return computed <track, object> distance
    static float computeDistance(ObjectTrackerConstPtr tracker,
                                 const Eigen::VectorXf& tracker_predict,
                                 TrackableObjectConstPtr object_obsved);

    static std::string Name() { return "TrackerObsvDistance"; }

 private:
    //------------------- compute different kinds of distance
    //--------------------//
    // @brief compute location distance for given track & object
    // @params[IN] track: track for <track, object> distance computing
    // @params[IN] track_predict: predicted state of given track
    // @params[IN] new_object: recently detected object
    // @return location distacne of given <track, object>
    static float computeLocationDistance(
        const ObjectTrackerConstPtr tracker,
        const Eigen::VectorXf& tracker_predict,
        const TrackableObjectConstPtr object_obsved);

    // @brief compute direction distance for given track & object
    // @params[IN] track: track for <track, object> distance computing
    // @params[IN] track_predict: predicted state of given track
    // @params[IN] new_object: recently detected object
    // @return direction distance of given <track, object>
    static float computeDirectionDistance(
        const ObjectTrackerConstPtr tracker,
        const Eigen::VectorXf& tracker_predict,
        const TrackableObjectConstPtr object_obsved);

    // @brief compute bbox size distance for given track & object
    // @params[IN] track: track for <track, object> distance computing
    // @params[IN] new_object: recently detected object
    // @return bbox size distance of given <track, object>
    static float computeBboxSizeDistance(
        const ObjectTrackerConstPtr tracker,
        const TrackableObjectConstPtr object_obsved);

    // @brief compute point num distance for given track & object
    // @params[IN] track: track for <track, object> distance computing
    // @params[IN] new_object: recently detected object
    // @return point num distance of given <track, object>
    static float computePointNumDistance(
        const ObjectTrackerConstPtr tracker,
        const TrackableObjectConstPtr object_obsved);

    // @brief compute histogram distance for given track & object
    // @params[IN] track: track for <track, object> distance computing
    // @params[IN] new_object: recently detected object
    // @return histogram distance of given <track, object>
    static float computeHistogramDistance(
        const ObjectTrackerConstPtr tracker,
        const TrackableObjectConstPtr object_obsved);
};  // class TrackerObsvDistance

}  // namespace tracking
}  // namespace autosense

#endif  // TRACKING_INCLUDE_TRACKING_DISTANCES_TRACKER_OBSV_DISTANCE_HPP_
