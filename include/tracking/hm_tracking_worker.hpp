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

#ifndef TRACKING_INCLUDE_TRACKING_HM_TRACKING_WORKER_HPP_
#define TRACKING_INCLUDE_TRACKING_HM_TRACKING_WORKER_HPP_

#include <map>
#include <string>
#include <vector>

#include "common/common.hpp"     // common::EPSILON
#include "common/transform.hpp"  // common::transform::transformPointCloud

#include "feature_extractors/base_feature_extractor.hpp"
#include "tracking/base_tracking_worker.h"
#include "tracking/filters/base_filter.hpp"
#include "tracking/matchers/base_tracker_obsv_matcher.h"
#include "tracking/trackers/multi_object_tracker.hpp"

namespace autosense {
namespace tracking {

class HmTrackingWorker : public BaseTrackingWorker {
 public:
    explicit HmTrackingWorker(const TrackingWorkerParams &params);

    virtual ~HmTrackingWorker();

    // @brief set matcher method
    // @params[IN] matcher_method_name: name of mathcer method
    // @return true if set successfully, otherwise return fasle
    bool setMatcherMethod(const std::string &matcher_method_name);

    // @brief set matcher method
    // @params[IN] matcher_method_name: name of mathcer method
    // @return true if set successfully, otherwise return fasle
    bool setFilterMethod(const std::string &filter_method_name);

    // @brief track detected objects over consecutive frames
    // @params[IN] objects: recently detected objects
    // @params[IN] timestamp: timestamp of recently detected objects
    // @params[IN] options: tracker options with necessary information
    // @params[OUT] tracked_objects: tracked objects with tracking information
    // @return true if track successfully, otherwise return false
    bool track(const std::vector<ObjectPtr> &objects_obsved,
               double timestamp,
               const TrackingOptions &options,
               std::vector<ObjectPtr> *objects_tracked);

    /**
     * @brief get object trackers from current tracking system
     * @return object tracks maintained in tracker
     */
    virtual inline const std::vector<ObjectTrackerPtr> &getTrackers() const {
        return multi_object_tracker_->getTrackers();
    }

    virtual inline const std::vector<IdType> &getUnassignedTrackers() const {
        return multi_object_tracker_->getUnassignedTrackers();
    }

    virtual inline const std::vector<IdType> &getLostTrackers() const {
        return multi_object_tracker_->getLostTrackers();
    }

    virtual inline const std::map<IdType, Trajectory> &collectTrajectories()
        const {
        return trajectory_poses_;
    }

    virtual inline const std::vector<ObjectPtr> &collectTrackingObjectsInWorld()
        const {
        return tracking_objects_;
    }

    /*virtual std::vector<FixedTrajectory2> collectFixedTrajectories() {
        std::vector<FixedTrajectory2> fixedTrajectories;
        auto iter_period = trajectory_periods_.begin();
        for (; iter_period != trajectory_periods_.end();) {
            if (iter_period->second > 0.) {
                auto iter_ids = trajectory_segments_.find(iter_period->first);
                if (iter_ids != trajectory_segments_.end()) {
                    fixedTrajectories.push_back(
                            std::make_tuple(iter_period->second,
                                            iter_ids->second));
                    trajectory_segments_.erase(iter_ids);
                }
                iter_period = trajectory_periods_.erase(iter_period);
            } else {
                ++iter_period;
            }
        }

        return fixedTrajectories;
    };*/

    // TODO(gary): just collect fixed trajectory's tracker id and period
    virtual std::vector<FixedTrajectory> collectFixedTrajectories() {
        std::vector<FixedTrajectory> fixedTrajectories;
        auto iter = trajectory_periods_.begin();
        for (; iter != trajectory_periods_.end();) {
            if (iter->second > 0.) {
                fixedTrajectories.push_back(
                    std::make_tuple(iter->first, iter->second));
                iter = trajectory_periods_.erase(iter);
            } else {
                ++iter;
            }
        }

        return fixedTrajectories;
    }

    /**
     * @brief Get predicted objects for over-/under-segmentation classify with
     * current segments
     * @return
     */
    virtual std::vector<ObjectPtr> collectExpectedObjects(
        const double &timestamp, const Eigen::Matrix4d &trans_pose) const {
        std::vector<ObjectPtr> objects_expected;
        double time_diff = timestamp - time_stamp_;

        /// @note expected objects with size 0 before Tracking initialization
        if (booted_ && ((time_diff - 0.) > common::EPSILON)) {
            const std::vector<ObjectTrackerPtr> &trackers =
                multi_object_tracker_->getTrackers();
            objects_expected.resize(trackers.size());

            const Eigen::Matrix4d &pose_world2velo = trans_pose.inverse();

            // 对每个追踪器追踪物体进行操作
            for (size_t i = 0u; i < trackers.size(); ++i) {
                // 障碍物实体(用于显示/最终的输出等)
                ObjectPtr obj(new Object);
                // 从追踪器中提取障碍物预测
                TrackableObjectPtr expected_obj(new TrackableObject);
                trackers[i]->getExpectedObject(time_diff, expected_obj);
                obj->clone(*(expected_obj->object_ptr));

                // fill tracked information of object
                // size and direction
                obj->length = expected_obj->size[0];
                obj->width = expected_obj->size[1];
                obj->height = expected_obj->size[2];

                obj->direction = expected_obj->direction.cast<double>();
                Eigen::Vector3d &dir = obj->direction;
                dir = (pose_world2velo *
                       Eigen::Vector4d(dir[0], dir[1], dir[2], 0))
                          .head(3);

                Eigen::Vector3d coord_dir(1.0, 0.0, 0.0);
                obj->yaw_rad = common::geometry::computeTheta2dXyBetweenVectors<
                    Eigen::Vector3d>(coord_dir, obj->direction);

                // tracking information
                obj->tracker_id = trackers[i]->idx_;
                // 持续跟踪时间
                obj->tracking_time = trackers[i]->period_;
                obj->type = expected_obj->type;

                // position states restore original world coordinates
                // 局部坐标-->世界坐标
                // obj->ground_center = result_obj->ground_center.cast<double>()
                // -
                // global_to_local_offset_;
                obj->ground_center = expected_obj->ground_center.cast<double>();
                Eigen::Vector3d &center = obj->ground_center;
                center = (pose_world2velo *
                          Eigen::Vector4d(center[0], center[1], center[2], 1))
                             .head(3);

                // obj->anchor_point = result_obj->anchor_point.cast<double>() -
                // global_to_local_offset_;
                obj->anchor_point = expected_obj->anchor_point.cast<double>();

                // velocity
                obj->velocity = expected_obj->velocity.cast<double>();
                Eigen::Vector3d &vel = obj->velocity;
                vel = (pose_world2velo *
                       Eigen::Vector4d(vel[0], vel[1], vel[2], 0))
                          .head(3);

                obj->velocity_uncertainty = expected_obj->velocity_uncertainty;

                // restore original local Velodyne coordinates
                // 世界坐标-->局部坐标
                common::transform::transformPointCloud<PointI>(pose_world2velo,
                                                               obj->cloud);
                // TODO(gary): restore convex hull of the object
                /*
                common::transform::transformPointCloud<PointD>(pose_world2velo,
                                                               obj->polygon);*/

                objects_expected[i] = obj;
            }
        }

        return objects_expected;
    }

    std::string name() const { return "HmObjectTracker"; }

 protected:
    // @brief initialize tracker after obtaining detection of first frame
    // @params[IN] objects_obsved: recently detected objects
    // @params[IN] timestamp: timestamp of recently detected objects
    // @params[IN] options: tracker options with necessary information
    // @params[OUT] tracked_objects: tracked objects with tracking information
    // @return true if initialize successfully, otherwise return false
    bool bootloader(const std::vector<ObjectPtr> &objects_obsved,
                    const double &timestamp,
                    const TrackingOptions &options,
                    std::vector<ObjectPtr> *objects_tracked);

    // @brief construct tracked objects via necessray transformation & feature
    // computing
    // @params[IN] objects: objects for construction
    // @params[OUT] tracked_objects: constructed objects
    // @params[IN] pose: pose using for coordinate transformation
    // @params[IN] options: tracker options with necessary information
    // @return nothing
    void constructTrackedObjects(
        const std::vector<ObjectPtr> &objects_obsved,
        std::vector<TrackableObjectPtr> *objects_tracked,
        const Eigen::Matrix4d &pose,
        const TrackingOptions &options);

    //--------------------- construct trackable objects ----------------------//
    // @brief compute objects' shape feature
    // @params[OUT] object: object for computing shape feature
    // @return nothing
    void constructTrackedObjectShapeFeature(TrackableObjectPtr object_tracked);

    // @brief transform tracked object with given pose
    // @params[OUT] obj: tracked object for transfromation
    // @params[IN] pose: pose using for coordinate transformation
    // @return nothing
    void transformTrackedObject(TrackableObjectPtr object_obsved,
                                const Eigen::Matrix4d &pose);
    //--------------------- construct trackable objects ----------------------//

    // @brief collect tracked results
    // @params[OUT] tracked_objects: tracked objects with tracking information
    // @return nothing
    void collectTrackingObjects(std::vector<ObjectPtr> *objects_tracking);

 private:
    //--------- algorithm setup
    // Multi-object tracker
    MultiObjectTracker *multi_object_tracker_ = nullptr;
    // Motion estimation (运动估计)
    FilterType filter_method_;

    // Matcher: tracker<->observed object association
    MatcherType matcher_method_;
    BaseTrackerObsvMatcher *matcher_ = nullptr;

    feature::BaseFeatureExtractor *feature_extractor_ = nullptr;

    TrackingWorkerParams params_;

    // set offset to avoid huge value float computing
    // global coordinate, initial pose's offset, to local coordinate: Velodyne
    Eigen::Vector3d global_to_local_offset_;
    Eigen::Matrix4d velo2world_pose_;

    // TODO(gary): maintain trajectory
    /// @note tracking objects' tracker_id ---> ground center pose
    // TODO(gary): store poses in World Coordinate
    std::map<IdType, Trajectory> trajectory_poses_;
    std::map<IdType, std::vector<IdType>> trajectory_segments_;
    std::map<IdType, double> trajectory_periods_;
    // tracking objects in World coordiante at current frame
    std::vector<ObjectPtr> tracking_objects_;

    //--------- System states
    // Tracking system's timestamp
    double time_stamp_;

    // booted after bootloader()
    bool booted_;
};  // class HmTrackingWorker

}  // namespace tracking
}  // namespace autosense

#endif  // TRACKING_INCLUDE_TRACKING_HM_TRACKING_WORKER_HPP_
