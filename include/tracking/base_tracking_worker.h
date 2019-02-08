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
#ifndef TRACKING_INCLUDE_TRACKING_BASE_TRACKING_WORKER_H_
#define TRACKING_INCLUDE_TRACKING_BASE_TRACKING_WORKER_H_

#include <Eigen/Core>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "common/types/object.hpp"
#include "tracking/trackers/object_tracker.hpp"

namespace autosense {
namespace tracking {
struct TrackingOptions {
    TrackingOptions() = default;

    explicit TrackingOptions(Eigen::Matrix4d *pose) : velo2world_trans(pose) {}

    std::shared_ptr<Eigen::Matrix4d> velo2world_trans;

    // TODO(gary): HDMap provided information
    // HdmapStructPtr hdmap = nullptr;
    // HDMapInput* hdmap_input = NULL;
};

class BaseTrackingWorker {
 public:
    BaseTrackingWorker() {}

    virtual ~BaseTrackingWorker() {}

    // @brief: tracking objects.
    // @param[in] objects_obsved: timestamp.
    // @param[in] options: options.
    // @param[out] objects_tracked: current tracking objects.
    virtual bool track(const std::vector<ObjectPtr> &objects_obsved,
                       double timestamp,
                       const TrackingOptions &options,
                       std::vector<ObjectPtr> *objects_tracked) = 0;

    virtual inline const std::vector<ObjectTrackerPtr> &getTrackers() const = 0;

    virtual inline const std::vector<IdType> &getUnassignedTrackers() const = 0;

    virtual inline const std::vector<IdType> &getLostTrackers() const = 0;

    virtual inline const std::map<IdType, Trajectory> &collectTrajectories()
        const = 0;

    virtual inline const std::vector<ObjectPtr> &collectTrackingObjectsInWorld()
        const = 0;

    virtual std::vector<FixedTrajectory> collectFixedTrajectories() = 0;

    /**
     * @brief
     * @param timestamp
     *  used for calculating time interval between last prediction
     * @param trans_velo2world
     *  Tracking information processs in World coordinate,
     *    given transformation into local Velodyne coordinate
     * @return
     *  current tracking objects' expection
     */
    virtual std::vector<ObjectPtr> collectExpectedObjects(
        const double &timestamp, const Eigen::Matrix4d &trans_pose) const = 0;

    virtual std::string name() const = 0;
};  // class BaseTrackingWorker

}  // namespace tracking
}  // namespace autosense

#endif  // TRACKING_INCLUDE_TRACKING_BASE_TRACKING_WORKER_H_
