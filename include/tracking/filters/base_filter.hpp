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
#ifndef TRACKING_INCLUDE_TRACKING_FILTERS_BASE_FILTER_HPP_
#define TRACKING_INCLUDE_TRACKING_FILTERS_BASE_FILTER_HPP_

#include <string>

#include "common/types/object.hpp"
#include "common/types/trackable_object.hpp"

namespace autosense {
namespace tracking {

typedef enum {
    ROBUST_KALMAN_FILTER = 0,
} FilterType;

class BaseFilter {
 public:
    BaseFilter() { name_ = "BaseFilter"; }

    virtual ~BaseFilter() {}

    // @brief initialize the state of filter
    // @params[IN] anchor_point: initial anchor point for filtering
    // @params[IN] velocity: initial velocity for filtering
    // @return nothing
    virtual void initState(const Eigen::Vector3f &anchor_point,
                           const Eigen::Vector3f &velocity) = 0;

    // @brief predict the state of filter
    // @params[IN] time_diff: time interval for predicting
    // @return predicted states of filtering
    virtual Eigen::VectorXf execPredict(const double &time_diff) = 0;

    // @brief update filter with object
    // @params[IN] new_object: recently detected object for current updating
    // @params[IN] old_object: last detected object for last updating
    // @params[IN] time_diff: time interval from last updating
    // @return nothing
    virtual void updateWithObservation(TrackableObjectConstPtr new_object,
                                       TrackableObjectConstPtr old_object,
                                       const double &time_diff) = 0;

    // @brief update filter without object
    // @params[IN] time_diff: time interval from last updating
    // @return nothing
    virtual void updateWithoutObservation(const double &time_diff) = 0;

    // @brief get state of filter
    // @params[OUT] anchor_point: anchor point of current state
    // @params[OUT] velocity: velocity of current state
    // @return nothing
    virtual void getState(Eigen::Vector3f *anchor_point,
                          Eigen::Vector3f *velocity) = 0;

    // @brief get state of filter with accelaration
    // @params[OUT] anchor_point: anchor point of current state
    // @params[OUT] velocity: velocity of current state
    // @params[OUT] velocity_accelaration: accelaration of curret state
    // @return nothing
    virtual void getState(Eigen::Vector3f *anchor_point,
                          Eigen::Vector3f *velocity,
                          Eigen::Vector3f *velocity_accelaration) = 0;

    virtual void getAccelerationGain(Eigen::Vector3f *acceleration_gain) = 0;

    // @brief get online covariance of filter
    // @params[OUT] online_covariance: online covariance
    // @return noting
    virtual void getOnlineCovariance(Eigen::Matrix3d *online_covariance) = 0;

    // @brief get name of filter
    // @return name of filter
    std::string Name() { return name_; }

 protected:
    std::string name_;
};  // class BaseFilter

}  // namespace tracking
}  // namespace autosense

#endif  // TRACKING_INCLUDE_TRACKING_FILTERS_BASE_FILTER_HPP_
