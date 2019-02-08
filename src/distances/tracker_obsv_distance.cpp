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
#include "tracking/distances/tracker_obsv_distance.hpp"

#include <ros/ros.h>

#include "common/geometry.hpp"

namespace autosense {
namespace tracking {

double TrackerObsvDistance::s_location_distance_weight_ = 0.6;
double TrackerObsvDistance::s_direction_distance_weight_ = 0.2;
double TrackerObsvDistance::s_bbox_size_distance_weight_ = 0.1;
double TrackerObsvDistance::s_point_num_distance_weight_ = 0.1;
double TrackerObsvDistance::s_histogram_distance_weight_ = 0.5;

bool TrackerObsvDistance::setLocationDistanceWeight(
    const float &location_distance_weight) {
    if (location_distance_weight >= 0) {
        s_location_distance_weight_ = location_distance_weight;
        ROS_INFO("location distance weight of %s is %f.", Name().c_str(),
                 s_location_distance_weight_);
        return true;
    }
    ROS_ERROR("invalid location distance weight of %s!", Name().c_str());
    return false;
}

bool TrackerObsvDistance::setDirectionDistanceWeight(
    const float &direction_distance_weight) {
    if (direction_distance_weight >= 0) {
        s_direction_distance_weight_ = direction_distance_weight;
        ROS_INFO("direction distance weight of %s is %f.", Name().c_str(),
                 s_direction_distance_weight_);
        return true;
    }
    ROS_ERROR("invalid direction distance weight of %s!", Name().c_str());
    return false;
}

bool TrackerObsvDistance::setBboxSizeDistanceWeight(
    const float &bbox_size_distance_weight) {
    if (bbox_size_distance_weight >= 0) {
        s_bbox_size_distance_weight_ = bbox_size_distance_weight;
        ROS_INFO("bbox size distance weight of %s is %f.", Name().c_str(),
                 s_bbox_size_distance_weight_);
        return true;
    }
    ROS_ERROR("invalid bbox size distance weight of %s!", Name().c_str());
    return false;
}

bool TrackerObsvDistance::setPointNumDistanceWeight(
    const float &point_num_distance_weight) {
    if (point_num_distance_weight >= 0) {
        s_point_num_distance_weight_ = point_num_distance_weight;
        ROS_INFO("point num distance weight of %s is %f.", Name().c_str(),
                 s_point_num_distance_weight_);
        return true;
    }
    ROS_ERROR("invalid point num distance weight of %s!", Name().c_str());
    return false;
}

bool TrackerObsvDistance::setHistogramDistanceWeight(
    const float &histogram_distance_weight) {
    if (histogram_distance_weight >= 0) {
        s_histogram_distance_weight_ = histogram_distance_weight;
        ROS_INFO("histogram distance weight of %s is %f.", Name().c_str(),
                 s_histogram_distance_weight_);
        return true;
    }
    ROS_ERROR("invalid histogram distance weight of %s!", Name().c_str());
    return false;
}

/**
 * @brief 根据一系列关联特征(包括运动一致性,
 * 外观一致性等)计算追踪器到观测可追踪障碍物之间的距离
 * @note distance element for Tracker->Obsvs Associate Matrix
 *  | Tracker(i)        -->Obsv(j) -->Obsv(j+1) -->Obsv(j+...)  |
 *  | Tracker(i+1)          ...                                 |
 *  | Tracker(i+...)        ...                                 |
 * @param tracker
 * @param tracker_predict
 * @param object_obsved
 * @return
 */
float TrackerObsvDistance::computeDistance(
    ObjectTrackerConstPtr tracker,
    const Eigen::VectorXf &tracker_predict,
    TrackableObjectConstPtr object_obsved) {
    // Compute distance for given tracker & object
    // 评估运动一致性: 位置间距+方向间距(需要结合状态预测)
    float location_dist =
        computeLocationDistance(tracker, tracker_predict, object_obsved);
    float direction_dist =
        computeDirectionDistance(tracker, tracker_predict, object_obsved);
    // 评估外观一致性: 边框间距+点云数目间距+点云直方图间距
    float bbox_size_dist = computeBboxSizeDistance(tracker, object_obsved);
    float point_num_dist = computePointNumDistance(tracker, object_obsved);
    float histogram_dist = computeHistogramDistance(tracker, object_obsved);

    // 加权和: 将上述关联特征组合成最终距离测量
    float result_distance = s_location_distance_weight_ * location_dist +
                            s_direction_distance_weight_ * direction_dist +
                            s_bbox_size_distance_weight_ * bbox_size_dist +
                            s_point_num_distance_weight_ * point_num_dist +
                            s_histogram_distance_weight_ * histogram_dist;
    return result_distance;
}

// TODO(gary): 这个需要一些理论依据寻找
float TrackerObsvDistance::computeLocationDistance(
    ObjectTrackerConstPtr tracker,
    const Eigen::VectorXf &tracker_predict,
    TrackableObjectConstPtr object_obsved) {
    // Compute locatin distance for given track & object
    // range from 0 to positive infinity
    TrackableObjectConstPtr last_obsv = tracker->current_object_;
    // x-y 锚点
    Eigen::Vector2f measured_anchor_point = object_obsved->anchor_point.head(2);
    Eigen::Vector2f predicted_anchor_point = tracker_predict.head(2);
    // 预测-观测 x-y偏差
    Eigen::Vector2f measurement_predict_diff =
        measured_anchor_point - predicted_anchor_point;
    float location_distance = measurement_predict_diff.norm();

    Eigen::Vector2f tracker_motion_dir = last_obsv->velocity.head(2);
    // norm is sqrt(x^2+y^2)
    float tracker_speed = tracker_motion_dir.norm();
    tracker_motion_dir /= tracker_speed;
    /* Assume location distance is generated from a normal distribution with
     * 具有对称方差的正态分布
     * symmetric variance. Modify its variance when track speed greater than
     * a threshold. Penalize variance in the orthogonal direction of motion. */
    if (tracker_speed > 2) {
        Eigen::Vector2f track_motion_orthogonal_dir =
            Eigen::Vector2f(tracker_motion_dir(1), -tracker_motion_dir(0));
        float motion_dir_distance =
            tracker_motion_dir(0) * measurement_predict_diff(0) +
            tracker_motion_dir(1) * measurement_predict_diff(1);
        float motion_orthogonal_dir_distance =
            track_motion_orthogonal_dir(0) * measurement_predict_diff(0) +
            track_motion_orthogonal_dir(1) * measurement_predict_diff(1);
        location_distance =
            sqrt(motion_dir_distance * motion_dir_distance * 0.25 +
                 motion_orthogonal_dir_distance *
                     motion_orthogonal_dir_distance * 4);
    }
    return location_distance;
}

float TrackerObsvDistance::computeDirectionDistance(
    ObjectTrackerConstPtr tracker,
    const Eigen::VectorXf &tracker_predict,
    TrackableObjectConstPtr object_obsved) {
    // Compute direction distance for given track & object
    // range from 0 to 2
    TrackableObjectConstPtr last_obsv = tracker->current_object_;
    Eigen::Vector3f old_anchor_point = last_obsv->anchor_point;
    Eigen::Vector3f new_anchor_point = object_obsved->anchor_point;
    // 锚点偏移
    Eigen::Vector3f anchor_point_shift = new_anchor_point - old_anchor_point;
    anchor_point_shift(2) = 0;
    Eigen::Vector3f predicted_tracker_motion = tracker_predict.head(6).tail(3);
    predicted_tracker_motion(2) = 0;

    double cos_theta = 0.994;  // average cos
    if (!anchor_point_shift.head(2).isZero() &&
        !predicted_tracker_motion.head(2).isZero()) {
        cos_theta = common::geometry::computeCosTheta2dXyBetweenVectors(
            predicted_tracker_motion, anchor_point_shift);
    }
    float direction_distance = -cos_theta + 1.0;
    return direction_distance;
}

// TODO(gary): 需要结合边框构建进行思考
float TrackerObsvDistance::computeBboxSizeDistance(
    ObjectTrackerConstPtr tracker, TrackableObjectConstPtr object_obsved) {
    // Compute bbox size distance for given track & object
    // range from 0 to 1
    TrackableObjectConstPtr last_obsv = tracker->current_object_;
    Eigen::Vector3f old_bbox_dir = last_obsv->direction;
    Eigen::Vector3f new_bbox_dir = object_obsved->direction;
    Eigen::Vector3f old_bbox_size = last_obsv->size;
    Eigen::Vector3f new_bbox_size = object_obsved->size;

    float size_distance = 0.0;
    double dot_val_00 = fabs(old_bbox_dir(0) * new_bbox_dir(0) +
                             old_bbox_dir(1) * new_bbox_dir(1));
    double dot_val_01 = fabs(old_bbox_dir(0) * new_bbox_dir(1) -
                             old_bbox_dir(1) * new_bbox_dir(0));
    bool bbox_dir_close = dot_val_00 > dot_val_01;

    if (bbox_dir_close) {
        float diff_1 = fabs(old_bbox_size(0) - new_bbox_size(0)) /
                       std::max(old_bbox_size(0), new_bbox_size(0));
        float diff_2 = fabs(old_bbox_size(1) - new_bbox_size(1)) /
                       std::max(old_bbox_size(1), new_bbox_size(1));
        size_distance = std::min(diff_1, diff_2);
    } else {
        float diff_1 = fabs(old_bbox_size(0) - new_bbox_size(1)) /
                       std::max(old_bbox_size(0), new_bbox_size(1));
        float diff_2 = fabs(old_bbox_size(1) - new_bbox_size(0)) /
                       std::max(old_bbox_size(1), new_bbox_size(0));
        size_distance = std::min(diff_1, diff_2);
    }
    return size_distance;
}

// 点云数目距离
float TrackerObsvDistance::computePointNumDistance(
    ObjectTrackerConstPtr tracker, TrackableObjectConstPtr object_obsved) {
    // Compute point num distance for given track & object
    // range from 0 and 1
    TrackableObjectConstPtr last_obsv = tracker->current_object_;
    int old_point_number = last_obsv->object_ptr->cloud->size();
    int new_point_number = object_obsved->object_ptr->cloud->size();
    float point_num_distance = fabs(old_point_number - new_point_number) *
                               1.0f /
                               std::max(old_point_number, new_point_number);
    return point_num_distance;
}

/**
 * 点云直方图特征L1距离∑|old_object_shape_features[0...N] -
 * new_object_shape_features[0...N]|;
 * TODO How to compute TrackedObject's shape_features
 *  computed by FeatureDescriptor==>(3*bin_size) feature vector
 */
float TrackerObsvDistance::computeHistogramDistance(
    ObjectTrackerConstPtr tracker, TrackableObjectConstPtr object_obsved) {
    // Compute histogram distance for given tracker & obsved object
    // range from 0 to 3
    TrackableObjectConstPtr last_obsv = tracker->current_object_;
    const Feature &old_object_shape_features =
        last_obsv->object_ptr->shape_features;
    const Feature &new_object_shape_features =
        object_obsved->object_ptr->shape_features;
    if (old_object_shape_features.size() != new_object_shape_features.size()) {
        ROS_ERROR(
            "sizes of compared features not matched! ObjectTrackerDistance");
        return FLT_MAX;
    }

    float histogram_distance = 0.0;
    for (size_t i = 0u; i < old_object_shape_features.size(); ++i) {
        histogram_distance += std::fabs(old_object_shape_features.at(i).value -
                                        new_object_shape_features.at(i).value);
    }

    return histogram_distance;
}

}  // namespace tracking
}  // namespace autosense
