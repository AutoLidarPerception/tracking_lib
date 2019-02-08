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
#include "tracking/hm_tracking_worker.hpp"
#include <ros/ros.h>
#include <Eigen/Core>

#include "common/time.hpp"
#include "common/transform.hpp"  // common::transform::transformObject

#include "feature_extractors/feature_extractor_manager.hpp"
#include "tracking/distances/tracker_obsv_distance.hpp"
#include "tracking/filters/robust_kalman_filter.h"
#include "tracking/matchers/hungarian_tracker_obsv_matcher.hpp"

namespace autosense {
namespace tracking {

HmTrackingWorker::HmTrackingWorker(const TrackingWorkerParams &params)
    : params_(params), matcher_(nullptr), time_stamp_(0.0), booted_(false) {
    // A. Matcher setup
    // load match method
    if (!setMatcherMethod(params_.matcher_method_name)) {
        ROS_ERROR_STREAM("Failed to set tracking matcher method! " << name());
        return;
    }
    if (matcher_method_ == HUNGARIAN_MATCHER) {
        matcher_ = new HungarianTrackerObsvMatcher();
    } else {
        matcher_method_ = HUNGARIAN_MATCHER;
        matcher_ = new HungarianTrackerObsvMatcher();
        ROS_WARN("invalid matcher method! default HungarianMatcher in use!");
    }
    // load match distance maximum
    if (matcher_method_ == HUNGARIAN_MATCHER) {
        if (!HungarianTrackerObsvMatcher::setMatchDistanceMaximum(
                params_.matcher_match_distance_maximum)) {
            ROS_ERROR_STREAM("Failed to set match distance maximum! "
                             << name());
            return;
        }
    }
    // load location distance weight
    if (!TrackerObsvDistance::setLocationDistanceWeight(
            params_.matcher_location_distance_weight)) {
        ROS_ERROR_STREAM("Failed to set location distance weight! " << name());
        return;
    }
    // load direction distance weight
    if (!TrackerObsvDistance::setDirectionDistanceWeight(
            params_.matcher_direction_distance_weight)) {
        ROS_ERROR_STREAM("Failed to set direction distance weight! " << name());
        return;
    }
    // load bbox size distance weight
    if (!TrackerObsvDistance::setBboxSizeDistanceWeight(
            params_.matcher_bbox_size_distance_weight)) {
        ROS_ERROR_STREAM("Failed to set bbox size distance weight! " << name());
        return;
    }
    // load point num distance weight
    if (!TrackerObsvDistance::setPointNumDistanceWeight(
            params_.matcher_point_num_distance_weight)) {
        ROS_ERROR_STREAM("Failed to set point num distance weight! " << name());
        return;
    }
    // load histogram distance weight
    if (!TrackerObsvDistance::setHistogramDistanceWeight(
            params_.matcher_histogram_distance_weight)) {
        ROS_ERROR_STREAM("Failed to set histogram distance weight! " << name());
        return;
    }

    // B. Tracker Filter setup
    // 通过设置ObjectTracker::s_filter_method_属性在物体追踪器类中标记
    if (!setFilterMethod(params_.filter_method_name)) {
        ROS_ERROR_STREAM("Failed to set tracking filter method! " << name());
        return;
    } else {
        // using default Filter
        filter_method_ = ObjectTracker::s_filter_method_;
    }
    if (filter_method_ == ROBUST_KALMAN_FILTER) {
        RobustKalmanFilter::setUseAdaptive(params_.filter_use_adaptive);
        if (!RobustKalmanFilter::setAssociationScoreMaximum(
                params_.filter_association_score_maximum)) {
            ROS_ERROR_STREAM("Failed to set association score maximum! "
                             << name());
            return;
        }
        /*
          * 初始化KF噪声
          *	initial_velocity_noise 状态初始化噪声
          *	s_propagation_noise_ 预测噪声协方差矩阵
          *		| xy_propagation_noise 		0 0
          *|
          *		|		0		xy_propagation_noise
          *|
          *		| 		0 0
          *z_propagation_noise
          *|
          *	s_measurement_noise_ 观测噪声
          */
        if (!RobustKalmanFilter::initNoiseParams(
                params_.filter_measurement_noise,
                params_.filter_initial_velocity_noise,
                params_.filter_xy_propagation_noise,
                params_.filter_z_propagation_noise)) {
            ROS_ERROR_STREAM("Failed to set params for kalman filter! "
                             << name());
            return;
        }
        if (!RobustKalmanFilter::setBreakdownThresholdMaximum(
                params_.filter_breakdown_threshold_maximum)) {
            ROS_ERROR_STREAM("Failed to set breakdown threshold maximum! "
                             << name());
            return;
        }
    }

    // C. Basic Tracker setup
    // load tracking cached history size maximum
    if (!ObjectTracker::setTrackerCachedHistorySizeMaximum(
            params_.tracker_cached_history_size_maximum)) {
        ROS_ERROR_STREAM("Failed to set tracking cached history size maximum! "
                         << name());
        return;
    }
    // load acceleration maximum
    if (!ObjectTracker::setAccelerationNoiseMaximum(
            params_.tracker_acceleration_noise_maximum)) {
        ROS_ERROR_STREAM("Failed to set acceleration noise maximum! "
                         << name());
        return;
    }
    // load speed noise maximum
    if (!ObjectTracker::setSpeedNoiseMaximum(
            params_.tracker_speed_noise_maximum)) {
        ROS_ERROR_STREAM("Failed to set speed noise maximum! " << name());
        return;
    }
    // load tracking consecutive invisible maximum
    if (!MultiObjectTracker::setTrackerConsecutiveInvisibleMaximum(
            params_.tracker_consecutive_invisible_maximum)) {
        ROS_ERROR_STREAM(
            "Failed to set tracking consecutive invisible maximum! " << name());
        return;
    }
    // load tracking visible ratio minimum
    if (!MultiObjectTracker::setTrackerVisibleRatioMinimum(
            params_.tracker_visible_ratio_minimum)) {
        ROS_ERROR_STREAM("Failed to set track visible ratio minimum! "
                         << name());
        return;
    }
    multi_object_tracker_ = new MultiObjectTracker;

    // D. Tracking collect configure
    // load collect age minimum 只发布追踪到一定age的障碍物
    /*if (!setCollectAgeMinimum(params_.tracking_collect_age_minimum)) {
        ROS_ERROR_STREAM("Failed to set collect age minimum! " << name());
        return;
    }
    // load collect consecutive invisible maximum 追踪障碍物持续丢失上限
    if (!setCollectConsecutiveInvisibleMaximum(
            params_.tracking_collect_consecutive_invisible_maximum)) {
        ROS_ERROR_STREAM("Failed to set collect consecutive invisible maximum! "
                                 << name());
        return;
    }*/

    // feature extractor
    FeatureExtractorParams extractor_config;
    extractor_config.extractor_type = "Histogram";
    extractor_config.bin_size_for_histogram =
        static_cast<size_t>(params_.tracking_histogram_bin_size);
    feature_extractor_ =
        feature::createFeatureExtractor(extractor_config).get();

    velo2world_pose_ = Eigen::Matrix4d::Identity();
}

HmTrackingWorker::~HmTrackingWorker() {
    if (feature_extractor_) {
        delete feature_extractor_;
        feature_extractor_ = nullptr;
    }

    if (multi_object_tracker_) {
        delete multi_object_tracker_;
        multi_object_tracker_ = nullptr;
    }

    if (matcher_) {
        delete matcher_;
        matcher_ = nullptr;
    }
}

bool HmTrackingWorker::setMatcherMethod(
    const std::string &matcher_method_name) {
    if (matcher_method_name == "hungarian_matcher") {
        matcher_method_ = HUNGARIAN_MATCHER;
        ROS_INFO_STREAM("matcher method of " << name() << " is "
                                             << matcher_method_name);
        return true;
    }
    ROS_ERROR_STREAM("invalid matcher method name of " << name());
    return false;
}

bool HmTrackingWorker::setFilterMethod(const std::string &filter_method_name) {
    if (filter_method_name == "robust_kalman_filter") {
        filter_method_ = ROBUST_KALMAN_FILTER;
        ObjectTracker::setFilterMethod(filter_method_name);
        ROS_INFO_STREAM("filter method of " << name() << " is "
                                            << filter_method_name);
        return true;
    }
    ROS_ERROR_STREAM("invalid filter method name of " << name());
    return false;
}

/**
 * @brief track detected objects over consecutive frames
 * @params[IN] objects: recently detected objects
 * @params[IN] timestamp: timestamp of recently detected objects
 * @params[IN] options: tracker options with necessary information
 * @params[OUT] objects_tracked: tracked objects with tracking information
 * 当前帧处理后的追踪障碍物信息
 * @return true if track successfully, otherwise return false
 *  tracking infos are returned in objects_tracked if succeed
 */
bool HmTrackingWorker::track(const std::vector<ObjectPtr> &objects_obsved,
                             double timestamp,
                             const TrackingOptions &options,
                             std::vector<ObjectPtr> *objects_tracked) {
    common::Clock clock;
    ROS_WARN_STREAM("HmTrackingWorker::track with " << objects_obsved.size()
                                                    << " observed objects.");
    if (objects_tracked == nullptr) {
        return false;
    }
    // A. setup tracking system
    // 初次追踪, 所有观测障碍物添加进系统多物体追踪器
    if (!booted_) {
        booted_ = true;
        return bootloader(objects_obsved, timestamp, options, objects_tracked);
    }

    /// @note 追踪状态位于起始位置全局坐标系(World)/物体检测位于
    /// Velodyne局部坐标系
    // Eigen::Matrix4d velo2world_pose = Eigen::Matrix4d::Identity();
    if (options.velo2world_trans != nullptr) {
        velo2world_pose_ = *(options.velo2world_trans);
    } else {
        ROS_ERROR("Input velodyne_trans is null!");
        return false;
    }

    // 获取时间间隔==>用于计算速度, 更新噪声协方差矩阵等
    double time_diff = timestamp - time_stamp_;
    time_stamp_ = timestamp;

    // B. preprocessing
    // B.1 transform given pose to local one
    /*TransformPoseGlobal2Local(&velo2world_pose);
    ADEBUG << "velo2local_pose\n" << velo2world_pose;*/

    // B.2 construct objects for tracking 根据检测障碍物构建可追踪障碍物
    // 丰富包括点云直方图特征shape_features;重心;锚点;车道方向;
    // 航向角/地表中心/点云/2D边框等转换到起始位置全局坐标系(World)
    std::vector<TrackableObjectPtr> objects_trackable_transformed;
    constructTrackedObjects(objects_obsved, &objects_trackable_transformed,
                            velo2world_pose_, options);

    // C. prediction 预测现有追踪障碍物的状态(障碍物追踪器预测/追踪器KF核心预测)
    std::vector<Eigen::VectorXf> tracker_predicts;
    multi_object_tracker_->execPredict(&tracker_predicts, time_diff);

    // D. match observed objects to tracking trackers
    // 观测到的可追踪障碍物[transformed_objects]<-->多物体追踪器[trackers]关联
    std::vector<TrackerObsvPair> assignments;
    std::vector<int> unassigned_obsvs;
    std::vector<int> unassigned_trackers;
    std::vector<ObjectTrackerPtr> &trackers =
        multi_object_tracker_->getTrackers();
    matcher_->match(&objects_trackable_transformed, trackers, tracker_predicts,
                    &assignments, &unassigned_trackers, &unassigned_obsvs);
    ROS_INFO_STREAM(
        "multi-object-tracking: "
        << "trackers.size(): " << trackers.size() << "  "
        << "assignments.size(): " << assignments.size() << "  "
        << "unassigned_trackers.size(): " << unassigned_trackers.size() << "  "
        << "unassigned_obsvs.size(): " << unassigned_obsvs.size() << "  "
        << "time_diff: " << time_diff);

    // E. update trackers
    // E.1 update trackers with associated observed objects
    multi_object_tracker_->updateAssignedTrackers(
        &objects_trackable_transformed, assignments, time_diff);
    // TODO(gary): add new obsvs to assigned tracker's trajectory
    for (size_t i = 0u; i < assignments.size(); ++i) {
        int tracker_id = assignments[i].first;
        int obsv_id = assignments[i].second;
        auto iter = trajectory_segments_.find(tracker_id);
        if (iter != trajectory_segments_.end()) {
            iter->second.push_back(
                objects_trackable_transformed[obsv_id]->object_ptr->id);
        }
    }
    // E.2 update tracks without associated objects
    multi_object_tracker_->updateUnassignedTrackers(
        tracker_predicts, unassigned_trackers, time_diff);
    // E.3 Delete lost trackers in system tracking
    // multi_object_tracker_->removeLostTrackers();
    multi_object_tracker_->removeLostTrackers(&trajectory_poses_,
                                              &trajectory_periods_);
    // E.3 TODO process lost object's trajectory
    /*std::vector<ObjectTrackerPtr> lost_trackers =
    multi_object_tracker_->getLostTrackers();
    for (size_t idx = 0u; idx < lost_trackers.size(); ++idx) {
        auto iter = previous_poses_.find(lost_trackers[idx]->idx_);
        if (iter != previous_poses_.end()) {
            //TODO maintain already fixed trajectories for tld's tracking
            auto iter_periods =
    trajectory_periods_.find(lost_trackers[idx]->idx_);
            iter_periods->second = lost_trackers[idx]->period_;

            previous_poses_.erase(iter);
        }
    }*/

    // E.4 create new trackers for observed objects without associated trackers
    // multi_object_tracker_->createNewTrackers(objects_trackable_transformed,
    // unassigned_obsvs);
    multi_object_tracker_->createNewTrackers(
        objects_trackable_transformed, unassigned_obsvs, &trajectory_segments_,
        &trajectory_periods_);

    // F. collect tracked results
    // 筛选可靠追踪障碍物信息输出(只是筛选并不会更新或者移除系统追踪列表)
    collectTrackingObjects(objects_tracked);

    return true;
}

/**
 * @brief Tracking system bootloader in first frame when system booting
 * @param objects_obsved
 * @param timestamp
 * @param options
 * @param objects_tracked
 * @return
 */
bool HmTrackingWorker::bootloader(const std::vector<ObjectPtr> &objects_obsved,
                                  const double &timestamp,
                                  const TrackingOptions &options,
                                  std::vector<ObjectPtr> *objects_tracked) {
    common::Clock clock;

    // A. Transform into World center
    // Eigen::Matrix4d velo2world_pose = Eigen::Matrix4d::Identity();
    if (options.velo2world_trans != nullptr) {
        velo2world_pose_ = *(options.velo2world_trans);
    } else {
        ROS_ERROR("Input velo2world_trans is null!");
        return false;
    }

    /*
     * TODO 追踪状态位于起始位置全局坐标系(World)/物体检测位于 Velodyne
     * 局部坐标系
     *  观测量需要投影到世界坐标系, 统一在该坐标系下进行追踪状态更新
     *  返回可视化(可能还有其他用途...)的追踪物体则需要投影回 Velodyne
     * 局部坐标系
     */
    /*global_to_local_offset_ = Eigen::Vector3d(-velo2world_pose(0, 3),
                                              -velo2world_pose(1, 3),
                                              -velo2world_pose(2, 3));*/

    // B. preprocessing
    // B.1 coordinate transformation
    // velodyne->world pose 转换到世界坐标系
    /*TransformPoseGlobal2Local(&velo2world_pose);
    ADEBUG << "velo2local_pose\n" << velo2world_pose;*/
    // B.2 construct tracked objects(bounding
    // box+states[anchor_point;velocity;acceleration;])
    std::vector<TrackableObjectPtr> objects_trackable_transformed;
    // 从当前帧检测到的障碍物初始化追踪信息
    constructTrackedObjects(objects_obsved, &objects_trackable_transformed,
                            velo2world_pose_, options);

    // C. create trackers
    // 初次追踪时将上一步构造的所有追踪障碍物transformed_objects绑定追踪器,
    // 记录在系统追踪器列表object_tracks_中
    std::vector<int> unassigned_obsvs;
    unassigned_obsvs.resize(objects_trackable_transformed.size());
    std::iota(unassigned_obsvs.begin(), unassigned_obsvs.end(), 0);
    // multi_object_tracker_->createNewTrackers(objects_trackable_transformed,
    // unassigned_obsvs);
    multi_object_tracker_->createNewTrackers(
        objects_trackable_transformed, unassigned_obsvs, &trajectory_segments_,
        &trajectory_periods_);
    time_stamp_ = timestamp;

    // TODO(gary): D. collect system tracked results 筛选可靠的追踪障碍物输出
    collectTrackingObjects(objects_tracked);

    ROS_INFO_STREAM("HmTrackingWorker::bootloader. Took "
                    << clock.takeRealTime() << "ms.");

    return true;
}

//------------------------ construct trackable objects -----------------------//
/**
 * @brief 观测障碍物 ==> 可追踪障碍物
 *    丰富包括点云直方图特征shape_features;重心;锚点;车道方向;
 *    更多的操作是：航向角/地表中心/点云/2D边框等转换到世界坐标系
 * @brief construct tracked objects via necessray transformation & feature
 * computing
 * @param[IN] objects_obsved: observed objects for construction
 * @param[OUT] objects_tracked: constructed objects
 * @param[IN] pose: pose using for coordinate transformation
 * @param[IN] options: tracker options with HD Map information
 * @return nothing
 */
void HmTrackingWorker::constructTrackedObjects(
    const std::vector<ObjectPtr> &objects_obsved,
    std::vector<TrackableObjectPtr> *objects_tracked,
    const Eigen::Matrix4d &pose,
    const TrackingOptions &options) {
    common::Clock clock;
    size_t num_obsvs = objects_obsved.size();
    objects_tracked->clear();
    objects_tracked->resize(num_obsvs);
    for (size_t i = 0u; i < num_obsvs; ++i) {
        ObjectPtr obj(new Object());
        obj->clone(*objects_obsved[i]);
        //追踪障碍物的信息在 TrackedObject(ObjectPtr obj_ptr)中初始化
        (*objects_tracked)[i].reset(new TrackableObject(obj));

        // Computing shape featrue 计算每个障碍物的形状特征: 点云直方图 ==>
        // calculate match distance
        if (params_.tracking_use_histogram_for_match) {
            constructTrackedObjectShapeFeature((*objects_tracked)[i]);
        }

        // Transforming all tracked objects 将障碍物从相对坐标转换到世界坐标
        // 航向角转换
        //  Transform center 地表中心/重心转换到世界坐标系
        //  Transform cloud & polygon 点云和2D边框转换
        transformTrackedObject((*objects_tracked)[i], pose);

        // Setting barycenter as anchor point of tracked objects 锚点初始化
        Eigen::Vector3f anchor_point = (*objects_tracked)[i]->barycenter;
        (*objects_tracked)[i]->anchor_point = anchor_point;
    }

    ROS_INFO_STREAM("HmTrackingWorker::constructTrackedObjects. Took "
                    << clock.takeRealTime() << "ms.");
}

/**
 * @brief Compute tracked object's shape feature(x/y/z histogram)
 * @param object_tracked
 */
void HmTrackingWorker::constructTrackedObjectShapeFeature(
    TrackableObjectPtr object_tracked) {
    ObjectPtr &temp_object = object_tracked->object_ptr;
    feature_extractor_->compute(*(temp_object->cloud),
                                &temp_object->shape_features);
}

/**
 * @brief transform tracked object with given pose
 * @note current velodyne pose ==> world coordinate(localization)
 * @param object_tracked
 * @param pose
 */
void HmTrackingWorker::transformTrackedObject(TrackableObjectPtr object_tracked,
                                              const Eigen::Matrix4d &pose) {
    //  航向角转换
    //   Transform center 地表中心/重心转换到世界坐标系
    //   Transform cloud & polygon 点云和2D边框转换
    common::transform::transformBuiltObject(pose, object_tracked->object_ptr);

    /// @note Trackable Objects inherit direction/ground center from
    ///   no-transformed built objects, compute barycenter from no-transformed
    ///   cloud transform direction
    Eigen::Vector3f &dir = object_tracked->direction;
    dir = (pose * Eigen::Vector4d(dir(0), dir(1), dir(2), 0))
              .head(3)
              .cast<float>();
    // transform center
    Eigen::Vector3f &center = object_tracked->ground_center;
    center = (pose * Eigen::Vector4d(center(0), center(1), center(2), 1))
                 .head(3)
                 .cast<float>();
    // transform barycenter(重心初始化通过点云计算得到)
    Eigen::Vector3f &barycenter = object_tracked->barycenter;
    barycenter =
        (pose * Eigen::Vector4d(barycenter(0), barycenter(1), barycenter(2), 1))
            .head(3)
            .cast<float>();
}
//------------------------ construct trackable objects -----------------------//

//----------------------------- collect status -------------------------------//
/**
 * @brief Collect system's tracking results for reporting,
 *      include objects may be occluded temporarily 物体可能暂时被遮挡
 *  筛选可靠追踪障碍物信息输出(只是筛选并不会更新或者移除系统追踪列表)
 * @param objects_tracked: tracking objects with tracking information
 */
void HmTrackingWorker::collectTrackingObjects(
    std::vector<ObjectPtr> *objects_tracked) {
    common::Clock clock;

    const std::vector<ObjectTrackerPtr> &trackers =
        multi_object_tracker_->getTrackers();
    objects_tracked->resize(trackers.size());
    tracking_objects_.clear();

    int num_objects_collected = 0;
    // 对每个追踪器追踪物体进行筛选
    for (size_t i = 0u; i < trackers.size(); ++i) {
        // <1>不输出追踪丢失的障碍物
        if (trackers[i]->consecutive_invisible_count_ >
            params_.tracking_collect_consecutive_invisible_maximum) {
            continue;
        }
        // <2>追踪器存活帧数需要满足要求
        if (trackers[i]->age_ < params_.tracking_collect_age_minimum) {
            continue;
        }
        // <3>障碍物实体(用于显示/最终的输出等)
        ObjectPtr obj(new Object);
        // 从追踪器中提取追踪障碍物
        TrackableObjectPtr result_obj = trackers[i]->current_object_;
        obj->clone(*(result_obj->object_ptr));

        // fill tracked information of object
        // size
        obj->length = result_obj->size[0];
        obj->width = result_obj->size[1];
        obj->height = result_obj->size[2];
        // direction
        obj->direction = result_obj->direction.cast<double>();
        // yaw orientation
        const Eigen::Vector3d coord_dir(1.0, 0.0, 0.0);
        obj->yaw_rad =
            common::geometry::computeTheta2dXyBetweenVectors<Eigen::Vector3d>(
                coord_dir, obj->direction);
        // tracking information
        obj->tracker_id = trackers[i]->idx_;
        // 持续跟踪时间
        obj->tracking_time = trackers[i]->period_;
        obj->type = result_obj->type;
        // position states
        obj->ground_center = result_obj->ground_center.cast<double>();
        // TODO(gary): maintain collected objects' trajectory, extend or create
        //   object's trajectory
        auto iter = trajectory_poses_.find(trackers[i]->idx_);
        if (iter == trajectory_poses_.end()) {
            // maintain previous pose array
            Trajectory poses;
            poses.push_back(result_obj->ground_center);
            trajectory_poses_.insert(std::make_pair(trackers[i]->idx_, poses));
        } else {
            iter->second.push_back(result_obj->ground_center);
        }
        // anchor_point
        obj->anchor_point = result_obj->anchor_point.cast<double>();
        // velocity
        obj->velocity = result_obj->velocity.cast<double>();
        obj->velocity_uncertainty = result_obj->velocity_uncertainty;

        // TODO(gary): collect objects in World coordinate
        ObjectPtr obj_in_world(new Object);
        obj_in_world->clone(*obj);
        tracking_objects_.push_back(obj_in_world);

        // <4>restore original local Velodyne coordinate 世界坐标-->局部坐标
        Eigen::Matrix4d pose_world2velo = velo2world_pose_.inverse();
        // direction
        common::transform::transformDirection(pose_world2velo,
                                              &(obj->direction));
        obj->yaw_rad =
            common::geometry::computeTheta2dXyBetweenVectors<Eigen::Vector3d>(
                coord_dir, obj->direction);
        // ground center
        common::transform::transformGroundBox(pose_world2velo,
                                              &(obj->ground_center));
        // velocity
        common::transform::transformVelocity(pose_world2velo, &(obj->velocity));
        // segment cloud
        common::transform::transformPointCloud<PointI>(pose_world2velo,
                                                       obj->cloud);

        (*objects_tracked)[num_objects_collected] = obj;
        num_objects_collected++;
    }

    objects_tracked->resize(num_objects_collected);

    ROS_INFO_STREAM("HmTrackingWorker::collectTrackingObjects. Took "
                    << clock.takeRealTime() << "ms.");
}
//----------------------------- collect status -------------------------------//
}  // namespace tracking
}  // namespace autosense
