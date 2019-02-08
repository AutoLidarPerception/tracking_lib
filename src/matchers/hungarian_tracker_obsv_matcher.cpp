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
#include "tracking/matchers/hungarian_tracker_obsv_matcher.hpp"
#include <ros/ros.h>

#include "common/algos/graph.hpp"
#include "common/algos/hungarian_bigraph_matcher.hpp"
#include "tracking/distances/tracker_obsv_distance.hpp"

namespace autosense {
namespace tracking {
// 追踪器<->观测是否连通的关联距离阈值
float HungarianTrackerObsvMatcher::s_match_distance_maximum_ = 4.0f;

bool HungarianTrackerObsvMatcher::setMatchDistanceMaximum(
    const float &match_distance_maximum) {
    if (match_distance_maximum >= 0) {
        s_match_distance_maximum_ = match_distance_maximum;
        ROS_INFO_STREAM("match distance maximum of HungarianMatcher is "
                        << s_match_distance_maximum_);
        return true;
    }
    ROS_ERROR("invalid match distance maximum of HungarianMatcher!");
    return false;
}

/**
 * @brief match detected objects to tracks
 *  当前帧检测到的追踪障碍物能否与系统追踪器列表中的某个追踪器关联
 * @params[IN] objects_obsved: new detected objects for matching
 * @params[IN] trackers: maintaining tracks for matching
 * @params[IN] trackers_predict: predicted state of maintained tracks
 * @params[OUT] assignments: assignment pair of object & track
 * @params[OUT] unassigned_trackers: tracks without matched object
 * @params[OUT] unassigned_objects_obsved: objects without matched track
 * @return nothing
 * @result
 *  find associate tracker<->observed object pairs
 *  update tracker-associate observed object's associate
 * score(->association_score)
 *  find unassigned trackers/observed objects
 */
void HungarianTrackerObsvMatcher::match(
    std::vector<TrackableObjectPtr> *objects_obsved,
    const std::vector<ObjectTrackerPtr> &trackers,
    const std::vector<Eigen::VectorXf> &trackers_predict,
    std::vector<TrackerObsvPair> *assignments,
    std::vector<int> *unassigned_trackers,
    std::vector<int> *unassigned_objects_obsved) {
    // A. computing association matrix 计算关联距离矩阵
    // tracks.size()*objects->size()
    Eigen::MatrixXf association_mat(trackers.size(), objects_obsved->size());
    // 需要结合追踪器六维度的预测状态 ==> 计算关联距离矩阵(association_mat)
    computeAssociateMatrix(trackers, trackers_predict, *objects_obsved,
                           &association_mat);

    // B. computing connected components
    // 由关联距离矩阵通过关联距离阈值为追踪器和可追踪障碍物添加边==>构建跟踪-检测关联图
    // 划分得到跟踪-检测关联图的各个连通分量(obsv_components[i]+tracker_components[i])
    std::vector<std::vector<int>> tracker_components;
    std::vector<std::vector<int>> obsv_components;
    // TODO(gary): 子图：要么只有一个顶点，要么是一个二分图
    computeConnectedComponents(association_mat, s_match_distance_maximum_,
                               &tracker_components, &obsv_components);
    ROS_INFO_STREAM("HungarianTrackerObsvMatcher: partition graph into "
                    << tracker_components.size() << " sub-graphs.");

    // C. matching each sub-graph 在每个子图(连通分量)里面进行匹配
    // assignments: <tracks index, objects index>...
    assignments->clear();
    // 关联不到的跟踪/检测
    unassigned_trackers->clear();
    unassigned_objects_obsved->clear();
    // 为每一个跟踪在其连通分量内寻找最佳的检测跟踪匹配
    for (size_t i = 0u; i < tracker_components.size(); ++i) {
        std::vector<TrackerObsvPair> sub_assignments;
        std::vector<int> sub_unassigned_trackers;
        std::vector<int> sub_unassigned_obsvs;
        matchInComponent(association_mat, tracker_components[i],
                         obsv_components[i], &sub_assignments,
                         &sub_unassigned_trackers, &sub_unassigned_obsvs);
        for (size_t j = 0u; j < sub_assignments.size(); ++j) {
            int tracker_id = sub_assignments[j].first;
            int obsv_id = sub_assignments[j].second;
            assignments->push_back(sub_assignments[j]);
            float association_score = association_mat(tracker_id, obsv_id);
            // update tracker-associate observed object's associate score
            (*objects_obsved)[obsv_id]->association_score = association_score;
        }
        for (size_t j = 0u; j < sub_unassigned_trackers.size(); ++j) {
            unassigned_trackers->push_back(sub_unassigned_trackers[j]);
        }
        for (size_t j = 0u; j < sub_unassigned_obsvs.size(); ++j) {
            unassigned_objects_obsved->push_back(sub_unassigned_obsvs[j]);
        }
    }
}

/**
 * @brief 计算系统追踪器与检测到的可追踪障碍物的关联距离矩阵association_mat
 * @note Tracker->Obsvs Associate Distance Matrix
 *  | Tracker(i)        -->Obsv(j) -->Obsv(j+1) -->Obsv(j+...)  |
 *  | Tracker(i+1)          ...                                 |
 *  | Tracker(i+...)        ...                                 |
 * @param trackers
 * @param trackers_predict
 * @param objects_obsved
 * @param association_mat
 */
void HungarianTrackerObsvMatcher::computeAssociateMatrix(
    const std::vector<ObjectTrackerPtr> &trackers,
    const std::vector<Eigen::VectorXf> &trackers_predict,
    const std::vector<TrackableObjectPtr> &objects_obsved,
    Eigen::MatrixXf *association_mat) {
    // Compute matrix of association distance
    //   每个元素[rowIdx][colIdx]是追踪器tracks[rowIdx]跟检测到的可追踪障碍物
    //   new_objects[colIdx]之间的综合考量距离
    for (size_t i = 0u; i < trackers.size(); ++i) {
        for (size_t j = 0u; j < objects_obsved.size(); ++j) {
            // 综合考量距离包括追踪器trackers[i]跟检测障碍物new_objects[j]之间的
            //   运动一致性: 位移持续性+方向一致性
            //   外观一致性: 边框大小+点云数目+直方图持续一致
            (*association_mat)(i, j) = TrackerObsvDistance::computeDistance(
                trackers[i], trackers_predict[i], objects_obsved[j]);
        }
    }
}

/**
 * @brief compute connected components within given threshold
 * <1> 通过关联距离阈值构建 检测-追踪关联图
 * <2> 通过连通分量分析除去噪点(估计异常检测居多+也可能是已经不在范围内的追踪器)
 * <3> 输出每个连通分量内存在的追踪器和检测障碍物
 *  tracker_components[i]+objects_obsved_components[i]
 * @params[IN] association_mat: matrix of association distance
 * @params[IN] connected_threshold: threshold of connected components
 * @params[OUT] track_components: connected objects of given tracks
 * @params[OUT] obj_components: connected tracks of given objects
 * @return nothing
 */
void HungarianTrackerObsvMatcher::computeConnectedComponents(
    const Eigen::MatrixXf &association_mat,
    const float &connected_threshold,
    std::vector<std::vector<int>> *tracker_components,
    std::vector<std::vector<int>> *objects_obsved_components) {
    // Compute connected components within given threshold
    int num_tracker = association_mat.rows();
    int num_obsv = association_mat.cols();
    // 临时的追踪器-观测关联图==>用于求解连通分量
    std::vector<std::vector<int>> DG_tracker_obsv;
    DG_tracker_obsv.resize(num_tracker + num_obsv);
    for (size_t tracker = 0u; tracker < num_tracker; ++tracker) {
        for (size_t obsv = 0u; obsv < num_obsv; ++obsv) {
            // 构建追踪器-观测有向图 (二维数组 std::vector<std::vector<int>>)
            // 关联距离小于阈值connected_threshold的追踪器-观测存在有向边
            if (association_mat(tracker, obsv) <= connected_threshold) {
                DG_tracker_obsv[tracker].push_back(num_tracker + obsv);
                DG_tracker_obsv[obsv + num_tracker].push_back(tracker);
            }
        }
    }

    // 图的连通分量求解
    std::vector<std::vector<int>> components;
    common::algos::connectedComponentAnalysis(DG_tracker_obsv, &components);
    // resize ==> wastes some space
    tracker_components->clear();
    tracker_components->resize(components.size());
    objects_obsved_components->clear();
    objects_obsved_components->resize(components.size());
    for (size_t i = 0; i < components.size(); ++i) {
        for (size_t j = 0; j < components[i].size(); ++j) {
            // 追踪器下标<num_tracker
            int id = components[i][j];
            if (id < num_tracker) {
                (*tracker_components)[i].push_back(id);
            } else {
                id -= num_tracker;
                (*objects_obsved_components)[i].push_back(id);
            }
        }
    }
}

/**
 * @brief match detected objects to tracks in component level
 *  单个连通分量内的追踪器-观测匹配
 * @params[IN] association_mat: association matrix of all objects to tracks
 * 关联距离矩阵
 * @params[IN] tracker_component: component of track
 * 本连通分量内的追踪(通过index索引)
 * @params[IN] objects_obsved_component: component of object
 * 本连通分量内的检测(通过index索引)
 * @params[OUT] sub_assignments: component assignment pair of object & track
 * 本连通分量内的跟踪-检测最佳匹配
 * @params[OUT] sub_unassigned_trackers: component tracks not matched
 * 本连通分量内未能匹配检测的追踪(通过index索引)
 * @params[OUT] sub_unassigned_objects_obsved: component objects not matched
 * 本连通分量内未能匹配追踪的检测(通过index索引)
 * @return nothing
 */
void HungarianTrackerObsvMatcher::matchInComponent(
    const Eigen::MatrixXf &association_mat,
    const std::vector<int> &tracker_component,
    const std::vector<int> &objects_obsved_component,
    std::vector<TrackerObsvPair> *sub_assignments,
    std::vector<int> *sub_unassigned_trackers,
    std::vector<int> *sub_unassigned_objects_obsved) {
    sub_assignments->clear();
    sub_unassigned_trackers->clear();
    sub_unassigned_objects_obsved->clear();
    // A. failed to match if either components is empty
    if (tracker_component.empty()) {
        for (size_t i = 0u; i < objects_obsved_component.size(); ++i) {
            sub_unassigned_objects_obsved->push_back(
                objects_obsved_component[i]);
        }
    }
    if (objects_obsved_component.empty()) {
        for (size_t i = 0u; i < tracker_component.size(); ++i) {
            sub_unassigned_trackers->push_back(tracker_component[i]);
        }
    }
    if (tracker_component.empty() || objects_obsved_component.empty()) {
        return;
    }

    // B. if components perfectly match 连通分量1对1, 完美匹配
    if (tracker_component.size() == 1 && objects_obsved_component.size() == 1) {
        int tracker_id = tracker_component[0];
        int obsv_id = objects_obsved_component[0];
        if (association_mat(tracker_id, obsv_id) <= s_match_distance_maximum_) {
            sub_assignments->push_back(std::make_pair(tracker_id, obsv_id));
        } else {
            sub_unassigned_trackers->push_back(tracker_id);
            sub_unassigned_objects_obsved->push_back(obsv_id);
        }
        return;
    }
    // C. multi observed object <-> tracker match 连通分量多对多
    /// @note local means in local component
    std::vector<int> local2global_tracker;
    std::vector<int> local2global_obsv;
    std::vector<TrackerObsvPair> local_assignments;
    std::vector<int> local_unassigned_trackers;
    std::vector<int> local_unassigned_obsvs;
    Eigen::MatrixXf local_association_mat(tracker_component.size(),
                                          objects_obsved_component.size());
    local2global_tracker.resize(tracker_component.size());
    local2global_obsv.resize(objects_obsved_component.size());
    for (size_t i = 0u; i < tracker_component.size(); ++i) {
        local2global_tracker[i] = tracker_component[i];
    }
    for (size_t i = 0u; i < objects_obsved_component.size(); ++i) {
        local2global_obsv[i] = objects_obsved_component[i];
    }
    // 构建连通分量内的关联距离矩阵: local_association_mat
    for (size_t i = 0u; i < tracker_component.size(); ++i) {
        for (size_t j = 0u; j < objects_obsved_component.size(); ++j) {
            int tracker_id = tracker_component[i];
            int obsv_id = objects_obsved_component[j];
            local_association_mat(i, j) = association_mat(tracker_id, obsv_id);
        }
    }
    // 为本连通分量内的每个追踪器寻找最佳匹配观测
    local_assignments.resize(local_association_mat.cols());
    local_unassigned_trackers.assign(local_association_mat.rows(), -1);
    local_unassigned_obsvs.assign(local_association_mat.cols(), -1);
    assignObsvedObjects2Trackers(
        local_association_mat, s_match_distance_maximum_, &local_assignments,
        &local_unassigned_trackers, &local_unassigned_obsvs);

    for (size_t i = 0u; i < local_assignments.size(); ++i) {
        int global_tracker_id =
            local2global_tracker[local_assignments[i].first];
        int global_obsv_id = local2global_obsv[local_assignments[i].second];
        sub_assignments->push_back(
            std::make_pair(global_tracker_id, global_obsv_id));
    }
    for (size_t i = 0u; i < local_unassigned_trackers.size(); ++i) {
        int global_track_id =
            local2global_tracker[local_unassigned_trackers[i]];
        sub_unassigned_trackers->push_back(global_track_id);
    }
    for (size_t i = 0u; i < local_unassigned_obsvs.size(); ++i) {
        int global_object_id = local2global_obsv[local_unassigned_obsvs[i]];
        sub_unassigned_objects_obsved->push_back(global_object_id);
    }
}

/**
 * 借助匈牙利优化器为单个连通分量内的每个跟踪寻找检测最佳匹配
 * @brief assign objects to tracks using components
 * @params[IN] association_mat: matrix of association distance
 * 本连通分量内的关联距离矩阵
 * @params[IN] assign_distance_maximum: threshold distance of assignment
 * @params[OUT] assignments: assignment pair of matched object & track
 * @params[OUT] unassigned_trackers: tracks without matched object
 * @params[OUT] unassigned_objects_obsved: objects without matched track
 * @return nothing
 */
void HungarianTrackerObsvMatcher::assignObsvedObjects2Trackers(
    const Eigen::MatrixXf &association_mat,
    const double &assign_distance_maximum,
    std::vector<TrackerObsvPair> *assignments,
    std::vector<int> *unassigned_trackers,
    std::vector<int> *unassigned_objects_obsved) {
    // Assign objects to tracks with null tracks setup
    std::vector<int> tracker_idxs;
    std::vector<int> obsv_idxs;
    int num_trackers = association_mat.rows();
    int num_obsvs = association_mat.cols();
    /// @note build cost
    ///构建本连通分量内的代价图(追踪器->观测的代价+观测->观测的代价)
    std::vector<std::vector<double>> cost(num_trackers + num_obsvs);
    // 追踪->检测的代价=追踪->检测的关联距离
    for (int i = 0; i < num_trackers; ++i) {
        cost[i].resize(association_mat.cols());
        for (int j = 0; j < association_mat.cols(); ++j) {
            cost[i][j] = association_mat(i, j);
        }
    }
    // 观测->观测的代价=尽量不可达(除非观测->观测本身, 但也比关联距离阈值大)
    for (int i = 0; i < num_obsvs; ++i) {
        cost[i + num_trackers].resize(num_obsvs);
        for (int j = 0; j < num_obsvs; ++j) {
            if (j == i) {
                cost[i + num_trackers][j] = assign_distance_maximum * 1.2f;
            } else {
                cost[i + num_trackers][j] = 999999.0f;
            }
        }
    }

    common::algos::HungarianBigraphMatcher hungarian_optimizer(cost);
    /// @note Get minimize-cost matches
    hungarian_optimizer.minimize(&tracker_idxs, &obsv_idxs);

    /// @note Find assignments
    int num_assignments = 0;
    std::vector<bool> tracker_used(num_trackers + num_obsvs, false);
    std::vector<bool> obsv_used(num_obsvs, false);
    for (size_t i = 0u; i < tracker_idxs.size(); ++i) {
        // Filter put invalid id
        if (tracker_idxs[i] < 0 || tracker_idxs[i] >= num_trackers ||
            obsv_idxs[i] < 0 || obsv_idxs[i] >= num_obsvs) {
            continue;
        }
        // Minimize-cost matches should satisfy
        // threshold(通过关联距离阈值筛选最佳匹配)
        if (association_mat(tracker_idxs[i], obsv_idxs[i]) <
            assign_distance_maximum) {
            (*assignments)[num_assignments++] =
                std::make_pair(tracker_idxs[i], obsv_idxs[i]);
            tracker_used[tracker_idxs[i]] = true;
            obsv_used[obsv_idxs[i]] = true;
        }
    }
    assignments->resize(num_assignments);

    /// @note Find unassigned trackers
    unassigned_trackers->resize(association_mat.rows());
    int num_unassigned_trackers = 0;
    for (int i = 0; i < association_mat.rows(); ++i) {
        if (tracker_used[i] == false) {
            (*unassigned_trackers)[num_unassigned_trackers++] = i;
        }
    }
    unassigned_trackers->resize(num_unassigned_trackers);

    /// @note Find unassigned observed objects
    unassigned_objects_obsved->resize(association_mat.cols());
    int num_unassigned_obsvs = 0;
    for (int i = 0; i < association_mat.cols(); ++i) {
        if (obsv_used[i] == false) {
            (*unassigned_objects_obsved)[num_unassigned_obsvs++] = i;
        }
    }
    unassigned_objects_obsved->resize(num_unassigned_obsvs);
}

}  // namespace tracking
}  // namespace autosense
