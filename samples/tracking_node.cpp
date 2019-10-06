/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <memory>

#include "common/msgs/autosense_msgs/PointCloud2Array.h"
#include "common/msgs/autosense_msgs/TrackingFixedTrajectoryArray.h"
#include "common/msgs/autosense_msgs/TrackingObjectArray.h"

#include "common/bounding_box.hpp"
#include "common/color.hpp"
#include "common/parameter.hpp"
#include "common/publisher.hpp"
#include "common/time.hpp"
#include "common/transform.hpp"
#include "common/types/object.hpp"
#include "common/types/type.h"
#include "object_builders/object_builder_manager.hpp"
#include "tracking/tracking_worker_manager.hpp"

const std::string param_ns_prefix_ = "tracking";  // NOLINT
std::string local_frame_id_, global_frame_id_;    // NOLINT
int tf_timeout_ms_ = 0;
double threshold_contian_IoU_ = 0.0;
autosense::TrackingWorkerParams tracking_params_;
// ROS Subscriber
ros::Subscriber pcs_segmented_sub_;
std::unique_ptr<tf::TransformListener> tf_listener_;
// ROS Publisher
ros::Publisher segments_coarse_pub_;
ros::Publisher segments_predict_pub_;
ros::Publisher segments_pub_;
ros::Publisher tracking_output_objects_pub_;
ros::Publisher tracking_output_trajectories_pub_;
ros::Publisher tracking_objects_pub_;
ros::Publisher tracking_objects_cloud_pub_;
ros::Publisher tracking_objects_velocity_pub_;
ros::Publisher tracking_objects_trajectory_pub_;
/// @note Core components
std::unique_ptr<autosense::object_builder::BaseObjectBuilder> object_builder_ =
    nullptr;
std::unique_ptr<autosense::tracking::BaseTrackingWorker> tracking_worker_ =
    nullptr;

// TODO(chenshengjie): callback function as fast as possible
void OnSegmentClouds(
    const autosense_msgs::PointCloud2ArrayConstPtr &segments_msg) {
    const double kTimeStamp = segments_msg->header.stamp.toSec();
    ROS_INFO("Clusters size: %d at %lf.", segments_msg->clouds.size(),
             kTimeStamp);

    std_msgs::Header header;
    header.frame_id = local_frame_id_;
    header.stamp = segments_msg->header.stamp;

    // initial coarse segments directly from segment node or after classified by
    // learning node
    std::vector<autosense::PointICloudPtr> segment_clouds;
    for (size_t i = 0u; i < segments_msg->clouds.size(); ++i) {
        autosense::PointICloudPtr cloud(new autosense::PointICloud);
        pcl::fromROSMsg(segments_msg->clouds[i], *cloud);
        segment_clouds.push_back(cloud);
    }

    // current pose
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    auto status = autosense::common::transform::getVelodynePose(
        *tf_listener_, local_frame_id_, global_frame_id_, kTimeStamp, &pose);
    if (!status) {
        ROS_WARN("Failed to fetch current pose, tracking skipped...");
        return;
    }
    auto velo2world = std::make_shared<Eigen::Matrix4d>(pose);

    // object builder
    autosense::common::Clock clock_builder;
    std::vector<autosense::ObjectPtr> objects;
    object_builder_->build(segment_clouds, &objects);
    ROS_INFO_STREAM("Objects built. Took " << clock_builder.takeRealTime()
                                           << "ms.");

    // visualize initial coarse segments
    autosense::common::publishObjectsMarkers(
        segments_coarse_pub_, header, autosense::common::MAGENTA.rgbA, objects);

    /**
     * @brief Use Tracking temporal information to improve segmentation
     * @note
     *  <1> project coarse segmentation into World coordinate
     *  <2> get expected objectd (in world coordinate)
     *  <3> over-segmentation/under-segmentation detect and improve segmentation
     *  <4> project back to current Velodyne coordinate
     */
    std::vector<autosense::ObjectPtr> expected_objects =
        tracking_worker_->collectExpectedObjects(kTimeStamp, *velo2world);
    std::vector<autosense::ObjectPtr> obsv_objects(objects.begin(),
                                                   objects.end());
    /// @note Tracking 就靠粗分割进行初始化
    if (!expected_objects.empty()) {
        /// @note 通过Tracking信息提升分割：过/欠分割,
        /// 主要目的是因为遮挡等造成的过分割
        // TODO(chenshengjie): 能否结合一些历史做一下当前完整修饰
        for (size_t expected_idx = 0u; expected_idx < expected_objects.size();
             ++expected_idx) {
            autosense::common::bbox::GroundBox gbox_expected;
            autosense::common::bbox::toGroundBox(expected_objects[expected_idx],
                                                 &gbox_expected);

            autosense::ObjectPtr object_merged(new autosense::Object);

            for (size_t obsv_idx = 0u; obsv_idx < objects.size(); ++obsv_idx) {
                autosense::common::bbox::GroundBox gbox_obsv;
                autosense::common::bbox::toGroundBox(objects[obsv_idx],
                                                     &gbox_obsv);

                // combining all connected components within an expected
                // object’s bounding box into a new one
                if (autosense::common::bbox::groundBoxOverlap(
                        gbox_expected, gbox_obsv, threshold_contian_IoU_)) {
                    *object_merged->cloud += *objects[obsv_idx]->cloud;
                    obsv_objects[obsv_idx]->cloud->clear();
                }
            }
            // build merged object
            object_builder_->build(object_merged);
            // maintain tracking-help segmented objects
            obsv_objects.push_back(object_merged);
        }
        // remove all connected components at once
        auto iter = obsv_objects.begin();
        for (; iter != obsv_objects.end();) {
            if ((*iter)->cloud->empty()) {
                iter = obsv_objects.erase(iter);
            } else {
                ++iter;
            }
        }
    }

    // visualize expected objects
    autosense::common::publishObjectsMarkers(segments_predict_pub_, header,
                                             autosense::common::DARKGREEN.rgbA,
                                             expected_objects);
    // visualize segmentation results
    autosense::common::publishObjectsMarkers(
        segments_pub_, header, autosense::common::GREEN.rgbA, obsv_objects);

    autosense::tracking::TrackingOptions tracking_options;
    tracking_options.velo2world_trans = velo2world;
    std::vector<autosense::ObjectPtr> tracking_objects_velo;
    autosense::common::Clock clock_tracking;
    tracking_worker_->track(obsv_objects, kTimeStamp, tracking_options,
                            &tracking_objects_velo);
    ROS_INFO_STREAM("Finish tracking. "
                    << tracking_objects_velo.size() << " Objects Tracked. Took "
                    << clock_tracking.takeRealTime() << "ms.");

    /**
     * publish tracking object clouds for classification
     *   object state: ground center & yaw & velocity
     *   object size, observed segment & its id
     */
    const std::vector<autosense::ObjectPtr> &tracking_objects_world =
        tracking_worker_->collectTrackingObjectsInWorld();
    autosense::common::publishTrackingObjects(tracking_output_objects_pub_,
                                              header, tracking_objects_world);
    // publish fixed trajectory for classification
    const std::vector<autosense::FixedTrajectory> &fixed_trajectories =
        tracking_worker_->collectFixedTrajectories();
    autosense::common::publishTrackingFixedTrajectories(
        tracking_output_trajectories_pub_, header, fixed_trajectories);

    // visualize tracking process results, Object Trajectories
    const std::map<autosense::IdType, autosense::Trajectory> &trajectories =
        tracking_worker_->collectTrajectories();
    autosense::common::publishObjectsTrajectory(
        tracking_objects_trajectory_pub_, header, pose.inverse(), trajectories);
    autosense::common::publishObjectsMarkers(tracking_objects_pub_, header,
                                             autosense::common::CYAN.rgbA,
                                             tracking_objects_velo);
    // construct tracking-help segmentation results
    std::vector<autosense::PointICloudPtr> objects_cloud;
    for (size_t idx = 0u; idx < tracking_objects_velo.size(); ++idx) {
        objects_cloud.push_back(tracking_objects_velo[idx]->cloud);
    }
    autosense::common::publishClustersCloud<autosense::PointI>(
        tracking_objects_cloud_pub_, header, objects_cloud);
    // Velocity value and direction
    autosense::common::publishObjectsVelocityArrow(
        tracking_objects_velocity_pub_, header, autosense::common::RED.rgbA,
        tracking_objects_velo);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tracking_node");

    // Node handle
    ros::NodeHandle nh = ros::NodeHandle();
    ros::NodeHandle private_nh = ros::NodeHandle("~");
    ros::AsyncSpinner spiner(1);

    // Load ROS parameters from rosparam server
    private_nh.getParam(param_ns_prefix_ + "/local_frame_id", local_frame_id_);
    private_nh.getParam(param_ns_prefix_ + "/global_frame_id",
                        global_frame_id_);
    private_nh.getParam(param_ns_prefix_ + "/tf_timeout_ms", tf_timeout_ms_);

    std::string sub_pcs_segmented_topic;
    int sub_pcs_queue_size;
    private_nh.getParam(param_ns_prefix_ + "/sub_pcs_segmented_topic",
                        sub_pcs_segmented_topic);
    private_nh.getParam(param_ns_prefix_ + "/sub_pcs_queue_size",
                        sub_pcs_queue_size);

    std::string pub_segments_coarse_topic, pub_segments_predict_topic,
        pub_segments_topic;
    private_nh.getParam(param_ns_prefix_ + "/pub_segments_coarse_topic",
                        pub_segments_coarse_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_segments_predict_topic",
                        pub_segments_predict_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_segments_topic",
                        pub_segments_topic);

    std::string pub_output_objects_topic, pub_output_trajectories_topic;
    private_nh.getParam(param_ns_prefix_ + "/pub_output_objects_topic",
                        pub_output_objects_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_output_trajectories_topic",
                        pub_output_trajectories_topic);

    std::string pub_tracking_objects_topic, pub_tracking_objects_cloud_topic,
        pub_tracking_objects_velocity_topic,
        pub_tracking_objects_trajectory_topic;
    private_nh.getParam(param_ns_prefix_ + "/pub_tracking_objects_topic",
                        pub_tracking_objects_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_tracking_objects_cloud_topic",
                        pub_tracking_objects_cloud_topic);
    private_nh.getParam(
        param_ns_prefix_ + "/pub_tracking_objects_velocity_topic",
        pub_tracking_objects_velocity_topic);
    private_nh.getParam(
        param_ns_prefix_ + "/pub_tracking_objects_trajectory_topic",
        pub_tracking_objects_trajectory_topic);

    private_nh.param<double>(param_ns_prefix_ + "/threshold_contian_IoU",
                             threshold_contian_IoU_, 1.0);
    tracking_params_ =
        autosense::common::getTrackingWorkerParams(nh, param_ns_prefix_);

    // Init core compoments
    object_builder_ = autosense::object_builder::createObjectBuilder();
    if (nullptr == object_builder_) {
        ROS_FATAL("Failed to create object_builder_.");
        return -1;
    }
    tracking_worker_ =
        autosense::tracking::createTrackingWorker(tracking_params_);
    if (nullptr == tracking_worker_) {
        ROS_FATAL("Failed to create tracking_worker_.");
        return -1;
    }

    // Init subscribers and publishers
    pcs_segmented_sub_ = nh.subscribe<autosense_msgs::PointCloud2Array>(
        sub_pcs_segmented_topic, sub_pcs_queue_size, OnSegmentClouds);
    tf_listener_.reset(new tf::TransformListener);
    // segments
    segments_coarse_pub_ =
        private_nh.advertise<visualization_msgs::MarkerArray>(
            pub_segments_coarse_topic, 1);
    segments_predict_pub_ =
        private_nh.advertise<visualization_msgs::MarkerArray>(
            pub_segments_predict_topic, 1);
    segments_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>(
        pub_segments_topic, 1);
    // tracking infos for debugging
    tracking_objects_pub_ =
        private_nh.advertise<visualization_msgs::MarkerArray>(
            pub_tracking_objects_topic, 1);
    tracking_objects_cloud_pub_ =
        private_nh.advertise<sensor_msgs::PointCloud2>(
            pub_tracking_objects_cloud_topic, 1);
    tracking_objects_velocity_pub_ =
        private_nh.advertise<visualization_msgs::MarkerArray>(
            pub_tracking_objects_velocity_topic, 1);
    tracking_objects_trajectory_pub_ =
        private_nh.advertise<visualization_msgs::MarkerArray>(
            pub_tracking_objects_trajectory_topic, 1);
    // the whole tracking output
    tracking_output_objects_pub_ =
        private_nh.advertise<autosense_msgs::TrackingObjectArray>(
            pub_output_objects_topic, 1);
    tracking_output_trajectories_pub_ =
        private_nh.advertise<autosense_msgs::TrackingFixedTrajectoryArray>(
            pub_output_trajectories_topic, 1);

    spiner.start();
    ROS_INFO("tracking_node started...");

    ros::waitForShutdown();
    ROS_INFO("tracking_node exited...");

    return 0;
}
