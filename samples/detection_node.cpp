/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */

#include <pcl_conversions/pcl_conversions.h>  // pcl::fromROSMsg
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include "common/msgs/autosense_msgs/PointCloud2Array.h"

#include "common/parameter.hpp"              // common::getSegmenterParams
#include "common/publisher.hpp"              // common::publishCloud
#include "common/time.hpp"                   // common::Clock
#include "common/types/type.h"               // PointICloudPtr
#include "segmenters/segmenter_manager.hpp"  // segmenter::createGroundSegmenter

#include "roi_filters/roi.hpp"  // roi::applyROIFilter

const std::string param_ns_prefix_ = "detect";  // NOLINT
std::string frame_id_;                          // NOLINT
bool use_roi_filter_;
autosense::ROIParams params_roi_;
// ROS Subscriber
ros::Subscriber pointcloud_sub_;
// ROS Publisher
ros::Publisher pcs_segmented_pub_;
/// @note Core components
boost::shared_ptr<autosense::segmenter::BaseSegmenter> ground_remover_;
boost::shared_ptr<autosense::segmenter::BaseSegmenter> segmenter_;

void OnPointCloud(const sensor_msgs::PointCloud2ConstPtr &ros_pc2) {
    autosense::common::Clock clock;

    autosense::PointICloudPtr cloud(new autosense::PointICloud);
    pcl::fromROSMsg(*ros_pc2, *cloud);
    ROS_INFO_STREAM(" Cloud inputs: #" << cloud->size() << " Points");

    std_msgs::Header header = ros_pc2->header;
    header.frame_id = frame_id_;
    header.stamp = ros::Time::now();

    if (use_roi_filter_) {
        autosense::roi::applyROIFilter<autosense::PointI>(params_roi_, cloud);
    }

    std::vector<autosense::PointICloudPtr> cloud_clusters;
    autosense::PointICloudPtr cloud_ground(new autosense::PointICloud);
    autosense::PointICloudPtr cloud_nonground(new autosense::PointICloud);

    ground_remover_->segment(*cloud, cloud_clusters);
    *cloud_ground = *cloud_clusters[0];
    *cloud_nonground = *cloud_clusters[1];

    // reset clusters
    cloud_clusters.clear();
    segmenter_->segment(*cloud_nonground, cloud_clusters);
    autosense::common::publishPointCloudArray<autosense::PointICloudPtr>(
        pcs_segmented_pub_, header, cloud_clusters);

    ROS_INFO_STREAM("Cloud processed. Took " << clock.takeRealTime()
                                             << "ms.\n");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "detection_node");

    // Node handle
    ros::NodeHandle nh = ros::NodeHandle();
    ros::NodeHandle private_nh = ros::NodeHandle("~");
    ros::AsyncSpinner spiner(1);

    /// @brief Load ROS parameters from rosparam server
    private_nh.getParam(param_ns_prefix_ + "/frame_id", frame_id_);

    std::string sub_pc_topic, pub_pcs_segmented_topic;
    int sub_pc_queue_size;
    private_nh.getParam(param_ns_prefix_ + "/sub_pc_topic", sub_pc_topic);

    private_nh.getParam(param_ns_prefix_ + "/sub_pc_queue_size",
                        sub_pc_queue_size);
    private_nh.getParam(param_ns_prefix_ + "/pub_pcs_segmented_topic",
                        pub_pcs_segmented_topic);

    /// @note Important to use roi filter for "Ground remover"
    private_nh.param<bool>(param_ns_prefix_ + "/use_roi_filter",
                           use_roi_filter_, false);
    params_roi_ = autosense::common::getRoiParams(private_nh, param_ns_prefix_);

    // Ground remover & non-ground segmenter
    std::string ground_remover_type, non_ground_segmenter_type;
    private_nh.param<std::string>(param_ns_prefix_ + "/ground_remover_type",
                                  ground_remover_type,
                                  "GroundPlaneFittingSegmenter");
    private_nh.param<std::string>(
        param_ns_prefix_ + "/non_ground_segmenter_type",
        non_ground_segmenter_type, "RegionEuclideanSegmenter");
    autosense::SegmenterParams param =
        autosense::common::getSegmenterParams(private_nh, param_ns_prefix_);

    param.segmenter_type = ground_remover_type;
    ground_remover_ = autosense::segmenter::createGroundSegmenter(param);

    param.segmenter_type = non_ground_segmenter_type;
    segmenter_ = autosense::segmenter::createNonGroundSegmenter(param);

    pcs_segmented_pub_ = nh.advertise<autosense_msgs::PointCloud2Array>(
        pub_pcs_segmented_topic, 1);

    pointcloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
        sub_pc_topic, sub_pc_queue_size, OnPointCloud);

    spiner.start();
    ROS_INFO("detection_node started...");

    ros::waitForShutdown();
    ROS_INFO("detection_node exited...");

    return 0;
}
