/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#ifndef TRACKING_INCLUDE_TRACKING_TRACKING_WORKER_MANAGER_HPP_
#define TRACKING_INCLUDE_TRACKING_TRACKING_WORKER_MANAGER_HPP_

#include <memory>

#include "tracking/base_tracking_worker.h"
#include "tracking/hm_tracking_worker.hpp"

namespace autosense {
namespace tracking {

static std::unique_ptr<BaseTrackingWorker> createTrackingWorker(
    const TrackingWorkerParams &params) {
    std::unique_ptr<BaseTrackingWorker> tracking_worker;
    tracking_worker =
        std::unique_ptr<BaseTrackingWorker>(new HmTrackingWorker(params));
    return tracking_worker;
}

}  // namespace tracking
}  // namespace autosense

#endif  // TRACKING_INCLUDE_TRACKING_TRACKING_WORKER_MANAGER_HPP_
