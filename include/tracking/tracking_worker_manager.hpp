#ifndef TRACKING_WORKER_MANAGER_HPP_
#define TRACKING_WORKER_MANAGER_HPP_

#include "base_tracking_worker.h"
#include "hm_tracking_worker.hpp"

namespace tracking {

    static std::unique_ptr<BaseTrackingWorker> createTrackingWorker(TrackingWorkerParams params)
    {
        std::unique_ptr<BaseTrackingWorker> tracking_worker;
        tracking_worker = std::unique_ptr<BaseTrackingWorker>(new HmTrackingWorker(params));
        return tracking_worker;
    }
}

#endif  /* TRACKING_WORKER_MANAGER_HPP_ */