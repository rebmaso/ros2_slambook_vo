//
// Created by gaoxiang on 19-5-2.
//

#ifndef MYSLAM_BACKEND_H
#define MYSLAM_BACKEND_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"
#include "myslam/camera.h"
#include "myslam/algorithm.h"
#include "myslam/feature.h"
#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/mappoint.h"

namespace myslam {
class Map;

/**
 * Backend
 * Has a separate optimization thread, which is started when the Map is updated
 * Map updates are triggered by the frontend
 */
class Backend {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Backend> Ptr;

    /// Constructor: Starts the optimization thread and suspends it
    Backend();

        // Set the left and right cameras for obtaining intrinsic and extrinsic parameters
    void SetCameras(Camera::Ptr left, Camera::Ptr right) {
        cam_left_ = left;
        cam_right_ = right;
    }

        /// Set the map
    void SetMap(std::shared_ptr<Map> map) { map_ = map; }

        /// Trigger map update and start optimization
    void UpdateMap();

        /// Stop the backend thread
    void Stop();

   private:
        /// Backend thread
    void BackendLoop();

    /// Optimize given keyframes and landmarks
    //This is nonlinear refinement starting from triangulation + pnp frontend. 
    //This is the same as in monocular. the only change there is the frontend procedure and subsequent loss of scale, which we arbitrarily fix.
    void Optimize(Map::KeyframesType& keyframes, Map::LandmarksType& landmarks) const;

    std::shared_ptr<Map> map_;
    std::thread backend_thread_;
    std::mutex data_mutex_;

    std::condition_variable map_update_;
    std::atomic<bool> backend_running_;

    Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
};

}  // namespace myslam

#endif  // MYSLAM_BACKEND_H