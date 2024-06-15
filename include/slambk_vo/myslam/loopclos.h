/*
This is the object that takes care of detecting loop closures via orb descriptors & dbow2.
Once a keyframe is removed from the kf graph, it is added to the static loop clos graph.
that is just a static database of old keyframes.

At loop closure, we perforrm same optimization with gto, but add  extra (fixed) loop cloosure poses vertices)
Only closures with olf kfs (in the pose database) are meaningful.

This does not align the whole map! Its just visual odometry with relocalization . not proper slam!
That is the next step. This relkocalization procedure is similar to that in VINS-mono.

We use the guys map infrastructure. problem: it stores image too. expensive! we need it if we dont store orb descs!
*/

//
// Created by tommaso
//

#ifndef LOOPCLOSURE_H
#define LOOPCLOSURE_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"
#include "myslam/camera.h"
#include "myslam/config.h"
#include "myslam/threadsafe_queue.h"
#include "myslam/g2o_types.h"
#include "myslam/algorithm.h"

#include "DBoW3/DBoW3.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <stdexcept>

namespace myslam {

class LoopClos {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<LoopClos> Ptr;

    /// Constructor: Starts the loop closing thread
    LoopClos();

    /// Set the map
    void SetMap(std::shared_ptr<Map> map) { map_ = map; }

    /// Stop the loop closing thread
    void Stop();

    // called by frontend if new kf. triggers loop detection, unlocking loop
    void NewKF(Frame::Ptr kfptr);

    // Set the left and right cameras for obtaining intrinsic and extrinsic parameters
    void SetCameras(Camera::Ptr left, Camera::Ptr right) {
        cam_left_ = left;
        cam_right_ = right;
    }

   private:
    /// loop closing thread
    void LoopClosLoop();

    void Optimize(Map::KeyframesType& keyframes, 
                    Map::LandmarksType& landmarks, 
                    Map::KeyframesType &keyframes_loop) const;

    float measureCoverageFactor(int & img_width, 
                                int & img_height, 
                                std::vector<cv::Point2d> & keypoints);

    // threadsafe queue for new keyframe loop closure processing
    // frame pointers could be freed. always check when accessing!
    threadsafe_queue<Frame::Ptr> kf_queue;

    // dbow vocab. will load in constructor
    DBoW3::Vocabulary vocab;

    // dbow database 
    DBoW3::Database db;

    // ORB detector
    std::shared_ptr<cv::ORB> orb_descriptor;

    // gftt detector
    // cv::Ptr<cv::GFTTDetector> gftt_detector;

    // matcher
    cv::BFMatcher matcher;

    std::shared_ptr<Map> map_;
    std::thread loopclos_thread_;

    std::atomic<bool> loopclos_running_;

    Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
};

}  // namespace myslam

#endif  // LOOPCLOSURE_H