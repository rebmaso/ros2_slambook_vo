#pragma once
#ifndef MYSLAM_FRONTEND_H
#define MYSLAM_FRONTEND_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"
#include "myslam/camera.h"
#include "myslam/algorithm.h"
#include "myslam/backend.h"
#include "myslam/loopclos.h"
#include "myslam/config.h"
#include "myslam/feature.h"
#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/viewer.h"

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

namespace myslam {

class Backend;
class Viewer;

enum class FrontendStatus { INITING, TRACKING_GOOD, TRACKING_BAD, LOST };

/**
 * Frontend
 * Estimates the pose of the current frame and adds keyframes to the map and triggers optimization when keyframe conditions are met.
 */
class Frontend {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frontend> Ptr;

    Frontend();

    /// External interface, adds a frame and computes its pose estimation result
    bool AddFrame(Frame::Ptr frame);

    /// Set functions
    void SetMap(Map::Ptr map) { map_ = map; }

    void SetBackend(std::shared_ptr<Backend> backend) { backend_ = backend; }

    void SetLoopClos(std::shared_ptr<LoopClos> loopclos) { loopclos_ = loopclos; }

    void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }

    FrontendStatus GetStatus() const { return status_; }

    void SetCameras(Camera::Ptr left, Camera::Ptr right) {
        camera_left_ = left;
        camera_right_ = right;
    }

   private:
    /**
     * Track in normal mode
     * @return true if success
     */
    bool Track();

    /**
     * Reset when lost
     * @return true if success
     */
    bool Reset();

    /**
     * Track with last frame
     * @return num of tracked points
     */
    int TrackLastFrame();

    /**
     * estimate current frame's pose with pnp
     * @return num of inliers
     */
    int EstimateCurrentPose();

    /**
     * set current frame as a keyframe and insert it into backend
     * @return true if success
     */
    bool InsertKeyframe();

    /**
     * Try init the frontend with stereo images saved in current_frame_
     * @return true if success
     */
    bool StereoInit();

    /**
     * Detect features in left image in current_frame_
     * keypoints will be saved in current_frame_
     * @return
     */
    int DetectFeatures();

    /**
     * Find the corresponding features in right image of current_frame_
     * @return num of features found
     */
    int FindFeaturesInRight();

    /**
     * Build the initial map with single image
     * @return true if succeed
     */
    bool BuildInitMap();

    /**
     * Triangulate the 2D points in current frame
     * @return num of triangulated points
     */
    int TriangulateNewPoints();

    /**
     * Set the features in keyframe as new observation of the map points
     */
    void SetObservationsForKeyFrame();

    // data
    

    FrontendStatus status_ = FrontendStatus::INITING;

    Frame::Ptr current_frame_ = nullptr; 
    Frame::Ptr last_frame_ = nullptr;   
    Camera::Ptr camera_left_ = nullptr;   
    Camera::Ptr camera_right_ = nullptr;  

    Map::Ptr map_ = nullptr;
    std::shared_ptr<Backend> backend_ = nullptr;
    std::shared_ptr<LoopClos> loopclos_ = nullptr;
    std::shared_ptr<Viewer> viewer_ = nullptr;

    SE3 relative_motion_;  // Relative motion between the current frame and the previous frame, used for initial pose estimation

    int tracking_inliers_ = 0;  // inliers, used for testing new keyframes

    // params
    int num_features_ = 200;
    int num_features_init_ = 100;
    int num_features_tracking_ = 50;
    int num_features_tracking_bad_ = 20;
    int num_features_needed_for_keyframe_ = 80;

    // utilities
    cv::Ptr<cv::GFTTDetector> gftt_;  // feature detector in opencv
};

}  // namespace myslam

#endif  // MYSLAM_FRONTEND_H
