#pragma once

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include "myslam/camera.h"
#include "myslam/feature.h"
#include "myslam/mappoint.h"
#include "myslam/common_include.h"

namespace myslam {

// forward declare
struct MapPoint;
struct Feature;

struct Frame {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;

    unsigned long id_ = 0;           // id of this frame
    unsigned long keyframe_id_ = 0;  // id of key frame
    bool is_keyframe_ = false;       
    double time_stamp_;              
    SE3 pose_;                       // Tcw 形式Pose
    std::mutex pose_mutex_;          // Pose数据锁
    cv::Mat left_img_, right_img_;   // stereo images

    // extracted features in left image
    std::vector<std::shared_ptr<Feature>> features_left_;
    // corresponding features in right image, set to nullptr if no corresponding
    std::vector<std::shared_ptr<Feature>> features_right_;
    

   public:  // data members
    Frame() {}

    Frame(long id, double time_stamp, const SE3 &pose, const Mat &left,
          const Mat &right);

    // set and get pose, thread safe
    SE3 Pose() {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        return pose_;
    }

    void SetPose(const SE3 &pose) {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_ = pose;
    }

    void SetKeyFrame();

    static std::shared_ptr<Frame> CreateFrame();

    // get inlier keypoints and landmarks, in opencv format
    // also get features to add observation if loop closure check passes
    bool getKeypointsAndLandmarks(std::vector<cv::KeyPoint> & keypoints, 
                                    std::vector<cv::Point3d> & landmarks,
                                    std::vector<std::shared_ptr<Feature>> & good_features_left,
                                    const size_t & min_features_count);
};

}  // namespace myslam

#endif  // MYSLAM_FRAME_H
