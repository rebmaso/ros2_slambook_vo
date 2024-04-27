#pragma once
#ifndef MYSLAM_VISUAL_ODOMETRY_H
#define MYSLAM_VISUAL_ODOMETRY_H

#include "myslam/backend.h"
#include "myslam/common_include.h"
#include "myslam/dataset.h"
#include "myslam/frontend.h"
#include "myslam/loopclos.h"
#include "myslam/viewer.h"

namespace myslam {

/**
 * VO
 */
class VisualOdometry {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<VisualOdometry> Ptr;

    /// constructor with config file
    VisualOdometry(std::string &config_path);

    /**
     * do initialization things before run
     * @return true if success
     */
    bool Init();

    // to use in a ros callback
    bool Step(Frame::Ptr &  new_frame);

    FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus(); }

    // LoopClosStatus GetLoopClosStatus() const { return loopclos_->GetStatus(); }

   private:
    bool inited_ = false;
    std::string config_file_path_;
    
    Frontend::Ptr frontend_ = nullptr;
    Backend::Ptr backend_ = nullptr;
    LoopClos::Ptr loopclos_ = nullptr;
    Map::Ptr map_ = nullptr;
    Viewer::Ptr viewer_ = nullptr;

    // dataset
    Dataset::Ptr dataset_ = nullptr;
};
}  // namespace myslam

#endif  // MYSLAM_VISUAL_ODOMETRY_H
