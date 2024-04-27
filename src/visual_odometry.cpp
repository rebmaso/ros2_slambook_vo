//
// Created by gaoxiang on 19-5-4.
//
#include "myslam/visual_odometry.h"
#include <chrono>
#include "myslam/config.h"

namespace myslam {

VisualOdometry::VisualOdometry(std::string &config_path)
    : config_file_path_(config_path){}

// uses dataset class
bool VisualOdometry::Init() {


    // read from config file
    if (Config::SetParameterFile(config_file_path_) == false) {
        return false;
    }

    dataset_ =
        Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
    CHECK_EQ(dataset_->Init(), true);

    // create components
    frontend_ = Frontend::Ptr(new Frontend);
    backend_ = Backend::Ptr(new Backend);
    loopclos_ = LoopClos::Ptr(new LoopClos);
    map_ = Map::Ptr(new Map);
    // viewer_ = Viewer::Ptr(new Viewer);
    
    // Link frontend interfaces & pass camera params
    frontend_->SetBackend(backend_); // to inform backend of new kf
    frontend_->SetLoopClos(loopclos_); // to inform loopclos of new kf
    frontend_->SetMap(map_);
    // frontend_->SetViewer(viewer_);
    frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    // Link loopclos interfaces & pass camera params
    loopclos_->SetMap(map_);
    loopclos_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    // Link backend interfaces & pass camera params
    backend_->SetMap(map_);
    backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    // Link viewer interfaces
    // viewer_->SetMap(map_);

    return true;
}

bool VisualOdometry::Step(Frame::Ptr & new_frame) {

    if (new_frame == nullptr) {
        backend_->Stop();
        loopclos_->Stop();
        viewer_->Close();
        LOG(INFO) << "VO exit";
        return false;
    }

    auto t1 = std::chrono::steady_clock::now();
    bool success = frontend_->AddFrame(new_frame);
    auto t2 = std::chrono::steady_clock::now();
    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

    LOG(INFO) << "VO step cost time: " << time_used.count() << " seconds.";
    
    return success;
}

}  // namespace myslam
