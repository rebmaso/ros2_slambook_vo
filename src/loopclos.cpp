//
// Created by tommaso
//

#include "myslam/loopclos.h"

namespace myslam {

    // Constructor
    LoopClos::LoopClos() : kf_queue(Config::Get<int>("loop_closure_kf_queue_max_size")), 
                            db(false, 0), 
                            matcher(cv::NORM_HAMMING){

        // load vocabulary
        LOG(INFO) << "Loading vocabulary... ";
        vocab.load(Config::Get<std::string>("vocabulary"));
        if (vocab.empty()) {
            // fatal terminates program
            LOG(FATAL) << "Vocabulary does not exist.";
            throw std::runtime_error("Vocabulary does not exist.");
        }

        // load db with vocab
        db.setVocabulary(vocab);

        // gftt_detector = cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 10);
        orb_descriptor = cv::ORB::create(1000); // get 1000 features as suggested by artal 14

        loopclos_running_.store(true);
        loopclos_thread_ = std::thread(std::bind(&LoopClos::LoopClosLoop, this));
    }

    void LoopClos::Stop() {
        loopclos_running_.store(false);
        loopclos_thread_.join();
    }

    void LoopClos::NewKF(Frame::Ptr kfptr) {
        kf_queue.push_and_drop(kfptr);
    }

    void LoopClos::LoopClosLoop() {

        int img_width = Config::Get<int>("img_width");
        int img_height = Config::Get<int>("img_height");
        int loop_closure_min_inliers = Config::Get<int>("loop_closure_min_inliers");
        float loop_closure_min_coverage = Config::Get<float>("loop_closure_min_coverage");
        float loop_closure_min_similarity = Config::Get<float>("loop_closure_min_similarity");
        float loop_closure_p3p_threshold = Config::Get<float>("loop_closure_p3p_threshold");
    /*
        while (loopclos_running_.load()) {

            // wait for new kf
            Frame::Ptr kf;
            kf_queue.wait_and_pop(kf);

            // This is always zero for now - means loop closure faster than frontend!
            LOG(INFO) << "Queue size: " << kf_queue.size();

            // if empty kf
            if(kf == nullptr) continue;

            // LOG(INFO) << "Loop Closure thread detected new KF. ";
            
            // COMPUTE DBOW DESC FOR NEW KF
            // do not use VO features. these descs are to be used only for dbow
            std::vector<cv::KeyPoint> keypoints_dbow;
            cv::Mat descriptors_dbow;
            orb_descriptor->detectAndCompute(kf->left_img_, cv::Mat(), keypoints_dbow, descriptors_dbow);

            // add to database
            db.add(descriptors_dbow);

            //LOG(INFO) << "Extracted ORB descriptors and added to database. ";
            //LOG(INFO) << db;

            // Get top knn matches
            DBoW3::QueryResults results;
            db.query(descriptors_dbow, results, 4); // top n matches

            //LOG(INFO) << "Matched latest KF against database ";
            //LOG(INFO) << results;

            // LOG(INFO) << all_kfs.size();
            
            // Will put loop closure frames here
            Map::KeyframesType positive_loop_kfs;

            // loop features to remove after optimize
            std::vector<std::pair<Feature::Ptr, Feature::Ptr>> loop_features_to_remove;

            // forr each of the top n resuts
            for(auto & result : results) {

                LOG(INFO) << "CANDIDATE LOOP CLOSURE frames: " << kf->keyframe_id_ <<"  " << result.Id;
                
                // if similarity high enough (tune)
                if(result.Score < loop_closure_min_similarity) {
                    LOG(INFO) << "Similarity not high enough - skipping";
                    continue;
                }

                // if is in active frames or not found at all, skip
                // actually you should also handle short loops.
                auto all_kfs = map_->GetAllKeyFrames();
                auto active_kfs = map_->GetActiveKeyFrames();
                auto loop_kf_it = all_kfs.find(result.Id); // will use this iterator to access 
                if(active_kfs.find(result.Id) != active_kfs.end() || 
                    loop_kf_it == all_kfs.end()) {
                        LOG(INFO) << "Frame still active / not found - skipping";
                        continue;
                    }

                Frame::Ptr loop_kf = loop_kf_it->second;

                // ---- 2D-3D RANSAC geometric verification w PnP

                // get features & landmarks from new kf
                std::vector<cv::KeyPoint> keypoints;
                std::vector<cv::Point3d> landmarks;
                std::vector<std::shared_ptr<Feature>> good_features_left;

                // skip if not enough good features in image
                if(!kf->getKeypointsAndLandmarks(keypoints, landmarks, good_features_left, 30)) {
                    LOG(WARNING) << "Can't get new frame kpts and landmarks - skipping";
                    continue;
                }

                // describe tracked kpts in new kf
                cv::Mat descriptors;
                orb_descriptor->compute(kf->left_img_, keypoints, descriptors);

                // detect & describe loop kf
                std::vector<cv::KeyPoint> keypoints_loop;
                cv::Mat descriptors_loop;
                //gftt_detector->detect(loop_kf->left_img_, keypoints_loop);
                orb_descriptor->detectAndCompute(loop_kf->left_img_, cv::Mat(), keypoints_loop, descriptors_loop);

                // match keypoints
                std::vector<std::vector<cv::DMatch>> matches_raw;
                matcher.knnMatch(descriptors, descriptors_loop, matches_raw, 2); // need vecvec for knn

                // LOG(INFO) << "No. raw 2D-2D matches: " << matches_raw.size();

                // skip if too few matches
                if(matches_raw.size() < loop_closure_min_inliers) {
                    LOG(INFO) << "Not enough raw matches (" << matches_raw.size() << ") - skipping";
                    continue;
                }

                // Lowes test (if distance of second-best match is almost as good, bad!)
                const float lowes_constant = 0.9f;
                std::vector<cv::DMatch> good_matches; // this one is a simple vector (not vecvec) bc we dont need second best match anymore
                for(auto m : matches_raw) {
                    if(m[0].distance < m[1].distance * lowes_constant)
                    good_matches.push_back(m[0]);
                }

                // skip if too few matches
                if(good_matches.size() < 20) {
                    LOG(INFO) << "Not enough matches passing Lowe's (" << good_matches.size() << ")- skipping";
                    continue;
                }

                // Prepare data for pnp
                std::vector<cv::Point2d> src_pts, dst_pts;
                std::vector<cv::Point3d> src_pts_3D;
                for (size_t i = 0; i < good_matches.size(); i++) {
                    src_pts.push_back(keypoints[good_matches[i].queryIdx].pt);
                    dst_pts.push_back(keypoints_loop[good_matches[i].trainIdx].pt);
                    src_pts_3D.push_back(landmarks[good_matches[i].trainIdx]);
                }

                cv::Mat K_cv;
                cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);
                cv::eigen2cv(cam_left_->K(), K_cv);

                cv::Mat rvec, tvec;

                // P3P with RANSAC
                std::vector<int> inliers_pnp;
                solvePnPRansac(src_pts_3D, dst_pts, K_cv, dist_coeffs, rvec, tvec, false, 
                                100, loop_closure_p3p_threshold, 0.99, inliers_pnp, cv::SOLVEPNP_P3P);

                // extract inliers
                std::vector<cv::Point2d> dst_pts_inliers;
                size_t n_inliers_pnp =  inliers_pnp.size();
                for (size_t i = 0; i < n_inliers_pnp; i++) {
                    dst_pts_inliers.push_back(dst_pts[i]);
                }

                // Check number of effective inliers
                // skip if n of effective inliers low
                float coverage_factor = measureCoverageFactor(img_width, img_height, dst_pts_inliers);

                // LOG(INFO) << "LOOP CLOSURE: 2D-3D inliers: " << n_inliers_pnp << " / " << good_matches.size();
                LOG(INFO) << "LOOP CLOSURE: coverage factor: " << coverage_factor;

                if( coverage_factor < loop_closure_min_coverage) continue;

                // add frame to loop closure frames
                positive_loop_kfs.insert(std::make_pair(loop_kf->keyframe_id_,loop_kf));

                // and label landmarks as observed by this kf
                // for each inlier landmark in new kf -> set observed with new loop closure feature
                // take inspo frrom froontend when new featurres and landmark obs are added

                for(int inlier : inliers_pnp) {
                    // create new feature object for inlier keypoint in loop kf
                    auto loop_kp = keypoints_loop[good_matches[inlier].trainIdx];
                    Feature::Ptr loop_feat = std::make_shared<Feature>(loop_kf, loop_kp);
                    Feature::Ptr new_feat = good_features_left[good_matches[inlier].queryIdx];
                    auto mp = new_feat->map_point_.lock();

                    if(mp) mp->AddObservation(loop_feat);
                    else continue;

                    // push to vector to remove observations after optimize
                    loop_features_to_remove.push_back(std::make_pair(loop_feat,new_feat));
                }
            }

            // If no good loop closure frames, skip 
            if(positive_loop_kfs.empty()) continue;

            // trigger windowed optimization with added fixed vertices & edges due to loop closure
            // first, reserve map lock (handle concurrency with backend)
            LOG(INFO) << "LOOP CLOSURE: relocalization!";

            std::unique_lock<std::mutex> lck(map_->optimization_mutex_);
            Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();
            Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();
            Optimize(active_kfs, active_landmarks, positive_loop_kfs);
            // unlock

            LOG(INFO) << "LOOP CLOSURE: done!";

            // remove observations
            for(auto & pair : loop_features_to_remove){
                Feature::Ptr loop_feat = pair.first;
                Feature::Ptr new_feat = pair.second;
                if(!loop_feat || !new_feat ) continue;
                auto mp = new_feat->map_point_.lock();
                if(mp) mp->RemoveObservation(loop_feat);
            }
        }
    */

    }

    // todo: 
    // add vertices for each loop frame and set as fixed (how)
    // then some  landmarks should have observation pointing at loop frame (todo).
    // you should not modify anything as regards edges, if you did that part right.

    // todo: levenberg marquardt hides problem: gauge freedom! Jacoian singular
    // You should always fix oldest keyframe

    void LoopClos::Optimize(Map::KeyframesType &keyframes,
                       Map::LandmarksType &landmarks,
                       Map::KeyframesType &keyframes_loop) const {
    // setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
        LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // define pose vertices, use Keyframe id
    std::map<unsigned long, VertexPose *> vertices; ////// VERTICES POSE
    unsigned long max_kf_id = 0;
    for (auto &keyframe : keyframes) {
        auto kf = keyframe.second;
        VertexPose *vertex_pose = new VertexPose();
        vertex_pose->setId(kf->keyframe_id_);
        vertex_pose->setEstimate(kf->Pose());
        optimizer.addVertex(vertex_pose); ////// ADD VERTEX POSE
        if (kf->keyframe_id_ > max_kf_id) {
            max_kf_id = kf->keyframe_id_;
        }
        vertices.insert({kf->keyframe_id_, vertex_pose});
    }

    // Add loop keyframe vertices
    for (auto &keyframe : keyframes_loop) {
        auto kf = keyframe.second;
        VertexPose *vertex_pose = new VertexPose();
        vertex_pose->setId(kf->keyframe_id_);
        vertex_pose->setEstimate(kf->Pose());
        vertex_pose->setFixed(true); // Set loop closure vertices as fixed (not optimised)
        optimizer.addVertex(vertex_pose); ////// ADD VERTEX POSE
        vertices.insert({kf->keyframe_id_, vertex_pose});
    }

    // define landmark vertices, use landmark id
    std::map<unsigned long, VertexXYZ *> vertices_landmarks; /////// VERTICES LANDMARK

    // K and extrinsics
    Mat33 K = cam_left_->K();
    SE3 left_ext = cam_left_->pose();
    SE3 right_ext = cam_right_->pose();

    // declare edges
    int index = 1;
    double chi2_th = 5.991;  // robust kernel threshold
    std::map<EdgeProjection *, Feature::Ptr> edges_and_features;

    // for each landmark
    for (auto &landmark : landmarks) {
        if (landmark.second->is_outlier_) continue;
        unsigned long landmark_id = landmark.second->id_;
        auto observations = landmark.second->GetObs();
        // for each obs of landmarrk -> get keyfrrame
        for (auto &obs : observations) {
            if (obs.lock() == nullptr) continue;
            auto feat = obs.lock();
            if (feat->is_outlier_ || feat->frame_.lock() == nullptr) continue;
            auto frame = feat->frame_.lock();
            // Declare reprojection edge: one for each observation of each landmark
            EdgeProjection *edge = nullptr; 
            if (feat->is_on_left_image_) {
                edge = new EdgeProjection(K, left_ext);
            } else {
                edge = new EdgeProjection(K, right_ext);
            }

            // add landmark vertex (check if not added in previous iteration)
            if (vertices_landmarks.find(landmark_id) ==
                vertices_landmarks.end()) {
                VertexXYZ *v = new VertexXYZ;
                v->setEstimate(landmark.second->Pos());
                v->setId(landmark_id + max_kf_id + 1);
                v->setMarginalized(true); // landmarks are marginalised (schur trick)
                vertices_landmarks.insert({landmark_id, v});
                optimizer.addVertex(v); /////////// ADD VERTEX LANDMARK
            }

            // Create reprojection edge: one for each observation of each landmark
            // We may have created too many edges. E.g. theres some old loop closure observations,
            // or theres an observation from a new kf which is not yet in vertices.
            // Thats why we check if the kf is contained in vertices. If not, we delete edge.
            if (vertices.find(frame->keyframe_id_) !=
                vertices.end() && 
                vertices_landmarks.find(landmark_id) !=
                vertices_landmarks.end()) {
                    edge->setId(index);
                    edge->setVertex(0, vertices.at(frame->keyframe_id_));    // pose
                    edge->setVertex(1, vertices_landmarks.at(landmark_id));  // landmark
                    edge->setMeasurement(toVec2(feat->position_.pt));
                    edge->setInformation(Mat22::Identity());
                    auto rk = new g2o::RobustKernelHuber();
                    rk->setDelta(chi2_th);
                    edge->setRobustKernel(rk);
                    edges_and_features.insert({edge, feat});
                    optimizer.addEdge(edge);
                    index++;
            }
            else {
                delete edge;
            }

        }
    }

    // do optimization and eliminate the outliers
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    int cnt_outlier = 0, cnt_inlier = 0;
    int iteration = 0;
    while (iteration < 5) {
        cnt_outlier = 0;
        cnt_inlier = 0;
        // determine if we want to adjust the outlier threshold
        for (auto &ef : edges_and_features) {
            if (ef.first->chi2() > chi2_th) {
                cnt_outlier++;
            } else {
                cnt_inlier++;
            }
        }
        double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
        if (inlier_ratio > 0.5) {
            break;
        } else {
            chi2_th *= 2;
            iteration++;
        }
    }

    for (auto &ef : edges_and_features) {
        if (ef.first->chi2() > chi2_th) {
            ef.second->is_outlier_ = true;
            // remove the observation
            auto mp = ef.second->map_point_.lock();
            if(mp) mp->RemoveObservation(ef.second);
        } else {
            ef.second->is_outlier_ = false;
        }
    }

    // LOG(INFO) << "Outlier/Inlier in optimization: " << cnt_outlier << "/"
    //           << cnt_inlier;

    // DONE! -- SET REFINED POSES AND LANDMARKS 
    for (auto &v : vertices) {
        auto it = keyframes.find(v.first);
        // Do not include loop closure vertices (theyre fixed)
        if(it != keyframes.end()) {
            it->second->SetPose(v.second->estimate());
        }
    }
    for (auto &v : vertices_landmarks) {
        landmarks.at(v.first)->SetPos(v.second->estimate());
    }
}

float LoopClos::measureCoverageFactor(int & img_width, int & img_height, std::vector<cv::Point2d> & keypoints) {

    // create n by n grid
    // assign cell size in pixels

    int cell_width = 8;
    const int grid_height = floor(img_height / cell_width);
    const int grid_width = floor(img_width / cell_width);
    const int total_n_of_cells = grid_height * grid_width;

    std::vector<std::vector<bool>> occupancy_grid(grid_height, std::vector<bool>(grid_width, false));;

    // // set all grid cells to false
    // for (int i = 0; i < grid_height; i++){
    //     for (int j = 0; j < grid_width; j++){
    //         occupancy_grid[i][j] = false;
    //     }
    // }

    // set all covered grid cells to true
    for(cv::Point2d & point : keypoints) {
        // mark cell as covered
        int cell_i = floor(point.y / cell_width);
        int cell_j = floor(point.x / cell_width);
        occupancy_grid[cell_i][cell_j] = true;
    }

    // count number of covered cells
    int count_covered_cells = 0;
    for (int i = 0; i < grid_height; i++){
        for (int j = 0; j < grid_width; j++){
            if(occupancy_grid[i][j] == true) {
                count_covered_cells++;
            }
        }
    }

    // return coverage factor
    float coverage_factor = float(count_covered_cells) / float(total_n_of_cells);

    return coverage_factor;
     
}


} // namespace myslam