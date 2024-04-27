/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "myslam/frame.h"

namespace myslam {

Frame::Frame(long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right)
        : id_(id), time_stamp_(time_stamp), pose_(pose), left_img_(left), right_img_(right) {}

Frame::Ptr Frame::CreateFrame() {
    static long factory_id = 0;
    Frame::Ptr new_frame = std::make_shared<Frame>();
    new_frame->id_ = factory_id++;
    return new_frame;
}

void Frame::SetKeyFrame() {
    static long keyframe_factory_id = 0;
    is_keyframe_ = true;
    keyframe_id_ = keyframe_factory_id++;
}

bool Frame::getKeypointsAndLandmarks(std::vector<cv::KeyPoint> & keypoints, 
                                    std::vector<cv::Point3d> & landmarks,
                                    std::vector<std::shared_ptr<Feature>> & good_features_left,
                                    const size_t & min_features_count) {

    // just in case, clear vectors
    keypoints.clear();
    landmarks.clear();
    good_features_left.clear();

    // success set to true if at least one good feature
    size_t features_count = 0;

    for(auto & feature : features_left_) {

        if(feature){ // if not nullptr (shouldnt happen)
        
            auto mp = feature->map_point_.lock();
            
            if(mp) // if not nullptr (if landmark is outlier the guy frees ptr)
            {   

                features_count++;

                // push to kpts
                keypoints.push_back(feature->position_);

                // push to landmarks
                auto pos = mp->Pos();
                cv::Point3d pos_cv(pos(0),pos(1),pos(2));
                landmarks.push_back(pos_cv);

                // also save good features pointers
                good_features_left.push_back(feature);
            }

        }
        
    }

    return features_count >= min_features_count;

}

}
