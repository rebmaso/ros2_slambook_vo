//
// Created by gaoxiang on 19-5-4.
//

#ifndef MYSLAM_ALGORITHM_H
#define MYSLAM_ALGORITHM_H

// algorithms used in myslam
#include "myslam/common_include.h"

namespace myslam {

/**
 * linear triangulation with SVD
 * @param poses     poses,
 * @param points    points in normalized plane
 * @param pt_world  triangulated point in the world
 * @return true if success
 */
// See http://cmp.felk.cvut.cz/cmp/courses/TDV/2012W/lectures/tdv-2012-07-anot.pdf
inline bool triangulation(const std::vector<SE3> &poses,
                   const std::vector<Vec3> points, Vec3 &pt_world) {

    // Solve total Linear least squares (Ax = 0)
    // Solution: x = lowest eigenvalues of A'A
    MatXX A(2 * poses.size(), 4);
    VecX b(2 * poses.size()); // b= 0
    b.setZero();

    // For each pose (left & right in our case)
    for (size_t i = 0; i < poses.size(); ++i) {
        Mat34 m = poses[i].matrix3x4();
        A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
        A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
    }
    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    // Sol is last col of V matrix (eigenvalues), div by last el. (hom coords)
    pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
        // Check if sol well conditioned or not
        return true;
    }
    return false;
}

// converters
inline Vec2 toVec2(const cv::Point2f p) { return Vec2(p.x, p.y); }

}  // namespace myslam

#endif  // MYSLAM_ALGORITHM_H
