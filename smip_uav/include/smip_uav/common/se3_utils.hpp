#ifndef SE3_UTILS_HPP_
#define SE3_UTILS_HPP_

#include <Eigen/Geometry>

namespace smip_uav {

inline Eigen::Isometry3f interpolate_se3(
    const Eigen::Isometry3f& A,
    const Eigen::Isometry3f& B,
    float alpha)
{
    alpha = std::clamp(alpha, 0.0f, 1.0f);

    // Rotation: slerp via quaternions (numerically stable, geodesic on SO(3))
    const Eigen::Quaternionf qA(A.rotation());
    Eigen::Quaternionf qB(B.rotation());

    // Force shortest-arc interpolation: q and -q represent the same rotation,
    // but slerp from q to -q would go the long way arount the 4-sphere
    if (qA.dot(qB) < 0.0f) {
        qB.coeffs() *= -1.0f;
    }

    const Eigen::Quaternionf q = qA.slerp(alpha, qB);

    // Translation: linear interpolation in R3
    const Eigen::Vector3f t = (1.0f - alpha) * A.translation() + alpha * B.translation();

    Eigen::Isometry3f out = Eigen::Isometry3f::Identity();
    out.linear() = q.toRotationMatrix();
    out.translation() = t;
    return out;
}

} // smip_uav

#endif