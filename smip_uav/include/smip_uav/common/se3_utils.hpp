#ifndef SE3_UTILS_HPP_
#define SE3_UTILS_HPP_

#include <Eigen/Geometry>
#include <Eigen/LU>

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

// Lie log: returns tangent vector xi = [t; omega] such that Exp(xi) * Identity = dT.
// Translation: taken directly from dT (first-order approximation, exact for pure rotation).
// Rotation: angle-axis form of dT.linear().
// Accurate when |dT| is small; used for Kalman corrections after ICP convergence.
inline Eigen::Matrix<float,6,1> log_se3(const Eigen::Isometry3f& dT)
{
    Eigen::Matrix<float,6,1> xi;
    xi.head<3>() = dT.translation();
    const Eigen::AngleAxisf aa(dT.rotation());
    xi.tail<3>() = aa.angle() * aa.axis();
    return xi;
}

// Left SE3 perturbation: returns Exp(xi) where xi = [t; omega].
// Same convention as the ICP solver: T_new = exp_se3(xi) * T_old.
inline Eigen::Isometry3f exp_se3(const Eigen::Matrix<float,6,1>& xi)
{
    Eigen::Isometry3f dT = Eigen::Isometry3f::Identity();
    dT.translation() = xi.head<3>();
    const float angle = xi.tail<3>().norm();
    if (angle > 1e-8f) {
        dT.linear() = Eigen::AngleAxisf(angle, xi.tail<3>() / angle).toRotationMatrix();
    }
    return dT;
}

} // smip_uav

#endif