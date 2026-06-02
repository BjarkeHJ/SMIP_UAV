#ifndef FRAME_TRANSFORMS_HPP_
#define FRAME_TRANSFORMS_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Geometry>

namespace frame_transform {

inline Eigen::Vector3f TF_WORLD_NED_ENU(const Eigen::Vector3f& ned_pos) {
    Eigen::Vector3f enu_pos;
    enu_pos.x() = ned_pos.y();
    enu_pos.y() = ned_pos.x();
    enu_pos.z() = -ned_pos.z();
    return enu_pos;
}

inline Eigen::Quaternionf TF_BODY_FRD_FLU(const Eigen::Quaternionf& frd_q) {
    static const Eigen::Quaternionf q_x180(0.0f, 1.0f, 0.0f, 0.0f);
    static const Eigen::Quaternionf q_ned_enu(0.0f, M_SQRT1_2, M_SQRT1_2, 0.0f);
    const Eigen::Quaternionf flu_q = (q_ned_enu * frd_q * q_x180).normalized();
    return flu_q;
}


} // namespace frame_transform




#endif 