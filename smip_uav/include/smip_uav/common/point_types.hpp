#ifndef POINT_TYPES_HPP_
#define POINT_TYPES_HPP_

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

struct PointXYZ {
    float px, py, pz;
};

struct FramePixel {
    FramePixel() = default;
    FramePixel(const Eigen::Vector3f& vp, const Eigen::Vector3f& vn, float d, float w) : pos3d(vp), nrm3d(vn), depth(d), weight(w) {}
    Eigen::Vector3f pos3d{Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN())};
    Eigen::Vector3f nrm3d{Eigen::Vector3f::Zero()};
    float depth{std::numeric_limits<float>::infinity()};
    float weight{0.0f};
    bool valid{false};
};

struct Frame {
    static constexpr uint64_t INVALID_FRAME_ID = 0; 

    size_t W{0};
    size_t H{0};
    std::vector<FramePixel> pixels; // size = H * W, row-major

    Frame() = default;
    // construct frame with size stamp and assign sizes to edge map
    Frame(const size_t w, const size_t h, const int64_t stamp_ns) : W(w), H(h), pixels(w * h), timestamp(stamp_ns) {
        edge_h.assign(W * H, 0); 
        edge_v.assign(W * H, 0);
    }

    Eigen::Isometry3f tf_pose{Eigen::Isometry3f::Identity()}; // Frame World-Sensor transform (odometry state estimate)
    int64_t timestamp{0}; // Timestamp for sensor data
    uint64_t frame_id{INVALID_FRAME_ID}; // Frame/Scan identifier

    // Edge map
    std::vector<uint8_t> edge_h;
    std::vector<uint8_t> edge_v;

    size_t idx(size_t u, size_t v) const { return v * W + u; }
    FramePixel& operator()(size_t u, size_t v) { return pixels[idx(u, v)]; }
    const FramePixel& operator()(size_t u, size_t v) const { return pixels[idx(u, v)]; }

    static bool is_valid(const FramePixel& p) {
        return p.valid && p.pos3d.allFinite() && p.nrm3d.squaredNorm() > 0.0f;
    }

    std::vector<FramePixel> to_points() const {
        std::vector<FramePixel> out;
        for (const auto& p : pixels) {
            if (is_valid(p)) {
                out.push_back(p);
            }
        }
        return out;
    }

    size_t valid_count() const {
        return std::count_if(pixels.begin(), pixels.end(), is_valid);
    }

    // === USED FOR VIZ ONLY === 
    std::vector<float> depth_image() const {
        std::vector<float> img(pixels.size(), std::numeric_limits<float>::quiet_NaN());
        for (size_t i = 0; i < pixels.size(); ++i) {
            if (!is_valid(pixels[i])) continue;
            img[i] = pixels[i].pos3d.squaredNorm();
        }
        return img;
    }
    std::vector<float> normal_image() const {
        std::vector<float> img(pixels.size() * 3, 0.0f);
        for (size_t i = 0; i < pixels.size(); ++i) {
            if (!is_valid(pixels[i])) continue;
            img[3 * i + 0] = pixels[i].nrm3d.x();
            img[3 * i + 1] = pixels[i].nrm3d.y();
            img[3 * i + 2] = pixels[i].nrm3d.z();
        }
        return img;
    }
    std::vector<float> weight_image() const {
        std::vector<float> img(pixels.size(), 0.0f);
        for (size_t i = 0; i < pixels.size(); ++i) {
            if (!is_valid(pixels[i])) continue;
            img[i] = pixels[i].weight;
        }
        return img;
    }
};

struct FrameSurfel {
    uint32_t sid{0};
    Eigen::Vector3f centroid;
    Eigen::Vector3f normal;
    Eigen::Matrix3f R; // Measurement uncertainty model
    Eigen::Vector3f eigenvalues; // Geometrical Covariance Eigenvalues
    Eigen::Matrix3f eigenvectors; //                       Eigenvectors
    Eigen::Matrix3f C_shape;     // Geometric covariance
    float weight;
    float view_cos_theta{90.0f};
};

struct MapSurfel {
    uint32_t id;
    Eigen::Vector3f mu; // position mean
    Eigen::Matrix3f P; // surfel centroid uncertainty covariance

    Eigen::Matrix3f C_shape; // geometric shape around mu
    float W_shape{0.0f}; // accumulated weight

    uint32_t obs_count{0};
    int64_t last_seen{0};

    Eigen::Vector3f normal{Eigen::Vector3f::Zero()}; // cached normal, updated on fuse/create

    bool is_mature(uint32_t min_obs) const { return obs_count >= min_obs; }

    float planarity() const {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(C_shape);
        const auto& ev = eig.eigenvalues(); // sorted ascending: λ0 ≤ λ1 ≤ λ2

        const float denom = ev(2) + 1e-8f; // normalize by largest eigenvalue
        return (ev(1) - ev(0)) / denom;    // in [0, 1]: 1 = perfectly planar
    }

    float convergence() const {
        const float trace = P.trace() + 1e-8f;
        return 1.0f - P.eigenvalues().real().minCoeff() / trace;
    }
};


#endif