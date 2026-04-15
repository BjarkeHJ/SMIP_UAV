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
    uint32_t id{0};

    // Suffcient Statistics (Accumulated)
    float W{0.0f}; // sum(w_k*r_jk)
    Eigen::Vector3f S1{Eigen::Vector3f::Zero()};
    Eigen::Matrix3f S2{Eigen::Matrix3f::Zero()};

    // Reconstructed quantities from  stats
    Eigen::Vector3f mu{Eigen::Vector3f::Zero()};
    Eigen::Matrix3f sigma{Eigen::Matrix3f::Identity() * 1e-3f};
    Eigen::Vector3f normal{Eigen::Vector3f::Zero()};

    // Lifecycle
    uint32_t obs_count{0}; // incremented once per frame
    int64_t last_seen{0};

    // Competitive health
    float health{1.0f}; // EMA of per-frame capture ratio
    uint32_t n_eval{0}; // number of frames health was evaluated

    // Convergence
    bool converged{false};

    // recompute mu, sigma, normal - return false if W is too small (degenerate)
    bool reconstruct() {
        if (W < 1e-8f) return false;

        mu = S1 / W;
        sigma = S2 / W - mu * mu.transpose();
        sigma = 0.5 * (sigma + sigma.transpose()); // force symmetric 

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(sigma);
        if (eig.info() != Eigen::Success) return false;

        Eigen::Vector3f ev = eig.eigenvalues().cwiseMax(1e-6f);
        const Eigen::Matrix3f V = eig.eigenvectors();
        sigma = V * ev.asDiagonal() * V.transpose();

        Eigen::Vector3f n_new = V.col(0);
        if (normal.squaredNorm() > 0.5f && n_new.dot(normal) < 0.0f) {
            n_new = -n_new;
        }
        normal = n_new;
        return true;
    }

};


#endif