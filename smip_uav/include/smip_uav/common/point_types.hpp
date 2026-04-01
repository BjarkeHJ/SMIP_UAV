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
    FramePixel(const Eigen::Vector3f& vp, const Eigen::Vector3f& vn, float d, float w) : pos3d(vp), nrm3d(vn), depth(d), weight(w) {}
    Eigen::Vector3f pos3d;
    Eigen::Vector3f nrm3d;
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
    Frame(const size_t w, const size_t h, const int64_t stamp_ns) : W(w), H(h), pixels(w * h, invalid_pixel()), timestamp(stamp_ns) {} // initialize frame with size stamp, and invalid entries

    Eigen::Isometry3f tf_pose{Eigen::Isometry3f::Identity()}; // Frame World-Sensor transform (odometry state estimate)
    int64_t timestamp{0}; // Timestamp for sensor data
    uint64_t frame_id{INVALID_FRAME_ID}; // Frame/Scan identifier

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

    // USED FOR VIZ ONLY
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

private:
    static FramePixel invalid_pixel() {
        constexpr float nan = std::numeric_limits<float>::quiet_NaN();
        return {Eigen::Vector3f::Constant(nan), Eigen::Vector3f::Zero(), nan, nan};
    }
};

struct Surfel {
    float W{0.0f}; // Sum of weights
    Eigen::Vector3f S1{Eigen::Vector3f::Zero()}; // sum of products of weighted pointxyz
    Eigen::Matrix3f S2{Eigen::Matrix3f::Zero()}; // sum of weigthed outer products of pointxyz (uncentered)

    uint32_t sid{0};

    void fuse(const Surfel& other) {
        W += other.W;
        S1 += other.S1;
        S2 += other.S2;
        dirty_ = true;
        return;
    }

    // accessors: will recompute if needed
    Eigen::Vector3f centroid() const { recompute(); return centroid_; }
    Eigen::Vector3f normal() const { recompute(); return normal_; }
    Eigen::Matrix3f covariance() const { recompute(); return cov_; }
    Eigen::Vector3f eigenvalues() const { recompute(); return evals_; }
    Eigen::Matrix3f eigenvectors() const { recompute(); return evecs_; }
    float radius() const { recompute(); return radius_; }
    bool valid() const { recompute(); return valid_; }

    private:
        mutable bool dirty_{true};
        mutable Eigen::Vector3f centroid_, normal_;
        mutable Eigen::Matrix3f cov_;
        mutable Eigen::Vector3f evals_;
        mutable Eigen::Matrix3f evecs_;
        mutable float radius_;
        mutable bool valid_;

        void recompute() const {
            if (!dirty_) return;
            valid_ = false;
            dirty_ = false;

            if (W < 1e-8f) return;

            centroid_ = S1 / W;
            if (!centroid_.allFinite()) return;

            cov_ = S2 / W - centroid_ * centroid_.transpose();

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov_);
            if (eig.info() != Eigen::Success) return;

            evals_ = eig.eigenvalues().cwiseMax(0.0f);
            evecs_ = eig.eigenvectors();

            if (evals_(1) < 1e-8f) return;  // rank-deficient, < 2D spread
            const float planarity = evals_(0) / evals_(1);
            if (planarity > 0.3f) return;   // too thick to be a surface patch

            normal_ = evecs_.col(0);
            if (normal_.dot(centroid_) > 0.0f) {
                normal_ = -normal_;
            }

            radius_ = std::sqrt(evals_(1) + evals_(2));

            valid_ = true;
            dirty_ = false;
        }
};

#endif