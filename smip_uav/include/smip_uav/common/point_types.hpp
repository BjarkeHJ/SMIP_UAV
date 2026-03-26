#ifndef POINT_TYPES_HPP_
#define POINT_TYPES_HPP_

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

struct PointXYZ {
    float px, py, pz;
};

struct PointNormal {
    float px, py, pz; // Point position
    float nx, ny, nz; // Point normal
    float w; // Measurement weight
};

struct Frame {
    size_t W{0};
    size_t H{0};
    std::vector<PointNormal> pixels; // size = H * W, row-major

    Frame() = default;
    Frame(const size_t w, const size_t h, const uint64_t ts_ns) : W(w), H(h), ts_ns_(ts_ns), pixels(w * h, invalid_pixel()) {} // initialize frame with size stamp, and invalid entries

    uint64_t ts_ns_{0}; // Timestamp for sensor data (For synch.)
    
    // MAYBE TODO: Include capture pose in frame

    size_t idx(size_t u, size_t v) const { return v * W + u; }
    PointNormal& operator()(size_t u, size_t v) { return pixels[idx(u, v)]; }
    const PointNormal& operator()(size_t u, size_t v) const { return pixels[idx(u, v)]; }
    static bool is_valid(const PointNormal& p) {
        return (std::isfinite(p.px) && std::isfinite(p.py) && std::isfinite(p.pz) && std::isfinite(p.nx) && std::isfinite(p.ny) && std::isfinite(p.nz));
    }

    std::vector<PointNormal> to_points() const {
        std::vector<PointNormal> out;
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

    std::vector<float> depth_image() const {
        std::vector<float> img(pixels.size(), std::numeric_limits<float>::quiet_NaN());
        for (size_t i = 0; i < pixels.size(); ++i) {
            if (!is_valid(pixels[i])) continue;
            img[i] = std::sqrt(pixels[i].px * pixels[i].px
                             + pixels[i].py * pixels[i].py
                             + pixels[i].pz * pixels[i].pz);
        }
        return img;
    }

    std::vector<float> normal_image() const {
        std::vector<float> img(pixels.size() * 3, 0.0f);
        for (size_t i = 0; i < pixels.size(); ++i) {
            if (!is_valid(pixels[i])) continue;
            img[3 * i + 0] = pixels[i].nx;
            img[3 * i + 1] = pixels[i].ny;
            img[3 * i + 2] = pixels[i].nz;
        }
        return img;
    }

    std::vector<float> weight_image() const {
        std::vector<float> img(pixels.size(), 0.0f);
        for (size_t i = 0; i < pixels.size(); ++i) {
            if (!is_valid(pixels[i])) continue;
            img[i] = pixels[i].w;
        }
        return img;
    }

private:
    static PointNormal invalid_pixel() {
        constexpr float nan = std::numeric_limits<float>::quiet_NaN();
        return {nan, nan, nan, 0, 0, 0, 0};
    }
};

#endif