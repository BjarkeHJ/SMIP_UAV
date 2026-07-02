#ifndef SMIP_BUNDLE_ADJUSTMENT_HPP_
#define SMIP_BUNDLE_ADJUSTMENT_HPP_

#include <cstdint>
#include <unordered_map>
#include <vector>
#include <Eigen/Geometry>

#include "core/types.hpp"
#include "core/frozen_submap.hpp"
#include "mapping/map_state_container.hpp"

namespace smip_uav {

class BundleAdjustment {
public:
    struct Config {
        // Association
        float assoc_voxel_size{1.0f};
        float min_nromal_dot{0.90f};
        float max_plane_offset_m{0.10f};

        // Clustering
        uint32_t min_submaps_per_cluster{2};
        float min_cluster_weight{0.5f};
        float planarity_ratio{6.0f}; //lambda_mid / lambda_min > this
        float min_lateral_extend{0.10f};
        float min_normal_consistency{0.8f};

        // Cost (opt)
        float ba_sigma_m{0.01f};
        float fallback_confidence_ref{5.0f};

        // Odometry chain prior
        // float odom_sigma_rot_rad{0.005f};
        // float odom_sigma_trans_m{0.005f};
        float odom_sigma_rot_rad{0.05f};
        float odom_sigma_trans_m{0.05f};

        // Solver
        int max_outer_loops{2};
        int max_gn_iterations{6};
        double fd_step{1e-5};
        double lm_init{1e-4};
        double min_rel_cost_decrease{1e-6};
        double min_step_norm{1e-7};
        double reassoc_min_pose_change{1e-4};
    };

    struct Stats {
        uint32_t clusters{0};
        uint32_t dropped_few_submaps{0};
        uint32_t dropped_planarity{0};
        uint32_t dropped_weight{0};
        float mean_contribs{0.0f};
        double cost_initial{0.0};
        double cost_final{0.0};
        int iterations{0};
        int outer_loops{0};
        double t_associate_ms{0.0};
        double t_solve_ms{0.0};
    };

    struct Result {
        std::vector<PoseCorrection> corrections;
        Stats stats;
    };

    BundleAdjustment() : BundleAdjustment(Config{}) {}
    explicit BundleAdjustment(const Config& cfg) : cfg_(cfg) {}
    BundleAdjustment(const BundleAdjustment&) = delete;
    BundleAdjustment& operator=(const BundleAdjustment&) = delete;

    bool has(SubmapId id) const { return id_to_cache_.count(id) > 0; }
    size_t num_submaps() const { return cache_.size(); }
    
    // Copy per-surfel moments out of a frozen submap (read_submap)
    void ingest_submap(SubmapId id, const FrozenSubmap& fs);

    Result optimize(const MapSnapshot& snap);

private:
    struct CachedSurfel {
        Eigen::Vector3d mu;
        Eigen::Vector3d n;
        double W;
        Eigen::Vector3d S1;
        Eigen::Matrix3d S2;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    struct CachedSubmap {
        SubmapId id {kInvalidSubmapId};
        Eigen::Isometry3d T_origin{Eigen::Isometry3d::Identity()};
        std::vector<CachedSurfel> surfels;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    
    struct Contribution {
        uint32_t pose_idx;
        double W;
        Eigen::Vector3d S1;
        Eigen::Matrix3d S2;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    struct Cluster {
        std::vector<Contribution> contribs;
    };
    struct TransformedContribution {
        double W;
        Eigen::Vector3d S1;
        Eigen::Matrix3d S2;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    void associate(const std::vector<Eigen::Isometry3d>& poses, Stats& st);
    int lm_solve(std::vector<Eigen::Isometry3d>& poses, Stats& st);
    double total_cost(const std::vector<Eigen::Isometry3d>& poses) const;
    double chain_cost(const std::vector<Eigen::Isometry3d>& poses) const;

    static TransformedContribution transform_contribution(const Contribution& c, const Eigen::Isometry3d& T);
    double cluster_cost(double Wt, const Eigen::Vector3d& S1t, const Eigen::Matrix3d& S2t) const;

    Config cfg_;
    std::vector<CachedSubmap> cache_;
    std::unordered_map<SubmapId, uint32_t> id_to_cache_;
    std::vector<Cluster> clusters_;

};

} // namespace smip_uav

#endif