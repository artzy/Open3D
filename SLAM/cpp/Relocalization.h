// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2024 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <numeric>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "open3d/Open3D.h"
#include "open3d/t/pipelines/registration/Feature.h"

namespace open3d {
namespace examples {
namespace relocalization {

using namespace open3d;

struct RelocalizationConfig {
    bool enabled = true;
    int keyframe_interval_frames = 30;
    double keyframe_min_translation = 0.15;
    double keyframe_min_rotation_deg = 15.0;
    int max_keyframes = 48;
    int max_points_per_keyframe = 8000;
    int rgbd_stride = 2;
    float downsample_voxel = 0.06f;
    int max_candidates = 5;
    double candidate_radius_m = 2.5;
    int retry_interval_frames = 15;
    std::string global_method = "ransac";
    int ransac_max_iter = 100000;
    double ransac_confidence = 0.999;
    double min_fitness = 0.25;
    double min_information_ratio = 0.20;
    double max_pose_jump_m = 1.0;
    int verify_strong_streak = 3;
    int max_hypotheses = 3;
    bool show_keyframe_markers = false;
};

inline RelocalizationConfig RelocalizationConfigForProfile(
        const std::string& profile) {
    RelocalizationConfig config;
    if (profile == "low") {
        config.downsample_voxel = 0.08f;
    } else if (profile == "high") {
        config.downsample_voxel = 0.04f;
    }
    return config;
}

struct RelocalizationAttempt {
    bool accepted = false;
    int keyframe_id = -1;
    core::Tensor T_live_to_world;
    double global_fitness = 0.0;
    double icp_fitness = 0.0;
    double information_ratio = 0.0;
    std::string reject_reason;
};

struct KeyframeEntry {
    int id = -1;
    int frame_id = 0;
    core::Tensor T_world;
    t::geometry::PointCloud pcd_world;
    pipelines::registration::Feature fpfh_legacy;
    geometry::AxisAlignedBoundingBox bounds;
    Eigen::Vector3d capture_position = Eigen::Vector3d::Zero();
    std::vector<double> depth_histogram;
};

inline Eigen::Vector3d PoseTranslation(const core::Tensor& T) {
    Eigen::Matrix4d mat = core::eigen_converter::TensorToEigenMatrixXd(T);
    return mat.block<3, 1>(0, 3);
}

inline double PoseRotationAngleDeg(const core::Tensor& T) {
    Eigen::Matrix4d mat = core::eigen_converter::TensorToEigenMatrixXd(T);
    const double trace = mat.block<3, 3>(0, 0).trace();
    const double cos_angle = std::max(-1.0, std::min(1.0, (trace - 1.0) * 0.5));
    return std::acos(cos_angle) * 180.0 / 3.14159265358979323846;
}

inline double TranslationDistance(const core::Tensor& a, const core::Tensor& b) {
    return (PoseTranslation(a) - PoseTranslation(b)).norm();
}

inline std::vector<double> ComputeDepthHistogram(
        const t::geometry::RGBDImage& rgbd,
        float depth_scale,
        float depth_max,
        int num_bins = 64) {
    std::vector<double> histogram(num_bins, 0.0);
    if (rgbd.depth_.IsEmpty()) {
        return histogram;
    }
    t::geometry::Image depth = rgbd.depth_.To(core::Device("CPU:0"));
    core::Tensor depth_tensor = depth.AsTensor().Contiguous();
    const int64_t count = depth_tensor.NumElements();
    auto accumulate = [&](double raw_depth) {
        const double meters = raw_depth / depth_scale;
        if (meters <= 0.0 || meters > depth_max) {
            return;
        }
        const int bin = std::min(
                num_bins - 1,
                static_cast<int>(meters / depth_max * num_bins));
        histogram[bin] += 1.0;
    };
    if (depth_tensor.GetDtype() == core::UInt16) {
        const uint16_t* data_u16 = depth_tensor.GetDataPtr<uint16_t>();
        for (int64_t i = 0; i < count; ++i) {
            accumulate(static_cast<double>(data_u16[i]));
        }
    } else if (depth_tensor.GetDtype() == core::Float32) {
        const float* data_f32 = depth_tensor.GetDataPtr<float>();
        for (int64_t i = 0; i < count; ++i) {
            accumulate(static_cast<double>(data_f32[i]));
        }
    } else {
        core::Tensor depth_f64 = depth_tensor.To(core::Dtype::Float64);
        const double* data = depth_f64.GetDataPtr<double>();
        for (int64_t i = 0; i < count; ++i) {
            accumulate(data[i]);
        }
    }
    const double total =
            std::accumulate(histogram.begin(), histogram.end(), 0.0);
    if (total > 0.0) {
        for (double& value : histogram) {
            value /= total;
        }
    }
    return histogram;
}

inline double HistogramL1Distance(const std::vector<double>& a,
                                const std::vector<double>& b) {
    const size_t n = std::min(a.size(), b.size());
    double distance = 0.0;
    for (size_t i = 0; i < n; ++i) {
        distance += std::abs(a[i] - b[i]);
    }
    return distance;
}

inline t::geometry::PointCloud PreprocessLivePointCloud(
        const t::geometry::RGBDImage& rgbd,
        const core::Tensor& intrinsic,
        float depth_scale,
        float depth_max,
        const core::Device& device,
        const RelocalizationConfig& config) {
    t::geometry::RGBDImage rgbd_device = rgbd.To(device);
    t::geometry::PointCloud pcd =
            t::geometry::PointCloud::CreateFromRGBDImage(
                    rgbd_device, intrinsic, core::Tensor::Eye(
                                                     4, core::Float64,
                                                     core::Device("CPU:0")),
                    depth_scale, depth_max, config.rgbd_stride, false);
    if (!pcd.HasPointPositions()) {
        return pcd;
    }
    auto filtered = pcd.RemoveNonFinitePoints();
    pcd = std::get<0>(filtered);
    if (!pcd.HasPointPositions()) {
        return pcd;
    }
    pcd = pcd.VoxelDownSample(static_cast<double>(config.downsample_voxel));
    if (!pcd.HasPointPositions()) {
        return pcd;
    }
    pcd.EstimateNormals(30, config.downsample_voxel * 2.0);
    return pcd;
}

inline pipelines::registration::Feature TensorFPFHToLegacyFeature(
        const core::Tensor& fpfh_tensor) {
    pipelines::registration::Feature feature;
    core::Tensor fpfh_cpu =
            fpfh_tensor.To(core::Device("CPU:0"))
                    .To(core::Dtype::Float64)
                    .Contiguous();
    const int64_t num_points = fpfh_cpu.GetShape()[0];
    const int64_t dim = fpfh_cpu.GetShape()[1];
    Eigen::MatrixXd data(dim, num_points);
    const double* ptr = fpfh_cpu.GetDataPtr<double>();
    for (int64_t i = 0; i < num_points; ++i) {
        for (int64_t j = 0; j < dim; ++j) {
            data(j, i) = ptr[i * dim + j];
        }
    }
    feature.data_ = data;
    return feature;
}

inline bool BuildKeyframePointCloudAndFeatures(
        const t::geometry::RGBDImage& rgbd,
        const core::Tensor& intrinsic,
        const core::Tensor& T_world,
        float depth_scale,
        float depth_max,
        const core::Device& device,
        const RelocalizationConfig& config,
        t::geometry::PointCloud& pcd_world_out,
        pipelines::registration::Feature& fpfh_legacy_out) {
    t::geometry::PointCloud pcd_camera =
            PreprocessLivePointCloud(rgbd, intrinsic, depth_scale, depth_max,
                                     device, config);
    if (!pcd_camera.HasPointPositions()) {
        return false;
    }
    const int64_t count = pcd_camera.GetPointPositions().GetLength();
    if (count > config.max_points_per_keyframe) {
        const double ratio = static_cast<double>(config.max_points_per_keyframe) /
                             static_cast<double>(count);
        pcd_camera = pcd_camera.RandomDownSample(ratio);
    }
    if (!pcd_camera.HasPointPositions()) {
        return false;
    }

    core::Tensor fpfh = t::pipelines::registration::ComputeFPFHFeature(
            pcd_camera, 100, config.downsample_voxel * 5.0);
    if (fpfh.NumElements() == 0) {
        return false;
    }

    pcd_world_out = pcd_camera.Transform(T_world);
    fpfh_legacy_out = TensorFPFHToLegacyFeature(fpfh);
    return pcd_world_out.HasPointPositions() && fpfh_legacy_out.Num() > 0;
}

class KeyframeDatabase {
public:
    explicit KeyframeDatabase(RelocalizationConfig config)
        : config_(std::move(config)) {}

    int Size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return static_cast<int>(entries_.size());
    }

    int Capacity() const { return config_.max_keyframes; }

    void Clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        entries_.clear();
        next_id_ = 0;
        last_added_frame_id_ = -1;
        last_added_pose_.reset();
    }

    const KeyframeEntry* GetEntry(int id) const {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& entry : entries_) {
            if (entry.id == id) {
                return &entry;
            }
        }
        return nullptr;
    }

    std::vector<KeyframeEntry> SnapshotEntries() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return entries_;
    }

    bool MaybeAddKeyframe(const t::geometry::RGBDImage& rgbd,
                          const core::Tensor& intrinsic,
                          const core::Tensor& T_world,
                          int frame_id,
                          float depth_scale,
                          float depth_max,
                          const core::Device& device) {
        if (!config_.enabled) {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (last_added_frame_id_ >= 0 &&
            frame_id - last_added_frame_id_ < config_.keyframe_interval_frames) {
            return false;
        }
        if (last_added_pose_) {
            const double translation =
                    TranslationDistance(T_world, *last_added_pose_);
            const core::Tensor relative =
                    last_added_pose_->Inverse().Matmul(T_world);
            const double rotation = PoseRotationAngleDeg(relative);
            if (translation < config_.keyframe_min_translation &&
                rotation < config_.keyframe_min_rotation_deg) {
                return false;
            }
        }

        KeyframeEntry entry;
        entry.id = next_id_++;
        entry.frame_id = frame_id;
        entry.T_world = T_world.Contiguous();
        entry.capture_position = PoseTranslation(entry.T_world);
        entry.depth_histogram =
                ComputeDepthHistogram(rgbd, depth_scale, depth_max);
        if (!BuildKeyframePointCloudAndFeatures(
                    rgbd, intrinsic, entry.T_world, depth_scale, depth_max,
                    device, config_, entry.pcd_world, entry.fpfh_legacy)) {
            --next_id_;
            return false;
        }
        entry.bounds = entry.pcd_world.GetAxisAlignedBoundingBox().ToLegacy();

        entries_.push_back(std::move(entry));
        while (static_cast<int>(entries_.size()) > config_.max_keyframes) {
            entries_.erase(entries_.begin());
        }
        last_added_frame_id_ = frame_id;
        last_added_pose_ = entries_.back().T_world;
        utility::LogInfo(
                "Keyframe added #{} at frame {} ({} points).",
                entries_.back().id, frame_id,
                entries_.back().pcd_world.GetPointPositions().GetLength());
        return true;
    }

    std::vector<int> SelectCandidates(
            const Eigen::Vector3d& query_position,
            const std::vector<double>& live_histogram,
            const RelocalizationConfig& config) const {
        std::lock_guard<std::mutex> lock(mutex_);
        struct CandidateScore {
            int id;
            double spatial;
            double appearance;
            double combined;
        };
        std::vector<CandidateScore> scored;
        scored.reserve(entries_.size());
        for (const auto& entry : entries_) {
            const double spatial =
                    (entry.capture_position - query_position).norm();
            if (spatial > config.candidate_radius_m) {
                continue;
            }
            const double appearance =
                    live_histogram.empty()
                            ? 0.0
                            : HistogramL1Distance(live_histogram,
                                                  entry.depth_histogram);
            scored.push_back({entry.id, spatial, appearance,
                              spatial + 0.5 * appearance});
        }
        std::sort(scored.begin(), scored.end(),
                  [](const CandidateScore& a, const CandidateScore& b) {
                      return a.combined < b.combined;
                  });
        std::vector<int> ids;
        const int limit = std::min(config.max_candidates,
                                   static_cast<int>(scored.size()));
        ids.reserve(limit);
        for (int i = 0; i < limit; ++i) {
            ids.push_back(scored[i].id);
        }
        return ids;
    }

private:
    RelocalizationConfig config_;
    mutable std::mutex mutex_;
    std::vector<KeyframeEntry> entries_;
    int next_id_ = 0;
    int last_added_frame_id_ = -1;
    std::optional<core::Tensor> last_added_pose_;
};

inline pipelines::registration::RegistrationResult RunGlobalRegistration(
        const geometry::PointCloud& source_legacy,
        const geometry::PointCloud& target_legacy,
        const pipelines::registration::Feature& source_fpfh,
        const pipelines::registration::Feature& target_fpfh,
        const RelocalizationConfig& config) {
    const double distance_threshold = config.downsample_voxel * 1.5;
    std::vector<std::reference_wrapper<
            const pipelines::registration::CorrespondenceChecker>>
            checkers;
    pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength edge_checker(
            0.9);
    pipelines::registration::CorrespondenceCheckerBasedOnDistance dist_checker(
            distance_threshold);
    checkers.push_back(edge_checker);
    checkers.push_back(dist_checker);

    if (config.global_method == "fgr") {
        return pipelines::registration::
                FastGlobalRegistrationBasedOnFeatureMatching(
                        source_legacy, target_legacy, source_fpfh,
                        target_fpfh,
                        pipelines::registration::FastGlobalRegistrationOption(
                                distance_threshold));
    }

    return pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(
            source_legacy, target_legacy, source_fpfh, target_fpfh, false,
            distance_threshold,
            pipelines::registration::TransformationEstimationPointToPoint(
                    false),
            3, checkers,
            pipelines::registration::RANSACConvergenceCriteria(
                    config.ransac_max_iter, config.ransac_confidence));
}

inline RelocalizationAttempt TryRelocalizeAgainstKeyframe(
        const t::geometry::PointCloud& pcd_live,
        const pipelines::registration::Feature& fpfh_live_legacy,
        const KeyframeEntry& keyframe,
        const core::Tensor& last_stable_T,
        const RelocalizationConfig& config) {
    RelocalizationAttempt attempt;
    attempt.keyframe_id = keyframe.id;

    const geometry::PointCloud source_legacy = pcd_live.ToLegacy();
    const geometry::PointCloud target_legacy = keyframe.pcd_world.ToLegacy();
    if (source_legacy.points_.empty() || target_legacy.points_.empty()) {
        attempt.reject_reason = "empty point cloud";
        return attempt;
    }

    auto global_result = RunGlobalRegistration(
            source_legacy, target_legacy, fpfh_live_legacy,
            keyframe.fpfh_legacy, config);
    attempt.global_fitness = global_result.fitness_;
    if (global_result.transformation_.trace() == 4.0) {
        attempt.reject_reason = "identity global transform";
        return attempt;
    }

    core::Tensor init_T = core::eigen_converter::EigenMatrixToTensor(
            global_result.transformation_);
    const double max_corr = config.downsample_voxel * 1.4;
    const std::vector<double> voxel_sizes = {
            static_cast<double>(config.downsample_voxel)};
    const std::vector<t::pipelines::registration::ICPConvergenceCriteria>
            criteria = {t::pipelines::registration::ICPConvergenceCriteria(
                    1e-6, 1e-6, 30)};
    const std::vector<double> max_dists = {max_corr};

    auto icp_result = t::pipelines::registration::MultiScaleICP(
            pcd_live, keyframe.pcd_world, voxel_sizes, criteria, max_dists,
            init_T,
            t::pipelines::registration::TransformationEstimationPointToPlane());
    attempt.icp_fitness = icp_result.fitness_;
    attempt.T_live_to_world = icp_result.transformation_.Contiguous();

    auto eval = t::pipelines::registration::EvaluateRegistration(
            pcd_live, keyframe.pcd_world, max_corr, attempt.T_live_to_world);
    if (eval.fitness_ < config.min_fitness) {
        attempt.reject_reason = "low icp fitness";
        return attempt;
    }

    core::Tensor information_tensor =
            t::pipelines::registration::GetInformationMatrix(
                    pcd_live, keyframe.pcd_world, max_corr,
                    attempt.T_live_to_world);
    const int64_t min_points = std::min(
            pcd_live.GetPointPositions().GetLength(),
            keyframe.pcd_world.GetPointPositions().GetLength());
    const Eigen::Matrix6d information =
            core::eigen_converter::TensorToEigenMatrixXd(information_tensor);
    const double information_ratio =
            min_points > 0 ? information(5, 5) / static_cast<double>(min_points)
                           : 0.0;
    attempt.information_ratio = information_ratio;
    if (information_ratio < config.min_information_ratio) {
        attempt.reject_reason = "low information ratio";
        return attempt;
    }

    const double pose_jump =
            TranslationDistance(attempt.T_live_to_world, last_stable_T);
    if (pose_jump > config.max_pose_jump_m) {
        attempt.reject_reason = "pose jump too large";
        return attempt;
    }

    attempt.accepted = true;
    return attempt;
}

inline RelocalizationAttempt Relocalize(
        const t::geometry::RGBDImage& live_rgbd,
        const core::Tensor& intrinsic,
        const KeyframeDatabase& db,
        const std::vector<int>& candidate_ids,
        const core::Tensor& last_stable_T,
        float depth_scale,
        float depth_max,
        const RelocalizationConfig& config,
        const core::Device& device) {
    RelocalizationAttempt best;
    if (candidate_ids.empty()) {
        best.reject_reason = "no candidates";
        return best;
    }

    t::geometry::PointCloud pcd_live = PreprocessLivePointCloud(
            live_rgbd, intrinsic, depth_scale, depth_max, device, config);
    if (!pcd_live.HasPointPositions()) {
        best.reject_reason = "empty live cloud";
        return best;
    }
    core::Tensor fpfh = t::pipelines::registration::ComputeFPFHFeature(
            pcd_live, 100, config.downsample_voxel * 5.0);
    if (fpfh.NumElements() == 0) {
        best.reject_reason = "fpfh failed";
        return best;
    }
    pipelines::registration::Feature fpfh_live_legacy =
            TensorFPFHToLegacyFeature(fpfh);

    for (int candidate_id : candidate_ids) {
        const KeyframeEntry* keyframe = db.GetEntry(candidate_id);
        if (keyframe == nullptr) {
            continue;
        }
        RelocalizationAttempt attempt = TryRelocalizeAgainstKeyframe(
                pcd_live, fpfh_live_legacy, *keyframe, last_stable_T, config);
        if (attempt.accepted &&
            attempt.icp_fitness > best.icp_fitness) {
            best = attempt;
        } else if (!best.accepted && !attempt.accepted &&
                   attempt.icp_fitness > best.icp_fitness) {
            best = attempt;
        }
    }
    return best;
}

struct PoseHypothesis {
    core::Tensor T;
    std::string label;
    double score = -1.0;
    int keyframe_id = -1;
};

class MultiHypothesisTracker {
public:
    explicit MultiHypothesisTracker(RelocalizationConfig config)
        : config_(std::move(config)) {}

    void ResetOnLost(const core::Tensor& last_stable,
                     const core::Tensor& current) {
        hypotheses_.clear();
        AddOrReplace("last_stable", last_stable, -1);
        AddOrReplace("current", current, -1);
    }

    void AddOrReplace(const std::string& label,
                       const core::Tensor& T,
                       int keyframe_id) {
        for (auto& hypothesis : hypotheses_) {
            if (hypothesis.label == label) {
                hypothesis.T = T.Contiguous();
                hypothesis.keyframe_id = keyframe_id;
                hypothesis.score = -1.0;
                return;
            }
        }
        if (static_cast<int>(hypotheses_.size()) >= config_.max_hypotheses) {
            hypotheses_.erase(hypotheses_.begin());
        }
        hypotheses_.push_back({T.Contiguous(), label, -1.0, keyframe_id});
    }

    bool HasHypotheses() const { return !hypotheses_.empty(); }

    const std::vector<PoseHypothesis>& Hypotheses() const {
        return hypotheses_;
    }

    struct TrackProbeResult {
        double fitness = 0.0;
        core::Tensor transformation;
    };

    struct EvaluationResult {
        bool updated = false;
        core::Tensor best_T;
        std::string best_label;
        double best_score = -1.0;
        double best_fitness = 0.0;
    };

    template <typename TrackFn>
    EvaluationResult EvaluateAndPickBest(
            const core::Tensor& last_stable_T,
            TrackFn track_with_pose) {
        EvaluationResult result;
        double best_score = -1.0;
        for (const auto& hypothesis : hypotheses_) {
            TrackProbeResult track = track_with_pose(hypothesis.T);
            const double jump =
                    TranslationDistance(hypothesis.T, last_stable_T);
            const double score = track.fitness - 0.05 * jump;
            if (score > best_score) {
                best_score = score;
                result.best_T = hypothesis.T;
                result.best_label = hypothesis.label;
                result.best_score = score;
                result.best_fitness = track.fitness;
            }
        }
        if (best_score >= 0.0) {
            result.updated = true;
        }
        return result;
    }

    template <typename TrackFn>
    bool TryUpdateFromTracking(const core::Tensor& last_stable_T,
                               TrackFn track_with_pose,
                               core::Tensor& T_frame_to_model,
                               double min_fitness) {
        auto result = EvaluateAndPickBest(last_stable_T, track_with_pose);
        if (!result.updated || result.best_fitness < min_fitness) {
            return false;
        }
        for (auto& hypothesis : hypotheses_) {
            if (hypothesis.label == result.best_label) {
                TrackProbeResult track = track_with_pose(hypothesis.T);
                hypothesis.T =
                        hypothesis.T.Matmul(track.transformation).Contiguous();
                T_frame_to_model = hypothesis.T;
                hypothesis.score = result.best_score;
                return true;
            }
        }
        return false;
    }

private:
    RelocalizationConfig config_;
    std::vector<PoseHypothesis> hypotheses_;
};

}  // namespace relocalization
}  // namespace examples
}  // namespace open3d
