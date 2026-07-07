// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2024 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------
//
// Shared tracking tier classification and frame-to-frame pose bridge helpers
// for OnlineSLAMUtil.h and RealTimeSLAMRealSense.cpp.

#pragma once

#include <cmath>
#include <string>

#include "open3d/Open3D.h"

namespace open3d {
namespace examples {
namespace slam_tracking {

static constexpr double kWeakFitnessMin = 0.08;
static constexpr double kWeakTranslationMax = 0.30;
static constexpr double kOutlierTranslation = 0.50;
static constexpr double kOutlierRotationDeg = 25.0;
static constexpr float kOdometryHuberDelta = 0.05f;

// RealTimeSLAMRealSense strong-tier thresholds.
static constexpr double kRealtimePoseFitnessMin = 0.12;
static constexpr double kRealtimePoseTranslationMax = 0.15;

// OnlineSLAM strong-tier thresholds (rotation-aware).
static constexpr double kOnlinePoseTranslationMax = 0.12;
static constexpr double kOnlinePoseRotationMaxDeg = 8.0;
static constexpr double kOnlineMinFitnessDefault = 0.25;

enum class TrackingTier { kInit, kStrong, kWeak, kOutlier, kFail };

inline float SafeOdometryDepthDiff(float depth_diff) {
    return std::max(kOdometryHuberDelta + 0.001f, depth_diff);
}

inline bool IsOdometrySingularError(const std::exception& e) {
    const std::string msg = e.what();
    return msg.find("Singular 6x6") != std::string::npos ||
           msg.find("singular condition") != std::string::npos;
}

inline double TranslationNorm(const core::Tensor& transformation) {
    core::Tensor translation =
            transformation.Slice(0, 0, 3).Slice(1, 3, 4);
    return std::sqrt(
            (translation * translation).Sum({0, 1}).Item<double>());
}

inline double RotationDeg(const core::Tensor& transformation) {
    core::Tensor T_cpu = transformation.To(core::Device("CPU:0"), core::Float64)
                                 .Contiguous();
    const double* t_ptr = T_cpu.GetDataPtr<double>();
    const double trace = t_ptr[0] + t_ptr[5] + t_ptr[10];
    const double cos_angle =
            std::max(-1.0, std::min(1.0, 0.5 * (trace - 1.0)));
    return std::acos(cos_angle) * (180.0 / 3.14159265358979323846);
}

inline void MeasureMotion(const core::Tensor& transformation,
                          double& translation,
                          double& rotation_deg) {
    translation = TranslationNorm(transformation);
    rotation_deg = RotationDeg(transformation);
}

inline TrackingTier ClassifyTrackingRealTime(double fitness,
                                             double translation) {
    if (translation >= kOutlierTranslation) {
        return TrackingTier::kOutlier;
    }
    if (fitness >= kRealtimePoseFitnessMin &&
        translation < kRealtimePoseTranslationMax) {
        return TrackingTier::kStrong;
    }
    if (fitness >= kWeakFitnessMin && translation < kWeakTranslationMax) {
        return TrackingTier::kWeak;
    }
    return TrackingTier::kFail;
}

inline TrackingTier ClassifyTrackingOnline(double fitness,
                                           double translation,
                                           double rotation_deg,
                                           double min_fitness) {
    if (translation >= kOutlierTranslation ||
        rotation_deg >= kOutlierRotationDeg) {
        return TrackingTier::kOutlier;
    }
    if (fitness >= min_fitness && translation < kOnlinePoseTranslationMax &&
        rotation_deg < kOnlinePoseRotationMaxDeg) {
        return TrackingTier::kStrong;
    }
    if (fitness >= kWeakFitnessMin && translation < kWeakTranslationMax) {
        return TrackingTier::kWeak;
    }
    return TrackingTier::kFail;
}

inline bool FrameToFrameBridgeAccepted(double fitness, double translation) {
    return fitness >= kWeakFitnessMin && translation < kWeakTranslationMax &&
           translation < kOutlierTranslation;
}

// Stricter gate for relocalizing Outlier: only small per-frame motion.
inline bool FrameToFrameBridgeAcceptedRelocalizing(double fitness,
                                                   double translation,
                                                   double rotation_deg) {
    return fitness >= kWeakFitnessMin &&
           translation < kOnlinePoseTranslationMax &&
           rotation_deg < kOnlinePoseRotationMaxDeg &&
           translation < kOutlierTranslation;
}

inline bool ShouldAttemptFrameToFrameBridge(TrackingTier tier,
                                            bool relocalizing) {
    if (tier == TrackingTier::kOutlier) {
        return relocalizing;
    }
    return tier == TrackingTier::kWeak || tier == TrackingTier::kFail;
}

inline const char* TrackingTierName(TrackingTier tier) {
    switch (tier) {
        case TrackingTier::kInit:
            return "init";
        case TrackingTier::kStrong:
            return "strong";
        case TrackingTier::kWeak:
            return "weak";
        case TrackingTier::kOutlier:
            return "outlier";
        case TrackingTier::kFail:
            return "fail";
    }
    return "unknown";
}

}  // namespace slam_tracking
}  // namespace examples
}  // namespace open3d
