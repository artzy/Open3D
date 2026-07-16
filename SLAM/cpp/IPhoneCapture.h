// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2024 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------
//
// Record3D USB stream adapter: iPhone LiDAR RGB-D + ARKit pose -> Open3D.
// Depth arrives as float32 meters and is converted to uint16 millimeters
// (depth_scale = 1000) to match Open3D Integrate (uint16,uint8). RGB is
// downscaled to depth resolution. ARKit (-Z forward) poses are converted to
// OpenCV/Open3D (+Z forward) via diag(1,-1,-1).
//

#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <record3d/Record3DStream.h>

#include "open3d/Open3D.h"

namespace open3d {
namespace examples {
namespace iphone_capture {

/// One synchronized RGB-D frame plus optional ARKit pose (OpenCV convention).
struct CapturedFrame {
    t::geometry::RGBDImage rgbd;
    core::Tensor pose_opencv;  // 4x4 Float64, empty if unavailable
    bool has_pose = false;
    float fx = 0.f;
    float fy = 0.f;
    float cx = 0.f;
    float cy = 0.f;
    int width = 0;
    int height = 0;
};

inline core::Tensor CameraPoseToOpenCVTensor(
        const Record3D::CameraPose& pose) {
    Eigen::Quaterniond q(pose.qw, pose.qx, pose.qy, pose.qz);
    if (q.norm() < 1e-8) {
        q = Eigen::Quaterniond::Identity();
    } else {
        q.normalize();
    }
    Eigen::Matrix4d T_arkit = Eigen::Matrix4d::Identity();
    T_arkit.block<3, 3>(0, 0) = q.toRotationMatrix();
    T_arkit(0, 3) = pose.tx;
    T_arkit(1, 3) = pose.ty;
    T_arkit(2, 3) = pose.tz;

    // ARKit camera: -Z forward, Y up. OpenCV/Open3D: +Z forward, Y down.
    Eigen::Matrix4d C = Eigen::Matrix4d::Identity();
    C(1, 1) = -1.0;
    C(2, 2) = -1.0;
    const Eigen::Matrix4d T_cv = C * T_arkit * C;
    return core::eigen_converter::EigenMatrixToTensor(T_cv);
}

inline core::Tensor InvertPose(const core::Tensor& T) {
    Eigen::Matrix4d M =
            core::eigen_converter::TensorToEigenMatrixXd(T);
    Eigen::Matrix4d Minv = Eigen::Matrix4d::Identity();
    Minv.block<3, 3>(0, 0) = M.block<3, 3>(0, 0).transpose();
    Minv.block<3, 1>(0, 3) =
            -Minv.block<3, 3>(0, 0) * M.block<3, 1>(0, 3);
    return core::eigen_converter::EigenMatrixToTensor(Minv);
}

/// Relative transform T_prev^{-1} * T_curr (motion of camera in world).
inline core::Tensor RelativePose(const core::Tensor& T_prev,
                                 const core::Tensor& T_curr) {
    return InvertPose(T_prev).Matmul(T_curr);
}

class IPhoneCapture {
public:
    IPhoneCapture() = default;
    ~IPhoneCapture() { Stop(); }

    IPhoneCapture(const IPhoneCapture&) = delete;
    IPhoneCapture& operator=(const IPhoneCapture&) = delete;

    /// Connect to the first USB Record3D device. Returns false on failure.
    bool Start() {
        Stop();
        const auto devices = Record3D::Record3DStream::GetConnectedDevices();
        if (devices.empty()) {
            utility::LogWarning(
                    "No iOS devices found via USB. Install Apple Mobile Device "
                    "Support / iTunes, unlock the phone, and trust this PC.");
            return false;
        }
        for (const auto& d : devices) {
            utility::LogInfo("Found iOS device productId={} udid={}",
                             d.productId, d.udid);
        }

        stream_ = std::make_unique<Record3D::Record3DStream>();
        stream_->onStreamStopped = [this]() {
            connected_.store(false);
            utility::LogWarning("Record3D stream stopped.");
            cv_.notify_all();
        };
        stream_->onNewFrame =
                [this](const Record3D::BufferRGB& rgb,
                       const Record3D::BufferDepth& depth,
                       const Record3D::BufferConfidence& /*conf*/,
                       const Record3D::BufferMisc& /*misc*/,
                       uint32_t rgb_w, uint32_t rgb_h, uint32_t depth_w,
                       uint32_t depth_h, uint32_t /*conf_w*/,
                       uint32_t /*conf_h*/, Record3D::DeviceType device_type,
                       Record3D::IntrinsicMatrixCoeffs K,
                       Record3D::CameraPose camera_pose) {
                    OnNewFrame(rgb, depth, rgb_w, rgb_h, depth_w, depth_h,
                               device_type, K, camera_pose);
                };

        if (!stream_->ConnectToDevice(devices[0])) {
            utility::LogWarning(
                    "Could not connect to iPhone. Open Record3D, enable USB "
                    "Streaming, then press Record.");
            stream_.reset();
            return false;
        }
        connected_.store(true);
        utility::LogInfo(
                "Connected to Record3D USB stream. Enable USB Streaming in "
                "the Record3D app and start recording.");
        return true;
    }

    void Stop() {
        if (stream_) {
            try {
                stream_->Disconnect();
            } catch (...) {
            }
            stream_.reset();
        }
        connected_.store(false);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            has_frame_ = false;
        }
        cv_.notify_all();
    }

    bool IsConnected() const { return connected_.load(); }

    /// Block until a frame is available (or timeout). Empty rgbd on failure.
    CapturedFrame CaptureFrame(int timeout_ms = 500) {
        CapturedFrame out;
        std::unique_lock<std::mutex> lock(mutex_);
        if (!has_frame_) {
            cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                         [this]() { return has_frame_ || !connected_.load(); });
        }
        if (!has_frame_) {
            return out;
        }
        out = latest_;
        has_frame_ = false;
        return out;
    }

    /// Non-blocking peek of last intrinsics (valid after first frame).
    bool GetIntrinsics(float& fx, float& fy, float& cx, float& cy, int& w,
                       int& h) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!intrinsics_valid_) {
            return false;
        }
        fx = fx_;
        fy = fy_;
        cx = cx_;
        cy = cy_;
        w = width_;
        h = height_;
        return true;
    }

    static constexpr float kDepthScale = 1000.f;  // uint16 millimeters

private:
    void OnNewFrame(const Record3D::BufferRGB& rgb,
                    const Record3D::BufferDepth& depth,
                    uint32_t rgb_w,
                    uint32_t rgb_h,
                    uint32_t depth_w,
                    uint32_t depth_h,
                    Record3D::DeviceType device_type,
                    Record3D::IntrinsicMatrixCoeffs K,
                    Record3D::CameraPose camera_pose) {
        if (rgb_w == 0 || rgb_h == 0 || depth_w == 0 || depth_h == 0) {
            return;
        }
        if (rgb.size() < static_cast<size_t>(rgb_w) * rgb_h * 3 ||
            depth.size() < static_cast<size_t>(depth_w) * depth_h *
                                   sizeof(float)) {
            return;
        }

        // Downscale RGB to depth resolution (lighter SLAM input).
        const int out_w = static_cast<int>(depth_w);
        const int out_h = static_cast<int>(depth_h);
        std::vector<uint8_t> color_buf(static_cast<size_t>(out_w) * out_h * 3);
        for (int y = 0; y < out_h; ++y) {
            const int sy = static_cast<int>((static_cast<int64_t>(y) * rgb_h) /
                                            depth_h);
            for (int x = 0; x < out_w; ++x) {
                const int sx = static_cast<int>(
                        (static_cast<int64_t>(x) * rgb_w) / depth_w);
                const size_t src =
                        (static_cast<size_t>(sy) * rgb_w + sx) * 3;
                const size_t dst =
                        (static_cast<size_t>(y) * out_w + x) * 3;
                color_buf[dst] = rgb[src];
                color_buf[dst + 1] = rgb[src + 1];
                color_buf[dst + 2] = rgb[src + 2];
            }
        }

        // TrueDepth selfie streams are mirrored in Record3D demos; LiDAR is not.
        if (device_type == Record3D::R3D_DEVICE_TYPE__FACEID) {
            for (int y = 0; y < out_h; ++y) {
                uint8_t* row = color_buf.data() +
                               static_cast<size_t>(y) * out_w * 3;
                for (int x = 0; x < out_w / 2; ++x) {
                    const int xr = out_w - 1 - x;
                    for (int c = 0; c < 3; ++c) {
                        std::swap(row[x * 3 + c], row[xr * 3 + c]);
                    }
                }
            }
        }

        core::Tensor color_t({out_h, out_w, 3}, core::UInt8,
                             core::Device("CPU:0"));
        std::memcpy(color_t.GetDataPtr(), color_buf.data(), color_buf.size());

        // Record3D depth is float meters; Open3D Integrate expects
        // (uint16,uint8) or (float,float). Convert meters -> uint16 mm so we
        // can keep uint8 color (same as RealSense path).
        const auto* depth_m =
                reinterpret_cast<const float*>(depth.data());
        core::Tensor depth_t({out_h, out_w}, core::UInt16,
                             core::Device("CPU:0"));
        auto* depth_mm = depth_t.GetDataPtr<uint16_t>();
        const size_t npix = static_cast<size_t>(out_w) * out_h;
        for (size_t i = 0; i < npix; ++i) {
            const float m = depth_m[i];
            if (!(m > 0.0f) || !std::isfinite(m)) {
                depth_mm[i] = 0;
                continue;
            }
            const float mm = m * 1000.0f;
            depth_mm[i] = mm >= 65535.0f
                                  ? static_cast<uint16_t>(65535)
                                  : static_cast<uint16_t>(mm + 0.5f);
        }

        if (device_type == Record3D::R3D_DEVICE_TYPE__FACEID) {
            // Mirror depth to match mirrored color.
            for (int y = 0; y < out_h; ++y) {
                uint16_t* row = depth_mm + static_cast<size_t>(y) * out_w;
                for (int x = 0; x < out_w / 2; ++x) {
                    std::swap(row[x], row[out_w - 1 - x]);
                }
            }
        }

        // Scale RGB intrinsics to the downscaled (depth) resolution.
        const float sx = static_cast<float>(depth_w) /
                         static_cast<float>(rgb_w);
        const float sy = static_cast<float>(depth_h) /
                         static_cast<float>(rgb_h);
        float fx = K.fx * sx;
        float fy = K.fy * sy;
        float cx = K.tx * sx;
        float cy = K.ty * sy;
        if (device_type == Record3D::R3D_DEVICE_TYPE__FACEID) {
            cx = static_cast<float>(out_w - 1) - cx;
        }

        CapturedFrame frame;
        frame.rgbd = t::geometry::RGBDImage(t::geometry::Image(color_t),
                                            t::geometry::Image(depth_t), true);
        frame.pose_opencv = CameraPoseToOpenCVTensor(camera_pose);
        frame.has_pose = true;
        frame.fx = fx;
        frame.fy = fy;
        frame.cx = cx;
        frame.cy = cy;
        frame.width = out_w;
        frame.height = out_h;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_ = std::move(frame);
            fx_ = fx;
            fy_ = fy;
            cx_ = cx;
            cy_ = cy;
            width_ = out_w;
            height_ = out_h;
            intrinsics_valid_ = true;
            has_frame_ = true;
        }
        cv_.notify_all();
    }

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    CapturedFrame latest_;
    bool has_frame_ = false;
    bool intrinsics_valid_ = false;
    float fx_ = 0.f;
    float fy_ = 0.f;
    float cx_ = 0.f;
    float cy_ = 0.f;
    int width_ = 0;
    int height_ = 0;
    std::atomic<bool> connected_{false};
    std::unique_ptr<Record3D::Record3DStream> stream_;
};

}  // namespace iphone_capture
}  // namespace examples
}  // namespace open3d
