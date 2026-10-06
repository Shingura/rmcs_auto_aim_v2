#pragma once
#include "utility/math/linear.hpp"
#include "utility/pimpl.hpp"

#include <chrono>
#include <expected>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

namespace rmcs {

class VideoRecorder {
    RMCS_PIMPL_DEFINITION(VideoRecorder)
public:
    using Clock = std::chrono::steady_clock;

    struct Config {
        std::vector<std::string> directories {
            "/home/root/autoaim/",
            "/home/ubuntu/autoaim/",
            "/tmp/autoaim/",
        };
        std::chrono::seconds max_duration { 60 };
        std::size_t record_fps = 0;

        std::uintmax_t max_videos_size = 30ull * 1024 * 1024 * 1024; // 30 GB
    };
    auto update_config(Config) -> void;

    /// @brief 一帧的云台姿态快照
    struct Pose {
        // 曝光时刻云台在 OdomImu 坐标系下的姿态
        Orientation orientation { };
        // 同一时刻的机身角速度
        Vector3d gyro_body { };
    };

    /// @brief 只录制一帧画面，不记录姿态，不产生 csv
    auto tick(const cv::Mat&, Clock::time_point = Clock::now()) -> void;

    /// @brief 录制一帧画面，同时记录这一帧的姿态，产生 csv
    ///
    /// @NOTE:
    ///  每写入一帧视频，就在与视频同名的 csv 里追加一行，列为
    ///  `frame_index,timestamp_ns,qw,qx,qy,qz,gx,gy,gz`。
    ///  frame_index 从 0 开始，csv 的第 N 行与视频的第 N 帧
    ///  一一对应，回放侧据此把姿态还原到每一帧上。
    auto tick(const cv::Mat&, const Pose&, Clock::time_point = Clock::now()) -> void;

    auto start() -> std::expected<void, std::string>;
    auto stop(bool save = true) -> void;

    auto recording() const -> bool;
    auto filename() const -> std::optional<std::string>;

    auto status() const -> std::string;
};

}
