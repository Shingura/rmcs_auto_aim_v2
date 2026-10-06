#pragma once
#include "utility/math/linear.hpp"
#include "utility/serializable.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <vector>

namespace rmcs::util {

/// @brief 录像 csv 的一行，即一帧的姿态快照
struct PoseRow : Serializable {
    std::size_t frame_index   = 0;
    std::int64_t timestamp_ns = 0;

    double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
    double gx = 0.0, gy = 0.0, gz = 0.0;

    static constexpr std::tuple metas {
        // clang-format off
        &PoseRow::frame_index,  "frame_index",
        &PoseRow::timestamp_ns, "timestamp_ns",
        &PoseRow::qw,           "qw",
        &PoseRow::qx,           "qx",
        &PoseRow::qy,           "qy",
        &PoseRow::qz,           "qz",
        &PoseRow::gx,           "gx",
        &PoseRow::gy,           "gy",
        &PoseRow::gz,           "gz",
        // clang-format on
    };
};

/// @brief 读取姿态 csv，按文件顺序返回所有行
/// @throws std::runtime_error 文件打不开，或某一行的列数与表头不一致
auto read_pose_csv(const std::filesystem::path& path) -> std::vector<PoseRow>;

/// @brief 取第 frame_index 帧的姿态
/// @return 行号与帧号不一致、或超出范围时返回空指针，
///         调用方据此判断这份 csv 是否属于该视频、以及是否读到了结尾
auto pose_at(const std::vector<PoseRow>& rows, std::size_t frame_index) -> const PoseRow*;

}
