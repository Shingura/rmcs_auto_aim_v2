#include "utility/image/recorder.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

using namespace rmcs;

namespace {

/// @NOTE:
///  录像器按写入间隔丢帧：距上一帧不足 1/fps 秒的帧不会写进视频。
///  测试里的帧间隔取得比它大，让每一帧都能写进去
constexpr auto kRecordFps     = std::size_t { 5 };
constexpr auto kFrameInterval = std::chrono::milliseconds { 300 };
constexpr auto kFrameCount    = std::size_t { 5 };

auto make_frame(std::uint8_t value) -> cv::Mat {
    return cv::Mat { 64, 96, CV_8UC3, cv::Scalar::all(static_cast<double>(value)) };
}

/// @brief 第 index 次写入用的姿态。各次取值不同，便于检查行与帧的对应关系
auto make_pose(std::size_t index) -> VideoRecorder::Pose {
    const auto value = static_cast<double>(index) + 1.0;
    return VideoRecorder::Pose {
        .orientation = Orientation { value * 0.1, value * 0.2, value * 0.3, 1.0 },
        .gyro_body   = Vector3d { value, value * 2.0, value * 3.0 },
    };
}

struct CsvRow {
    std::size_t frame_index   = 0;
    std::int64_t timestamp_ns = 0;

    double qw = 0.0, qx = 0.0, qy = 0.0, qz = 0.0;
    double gx = 0.0, gy = 0.0, gz = 0.0;
};

auto read_csv_rows(const std::filesystem::path& path) -> std::vector<CsvRow> {
    auto file  = std::ifstream { path };
    auto line  = std::string { };
    auto rows  = std::vector<CsvRow> { };
    auto first = true;

    while (std::getline(file, line)) {
        if (first) { // 表头
            first = false;
            continue;
        }
        if (line.empty()) continue;

        auto fields = std::vector<std::string> { };
        auto field  = std::string { };
        auto stream = std::istringstream { line };
        while (std::getline(stream, field, ',')) fields.push_back(field);
        if (fields.size() != 9) continue;

        auto row         = CsvRow { };
        row.frame_index  = std::stoull(fields[0]);
        row.timestamp_ns = std::stoll(fields[1]);
        row.qw           = std::stod(fields[2]);
        row.qx           = std::stod(fields[3]);
        row.qy           = std::stod(fields[4]);
        row.qz           = std::stod(fields[5]);
        row.gx           = std::stod(fields[6]);
        row.gy           = std::stod(fields[7]);
        row.gz           = std::stod(fields[8]);
        rows.push_back(row);
    }

    return rows;
}

}

TEST(video_recorder, writes_one_pose_row_for_each_written_frame) {
    const auto dir = std::filesystem::temp_directory_path() / "rmcs_recorder_pose_test";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);

    auto recorder = VideoRecorder { };
    recorder.update_config({
        .directories     = { dir.string() },
        .max_duration    = std::chrono::seconds { 60 },
        .record_fps      = kRecordFps,
        .max_videos_size = 30ull * 1024 * 1024 * 1024,
    });

    if (const auto started = recorder.start(); !started) {
        GTEST_SKIP() << "录像器无法开始录制，跳过：" << started.error();
    }

    for (auto i = std::size_t { 0 }; i < kFrameCount; ++i) {
        recorder.tick(make_frame(static_cast<std::uint8_t>(i * 40)), make_pose(i),
            VideoRecorder::Clock::time_point { } + std::chrono::seconds { i + 1 });
        std::this_thread::sleep_for(kFrameInterval);
    }

    // 停止之后 filename() 不再返回值，先取出来
    const auto video_name = recorder.filename();
    recorder.stop();

    ASSERT_TRUE(video_name.has_value());
    const auto video_path = std::filesystem::path { *video_name };
    auto pose_path        = video_path;
    pose_path.replace_extension(".csv");

    if (!std::filesystem::exists(video_path) || std::filesystem::file_size(video_path) == 0) {
        GTEST_SKIP() << "没有写出视频文件（可能缺少 HFYU 编码器），跳过";
    }

    { // 表头
        auto file   = std::ifstream { pose_path };
        auto header = std::string { };
        ASSERT_TRUE(std::filesystem::exists(pose_path));
        std::getline(file, header);
        EXPECT_EQ(header, "frame_index,timestamp_ns,qw,qx,qy,qz,gx,gy,gz");
    }

    const auto rows = read_csv_rows(pose_path);
    // 第一次 tick 只用来打开视频文件，不写帧
    ASSERT_EQ(rows.size(), kFrameCount - 1);

    for (auto i = std::size_t { 0 }; i < rows.size(); ++i) {
        const auto pose = make_pose(i + 1);

        // 第 0 次调用只用来打开视频文件，因此第 i 行对应第 i+1 次调用，
        // 而第 k 次调用传入的时间戳是 k+1 秒
        EXPECT_EQ(rows[i].frame_index, i);
        EXPECT_EQ(rows[i].timestamp_ns,
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::seconds { i + 2 })
                .count());
        EXPECT_DOUBLE_EQ(rows[i].qw, pose.orientation.w);
        EXPECT_DOUBLE_EQ(rows[i].qx, pose.orientation.x);
        EXPECT_DOUBLE_EQ(rows[i].qy, pose.orientation.y);
        EXPECT_DOUBLE_EQ(rows[i].qz, pose.orientation.z);
        EXPECT_DOUBLE_EQ(rows[i].gx, pose.gyro_body.x);
        EXPECT_DOUBLE_EQ(rows[i].gy, pose.gyro_body.y);
        EXPECT_DOUBLE_EQ(rows[i].gz, pose.gyro_body.z);
    }

    // csv 的行数应当等于视频里真正的帧数
    auto capture = cv::VideoCapture { video_path.string() };
    if (capture.isOpened()) {
        EXPECT_EQ(static_cast<std::size_t>(capture.get(cv::CAP_PROP_FRAME_COUNT)), rows.size());
    } else {
        GTEST_LOG_(INFO) << "视频读不回来，跳过帧数比对：" << video_path.string();
    }

    std::filesystem::remove_all(dir);
}
