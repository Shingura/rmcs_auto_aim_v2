#include "recorder.hpp"
#include "utility/framerate.hpp"

#include <filesystem>
#include <format>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <memory>
#include <string>
#include <utility>

#include <opencv2/videoio.hpp>

using namespace rmcs;

struct VideoRecorder::Impl {
    struct Session : private cv::VideoWriter {
        using super = VideoWriter;

        FramerateCounter framerate;
        std::size_t fps = 0;

        // 上一次把帧写进视频的时刻，由构造函数初始化
        std::chrono::steady_clock::time_point append_timestamp { };
        // fps 自动测速的起点，仅在未配置 record_fps 时使用
        std::chrono::steady_clock::time_point measure_timestamp =
            std::chrono::steady_clock::time_point::min();
        std::chrono::steady_clock::time_point opened_timestamp =
            std::chrono::steady_clock::time_point::min();

        int cols = 0;
        int rows = 0;

        // 文件路径
        std::filesystem::path video_path { };

        std::ofstream pose_stream { };
        // 已写入视频的帧数
        std::size_t written_frames            = 0;
        std::optional<std::string> pose_error = std::nullopt;

        bool remove_later = false;

        explicit Session(const std::filesystem::path& dir) {
            // 创建一个人类可读的录制文件名称并将其打开
            const auto now =
                std::chrono::floor<std::chrono::seconds>(std::chrono::system_clock::now());
            const auto our_zone  = std::chrono::locate_zone("Asia/Shanghai");
            const auto zone_time = std::chrono::zoned_time { our_zone, now };

            const auto filename = std::format("autoaim_{:%Y-%m-%d_%H-%M-%S}.avi", zone_time);
            video_path          = dir / filename;

            // 写入间隔从会话开始算起，避免未初始化时间点参与比较
            append_timestamp = Clock::now();
        }

        /// @brief 姿态文件路径：与视频同名、同目录，扩展名换成 csv
        auto pose_path() const -> std::filesystem::path {
            auto result = video_path;
            result.replace_extension(".csv");
            return result;
        }

        ~Session() override {
            super::release();
            pose_stream.close();
            if (remove_later) {
                for (const auto& file : { video_path, pose_path() }) {
                    if (std::filesystem::exists(file)) std::filesystem::remove(file);
                }
            }
        }

        auto append(const cv::Mat& mat, const Pose* pose, Clock::time_point timestamp) {
            static const auto kEncode = cv::VideoWriter::fourcc('H', 'F', 'Y', 'U');
            framerate.tick();

            const auto now = std::chrono::steady_clock::now();
            if (fps == 0 && !mat.empty()) [[unlikely]] {
                if (measure_timestamp == std::chrono::steady_clock::time_point::min()) {
                    measure_timestamp = now;
                }
                using namespace std::chrono_literals;
                if (std::chrono::steady_clock::now() - measure_timestamp > 2s) {
                    fps = framerate.fps();
                    fps = fps > 80 ? fps - 20 : fps; // 给 20 帧的冗余
                }
                return;
            }
            if (opened_timestamp == std::chrono::steady_clock::time_point::min()) {
                opened_timestamp = now;
            }

            cols = mat.cols;
            rows = mat.rows;
            if (!super::isOpened()) {
                super::open(video_path, kEncode, static_cast<double>(fps), { cols, rows });
                return;
            }
            using namespace std::chrono_literals;
            if (now - append_timestamp < 1000ms / fps) {
                return;
            }
            super::write(mat);
            append_timestamp = now;

            ++written_frames;
            if (pose) write_pose(*pose, timestamp);
        }

        /// @brief 把一帧的姿态追加到与视频同名的 csv
        auto write_pose(const Pose& pose, Clock::time_point timestamp) -> void {
            if (!pose_stream.is_open()) {
                const auto csv_path = pose_path();

                pose_stream.imbue(std::locale::classic());
                pose_stream.open(csv_path, std::ios::out | std::ios::trunc);
                if (!pose_stream.is_open()) {
                    pose_error = std::format("姿态文件无法写入：{}", csv_path.string());
                    return;
                }
                pose_stream << "frame_index,timestamp_ns,qw,qx,qy,qz,gx,gy,gz\n";
                pose_stream << std::setprecision(std::numeric_limits<double>::max_digits10);
            }

            const auto timestamp_ns =
                std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp.time_since_epoch())
                    .count();

            pose_stream << written_frames - 1 << ',' << timestamp_ns << ',' << pose.orientation.w
                        << ',' << pose.orientation.x << ',' << pose.orientation.y << ','
                        << pose.orientation.z << ',' << pose.gyro_body.x << ',' << pose.gyro_body.y
                        << ',' << pose.gyro_body.z << '\n';

            if (written_frames % 64 == 0) pose_stream.flush();
        }

        auto filename() const { return video_path.string(); }

        auto duration() const {
            return std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - opened_timestamp);
        }

        auto set_remove_later() { remove_later = true; }

        auto set_fps(std::size_t hz) { fps = hz; }
    };
    std::unique_ptr<Session> session = nullptr;

    Config config;
    std::filesystem::path selected_directory { };

    std::optional<std::string> stop_reason = std::nullopt;

    auto stop(bool save = true) {
        if (session && !save) {
            session->set_remove_later();
        }
        if (session && !stop_reason)
            stop_reason = std::format(
                "录制已由用户停止，文件保存至 {}({})", session->filename(), session->duration());
        session = nullptr;
    }

    auto tick(const cv::Mat& mat, const Pose* pose, Clock::time_point timestamp) -> void {
        if (!session) {
            return;
        }
        // 时长过长，则自动停止录制，保存至本地
        if (session->duration() >= config.max_duration) {
            stop_reason = std::format("录制时长超过限制({})，已自动停止，文件保存至 {}",
                config.max_duration, session->filename());
            stop(true);
            return;
        }
        session->append(mat, pose, timestamp);
    }

    auto start() -> std::expected<void, std::string> {
        stop_reason.reset();

        { // 从配置中选择第一个有效的目录
            const auto& directories = config.directories;
            for (const auto& dir : directories) {
                try {
                    if (std::filesystem::exists(dir) || std::filesystem::create_directories(dir)) {
                        selected_directory = dir;
                        break;
                    }
                } catch (const std::filesystem::filesystem_error& e) {
                    continue;
                }
            }
            if (selected_directory.empty()) {
                return std::unexpected { "未能选择有效目录，请确认配置中的保存路径是否可达" };
            }
        }
        { // 判断系统空间是否充足
            constexpr auto kAllowSpaceFree = 50ull * 1024 * 1024 * 1024;

            const auto space = std::filesystem::space(selected_directory);
            if (space.available < kAllowSpaceFree) {
                const auto warn = std::format("空闲空间不足，剩余 {:.3f} GB，无法开始录制",
                    static_cast<double>(space.available) / 1024 / 1024 / 1024);
                return std::unexpected { warn };
            }
        }
        { // 查看录制保存目录下的视频是否超出限制
            auto total_size = std::uintmax_t { 0 };
            for (const auto& entry : std::filesystem::directory_iterator(selected_directory)) {
                if (entry.is_regular_file() && entry.path().extension() == ".avi") {
                    total_size += entry.file_size();
                }
            }
            if (total_size >= config.max_videos_size) {
                const auto warn = std::format("已录制文件总大小已达上限，当前大小 {:.3f} GB",
                    static_cast<double>(total_size) / 1024 / 1024 / 1024);
                return std::unexpected { warn };
            }
        }
        session = std::make_unique<Session>(selected_directory);
        if (config.record_fps != 0) {
            session->set_fps(config.record_fps);
        }

        return { };
    }

    auto status() const -> std::string {
        if (stop_reason) {
            return *stop_reason;
        }
        if (!session) {
            return std::format("未开始录制");
        }
        if (session->pose_error) {
            return std::format(
                "录制中，当前文件为 {}（{}）", session->filename(), *session->pose_error);
        }
        return std::format("录制中，当前文件为 {}", session->filename());
    }
};

auto VideoRecorder::update_config(Config config) -> void { pimpl->config = std::move(config); }

auto VideoRecorder::tick(const cv::Mat& mat, Clock::time_point timestamp) -> void {
    pimpl->tick(mat, nullptr, timestamp);
}

auto VideoRecorder::tick(const cv::Mat& mat, const Pose& pose, Clock::time_point timestamp) -> void {
    pimpl->tick(mat, &pose, timestamp);
}

auto VideoRecorder::start() -> std::expected<void, std::string> { return pimpl->start(); }

auto VideoRecorder::stop(bool save) -> void { pimpl->stop(save); }

auto VideoRecorder::recording() const -> bool { return pimpl->session != nullptr; }

auto VideoRecorder::filename() const -> std::optional<std::string> {
    if (!pimpl->session) {
        return std::nullopt;
    }
    return pimpl->session->filename();
}

auto VideoRecorder::status() const -> std::string { return pimpl->status(); }

VideoRecorder::VideoRecorder() noexcept
    : pimpl { std::make_unique<Impl>() } { }

VideoRecorder::~VideoRecorder() noexcept = default;
