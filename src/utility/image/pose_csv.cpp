#include "pose_csv.hpp"

#include "utility/csv/csv_reader.hpp"

#include <stdexcept>
#include <string>

namespace rmcs::util {

auto read_pose_csv(const std::filesystem::path& path) -> std::vector<PoseRow> {
    auto reader = CsvReader { path };
    auto rows   = std::vector<PoseRow> { };
    auto line   = std::size_t { 2 };

    while (true) {
        if (reader.corrupted_line()) {
            throw std::runtime_error(
                "line " + std::to_string(line) + " has unexpected column count");
        }
        if (reader.eof()) break;

        auto row = PoseRow { };
        if (auto result = row.serialize(reader); !result) {
            throw std::runtime_error("line " + std::to_string(line) + ": " + result.error());
        }
        rows.push_back(row);
        ++line;

        if (!reader.next()) {
            if (reader.corrupted_line()) {
                throw std::runtime_error(
                    "line " + std::to_string(line) + " has unexpected column count");
            }
            if (reader.eof()) break;
        }
    }

    return rows;
}

auto pose_at(const std::vector<PoseRow>& rows, std::size_t frame_index) -> const PoseRow* {
    if (frame_index >= rows.size()) return nullptr;
    // 行号对不上，说明这份 csv 不属于这个视频
    if (rows[frame_index].frame_index != frame_index) return nullptr;

    return &rows[frame_index];
}

}
