#include "utility/image/pose_csv.hpp"

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

using namespace rmcs::util;

namespace {

constexpr auto kHeader = "frame_index,timestamp_ns,qw,qx,qy,qz,gx,gy,gz\n";

auto write_csv(const std::string& content) -> std::filesystem::path {
    const auto path = std::filesystem::temp_directory_path() / "rmcs_pose_csv_test.csv";

    auto file = std::ofstream { path, std::ios::out | std::ios::trunc };
    file << content;

    return path;
}

}

TEST(pose_csv, reads_rows_in_order) {
    const auto path = write_csv(std::string { kHeader }
        + "0,100,1,0,0,0,0.1,0.2,0.3\n"
          "1,200,0.5,0.5,0.5,0.5,1,2,3\n");

    const auto rows = read_pose_csv(path);
    ASSERT_EQ(rows.size(), 2);

    EXPECT_EQ(rows[0].frame_index, 0);
    EXPECT_EQ(rows[0].timestamp_ns, 100);
    EXPECT_DOUBLE_EQ(rows[0].qw, 1.0);
    EXPECT_DOUBLE_EQ(rows[0].gx, 0.1);
    EXPECT_DOUBLE_EQ(rows[0].gz, 0.3);

    EXPECT_EQ(rows[1].frame_index, 1);
    EXPECT_DOUBLE_EQ(rows[1].qy, 0.5);
    EXPECT_DOUBLE_EQ(rows[1].gy, 2.0);
}

TEST(pose_csv, throws_on_wrong_column_count) {
    const auto path = write_csv(std::string { kHeader } + "0,100,1,0,0,0\n");

    EXPECT_THROW(read_pose_csv(path), std::runtime_error);
}

TEST(pose_csv, throws_when_the_file_is_missing) {
    EXPECT_THROW(read_pose_csv("/tmp/rmcs_pose_csv_missing.csv"), std::runtime_error);
}

TEST(pose_csv, pose_at_returns_the_row_of_that_frame) {
    const auto rows = std::vector<PoseRow> {
        PoseRow { .frame_index = 0, .qw = 1.0 },
        PoseRow { .frame_index = 1, .qx = 0.5 },
    };

    const auto* row = pose_at(rows, 1);
    ASSERT_NE(row, nullptr);
    EXPECT_EQ(row->frame_index, 1);
    EXPECT_DOUBLE_EQ(row->qx, 0.5);
}

TEST(pose_csv, pose_at_returns_null_out_of_range) {
    const auto rows = std::vector<PoseRow> { PoseRow { .frame_index = 0 } };

    EXPECT_EQ(pose_at(rows, 0), &rows[0]);
    EXPECT_EQ(pose_at(rows, 1), nullptr);
}

TEST(pose_csv, pose_at_returns_null_when_the_row_index_does_not_match) {
    // 行号与帧号不一致：这份 csv 不属于这个视频
    const auto rows = std::vector<PoseRow> { PoseRow { .frame_index = 7 } };

    EXPECT_EQ(pose_at(rows, 0), nullptr);
}
