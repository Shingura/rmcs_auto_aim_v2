#include "utility/image/frame_bounds.hpp"

#include <gtest/gtest.h>

using namespace rmcs::util;

namespace {

// 采集组件与视频播放组件实际输出的帧尺寸，边距取 config.yaml 里的 50
constexpr auto kWidth  = 1440.0;
constexpr auto kHeight = 1080.0;
constexpr auto kMargin = 50.0;

// 旧实现拿相机内参主点乘二当图像尺寸（701.3 * 2 与 564.6 * 2）
constexpr auto kLegacyWidth  = 1402.6;
constexpr auto kLegacyHeight = 1129.2;

auto box(double min_x, double max_x, double min_y, double max_y) -> BoundingBox2d {
    return BoundingBox2d { min_x, max_x, min_y, max_y };
}

}

TEST(within_image_margin, keeps_a_box_away_from_every_edge) {
    EXPECT_TRUE(within_image_margin(box(400.0, 600.0, 300.0, 500.0), kWidth, kHeight, kMargin));
}

TEST(within_image_margin, rejects_boxes_near_each_edge) {
    // 左、右、上、下各给一块贴边的装甲板
    EXPECT_FALSE(within_image_margin(box(10.0, 200.0, 300.0, 500.0), kWidth, kHeight, kMargin));
    EXPECT_FALSE(within_image_margin(box(1240.0, 1430.0, 300.0, 500.0), kWidth, kHeight, kMargin));
    EXPECT_FALSE(within_image_margin(box(400.0, 600.0, 10.0, 200.0), kWidth, kHeight, kMargin));
    EXPECT_FALSE(within_image_margin(box(400.0, 600.0, 900.0, 1070.0), kWidth, kHeight, kMargin));
}

TEST(within_image_margin, margin_edge_is_inclusive) {
    // 四边正好落在边距上算通过，再偏一点就丢
    EXPECT_TRUE(within_image_margin(box(50.0, 1390.0, 50.0, 1030.0), kWidth, kHeight, kMargin));
    EXPECT_FALSE(within_image_margin(box(49.9, 1390.0, 50.0, 1030.0), kWidth, kHeight, kMargin));
    EXPECT_FALSE(within_image_margin(box(50.0, 1390.1, 50.0, 1030.0), kWidth, kHeight, kMargin));
    EXPECT_FALSE(within_image_margin(box(50.0, 1390.0, 49.9, 1030.0), kWidth, kHeight, kMargin));
    EXPECT_FALSE(within_image_margin(box(50.0, 1390.0, 50.0, 1030.1), kWidth, kHeight, kMargin));
}

TEST(within_image_margin, rejects_boxes_outside_the_frame) {
    EXPECT_FALSE(within_image_margin(box(-10.0, 200.0, 300.0, 500.0), kWidth, kHeight, kMargin));
    EXPECT_FALSE(within_image_margin(box(400.0, 1500.0, 300.0, 500.0), kWidth, kHeight, kMargin));
    EXPECT_FALSE(within_image_margin(box(400.0, 600.0, -10.0, 500.0), kWidth, kHeight, kMargin));
    EXPECT_FALSE(within_image_margin(box(400.0, 600.0, 300.0, 1200.0), kWidth, kHeight, kMargin));
}

TEST(within_image_margin, admits_everything_when_the_frame_size_is_unknown) {
    // 拿不到帧尺寸时不做判断，避免异常路径上把观测全部丢掉
    EXPECT_TRUE(within_image_margin(box(0.0, 1440.0, 0.0, 1080.0), 0.0, 0.0, kMargin));
    EXPECT_TRUE(within_image_margin(box(0.0, 1440.0, 0.0, 1080.0), kWidth, 0.0, kMargin));
    EXPECT_TRUE(within_image_margin(box(0.0, 1440.0, 0.0, 1080.0), -1.0, kHeight, kMargin));
}

// 下面两条把修复前后差在哪钉住：同一块板，用真实帧尺寸与旧实现的尺寸会得到相反结论

TEST(within_image_margin, bottom_edge_is_covered_by_the_real_frame_height) {
    // 下边缘 50 px 内的板：真实高度 1080 时门槛是 1030，应当丢掉；
    // 旧实现把高当成 1129.2（门槛 1079.2），会把它放进滤波器
    const auto near_bottom = box(600.0, 800.0, 900.0, 1040.0);

    EXPECT_FALSE(within_image_margin(near_bottom, kWidth, kHeight, kMargin));
    EXPECT_TRUE(within_image_margin(near_bottom, kLegacyWidth, kLegacyHeight, kMargin));
}

TEST(within_image_margin, right_edge_no_longer_drops_complete_plates) {
    // 右侧 70 px 处的完整装甲板：真实宽度 1440 时门槛是 1390，应当收下；
    // 旧实现把宽当成 1402.6（门槛 1352.6），会把它误丢
    const auto complete = box(1200.0, 1370.0, 500.0, 600.0);

    EXPECT_TRUE(within_image_margin(complete, kWidth, kHeight, kMargin));
    EXPECT_FALSE(within_image_margin(complete, kLegacyWidth, kLegacyHeight, kMargin));
}
