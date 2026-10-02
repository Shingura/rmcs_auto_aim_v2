#include "utility/math/camera.hpp"

#include <cmath>
#include <gtest/gtest.h>

// Point3d、Transform 等在 rmcs 下，camera.hpp 的两个函数在 rmcs::util 下
using namespace rmcs;
using namespace rmcs::util;

namespace {

constexpr auto kPi = 3.14159265358979323846;

// 相机放在原点，朝 ROS 系 +X 方向，无旋转
auto camera_at_origin() -> Transform { return Transform::kIdentity(); }

auto degrees(double value) -> double { return value * kPi / 180.0; }

// 在正前方 distance 米处，水平偏开 angle 弧度的点
auto point_at(double distance, double angle) -> Point3d {
    return Point3d { distance * std::cos(angle), distance * std::sin(angle), 0.0 };
}

}

// -------------------- 角度函数本身 --------------------

TEST(compute_angle2cam_x, on_axis_is_zero) {
    const auto cam = camera_at_origin();
    EXPECT_NEAR(compute_angle2cam_x(cam, Point3d { 20.0, 0.0, 0.0 }), 0.0, 1e-9);
}

TEST(compute_angle2cam_x, matches_known_angle) {
    const auto cam = camera_at_origin();
    const auto angle = degrees(5.0);
    EXPECT_NEAR(compute_angle2cam_x(cam, point_at(20.0, angle)), angle, 1e-9);
}

TEST(compute_angle2cam_x, counts_vertical_offset_too) {
    const auto cam = camera_at_origin();
    const auto angle = degrees(10.0);
    // 同样的偏角，但偏在竖直方向上
    const auto point = Point3d { 20.0 * std::cos(angle), 0.0, 20.0 * std::sin(angle) };
    EXPECT_NEAR(compute_angle2cam_x(cam, point), angle, 1e-9);
}

// -------------------- 角度与距离脱钩 --------------------

TEST(compute_angle2cam_x, same_angle_gives_same_score_at_any_distance) {
    const auto cam   = camera_at_origin();
    const auto angle = degrees(10.0);

    EXPECT_NEAR(compute_angle2cam_x(cam, point_at(4.0, angle)), angle, 1e-9);
    EXPECT_NEAR(compute_angle2cam_x(cam, point_at(20.0, angle)), angle, 1e-9);
}

TEST(compute_distance2cam_x, same_angle_gives_different_score_at_any_distance) {
    const auto cam   = camera_at_origin();
    const auto angle = degrees(10.0);

    // 旧的算米方式：同一偏角，远近不同的两个目标得分不同
    EXPECT_LT(
        compute_distance2cam_x(cam, point_at(4.0, angle)),
        compute_distance2cam_x(cam, point_at(20.0, angle)));
}

// -------------------- 改造前后的选择差异 --------------------

TEST(target_selection, angle_picks_the_one_nearer_to_crosshair) {
    const auto cam = camera_at_origin();

    // A 在 20 米外偏 5 度，B 在 4 米外偏 15 度
    const auto a_angle = degrees(5.0);
    const auto b_angle = degrees(15.0);
    const auto a       = point_at(20.0, a_angle);
    const auto b       = point_at(4.0, b_angle);

    // 新算法（算角度）选 A，也就是离准星更近的那个
    EXPECT_LT(compute_angle2cam_x(cam, a), compute_angle2cam_x(cam, b));

    // 旧算法（算米）选 B
    EXPECT_GT(compute_distance2cam_x(cam, a), compute_distance2cam_x(cam, b));
}

TEST(target_selection, distance_is_no_longer_baked_into_the_score) {
    const auto cam = camera_at_origin();

    // 同一个目标，只把距离拉远，偏角不变
    const auto angle = degrees(5.0);
    const auto near_point = point_at(4.0, angle);
    const auto far_point  = point_at(20.0, angle);

    // 角度不变
    EXPECT_NEAR(
        compute_angle2cam_x(cam, near_point), compute_angle2cam_x(cam, far_point), 1e-9);

    // 米数变了 5 倍
    EXPECT_NEAR(
        compute_distance2cam_x(cam, far_point) / compute_distance2cam_x(cam, near_point), 5.0,
        1e-9);
}
