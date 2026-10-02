#include "utility/math/camera.hpp"
#include "utility/robot/priority.hpp"

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

// -------------------- 兵种优先级 --------------------

TEST(lookup_priority, returns_the_weight_when_present) {
    const auto table = PriorityMode {
        { DeviceId::HERO, 0.0 },
        { DeviceId::ENGINEER, 0.4 },
    };

    EXPECT_DOUBLE_EQ(lookup_priority(table, DeviceId::HERO), 0.0);
    EXPECT_DOUBLE_EQ(lookup_priority(table, DeviceId::ENGINEER), 0.4);
}

TEST(lookup_priority, falls_back_to_zero_when_missing) {
    const auto table = PriorityMode { { DeviceId::HERO, 0.0 } };

    // 表里没登记的兵种拿不到偏好
    EXPECT_DOUBLE_EQ(lookup_priority(table, DeviceId::SENTRY), 0.0);

    // 空表是有人车的形态：所有兵种都退回 0，等于这一项不参与打分
    const auto empty = PriorityMode { };
    EXPECT_DOUBLE_EQ(lookup_priority(empty, DeviceId::HERO), 0.0);
    EXPECT_DOUBLE_EQ(lookup_priority(empty, DeviceId::SENTRY), 0.0);
}

// -------------------- 装甲板朝向 --------------------

namespace {

// 板放在 (x, y, z)，orientation 是绕 Z 轴转 yaw。
// 按项目约定，orientation 的 X 轴指向「板心 → 旋转中心」，所以板面外法向是它的反向。
auto armor_at(double x, double y, double z, double yaw) -> Armor3d {
    return Armor3d {
        .genre       = DeviceId::INFANTRY_3,
        .color       = ArmorColor::RED,
        .id          = 0,
        .translation = Translation { x, y, z },
        .orientation = Orientation { 0.0, 0.0, std::sin(yaw / 2.0), std::cos(yaw / 2.0) },
    };
}

}

TEST(compute_armor_facing, plate_facing_the_observer_is_zero) {
    // 相机在原点，板在 +Y 方向 5 米处。绕 Z 转 90 度后板面朝向 -Y，正对相机
    const auto armor = armor_at(0.0, 5.0, 0.0, kPi / 2.0);

    EXPECT_NEAR(compute_armor_facing(Translation { 0.0, 0.0, 0.0 }, armor), 0.0, 1e-9);
}

TEST(compute_armor_facing, plate_side_on_is_a_right_angle) {
    // 板面朝 +X，与「板心 → 相机」的连线垂直
    const auto armor = armor_at(0.0, 5.0, 0.0, kPi);

    EXPECT_NEAR(compute_armor_facing(Translation { 0.0, 0.0, 0.0 }, armor), kPi / 2.0, 1e-9);
}

TEST(compute_armor_facing, plate_facing_away_is_a_straight_angle) {
    const auto armor = armor_at(0.0, 5.0, 0.0, -kPi / 2.0);

    EXPECT_NEAR(compute_armor_facing(Translation { 0.0, 0.0, 0.0 }, armor), kPi, 1e-9);
}

TEST(compute_armor_facing, the_most_facing_plate_gives_the_smallest_angle) {
    const auto eye = Translation { 0.0, 0.0, 0.0 };

    // 同一台车上的三块板，朝向依次是 正对 / 侧对 / 背对
    const auto front = armor_at(0.0, 5.0, 0.0, kPi / 2.0);
    const auto side  = armor_at(0.0, 5.0, 0.0, kPi);
    const auto back  = armor_at(0.0, 5.0, 0.0, -kPi / 2.0);

    // 取最小值就是取最正对的那块板，这是 calculate 里挑板的依据
    EXPECT_LT(compute_armor_facing(eye, front), compute_armor_facing(eye, side));
    EXPECT_LT(compute_armor_facing(eye, side), compute_armor_facing(eye, back));
}

TEST(compute_armor_facing, grows_when_the_observer_moves_sideways) {
    const auto armor = armor_at(0.0, 5.0, 0.0, kPi / 2.0);

    // 同一块板，相机从正前方挪到侧面，看到的夹角变大
    EXPECT_LT(
        compute_armor_facing(Translation { 0.0, 0.0, 0.0 }, armor),
        compute_armor_facing(Translation { 4.0, 0.0, 0.0 }, armor));
}
