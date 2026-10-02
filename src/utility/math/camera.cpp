#define OPENCV_DISABLE_EIGEN_TENSOR_SUPPORT

#include "camera.hpp"

#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include "utility/math/conversion.hpp"

namespace rmcs::util {

auto CameraFeature::from(std::array<double, 9> input) -> void {
    for (std::size_t i = 0; i < input.size(); ++i) {
        camera_matrix[i / 3][i % 3] = input[i];
    }
}

auto CameraFeature::from(std::array<double, 5> input) -> void { distort_coeff = input; }

auto CameraFeature::intrinsic() const -> cv::Mat {
    return cv::Mat(3, 3, CV_64F, const_cast<double*>(camera_matrix[0].data())).clone();
}

auto CameraFeature::distortion() const -> cv::Mat {
    return cv::Mat(1, 5, CV_64F, const_cast<double*>(distort_coeff.data())).clone();
}

auto CameraFeature::cv_orientation() const -> cv::Mat {
    auto q_ros = orientation.make<Eigen::Quaterniond>();
    auto r_ros = q_ros.toRotationMatrix();
    auto r_ocv = ros2opencv_rotation(r_ros);

    cv::Mat result(3, 3, CV_64F);
    cv::eigen2cv(r_ocv, result);
    return result;
}

auto CameraFeature::cv_translation() const -> cv::Vec3d {
    auto t_ros = translation.make<Eigen::Vector3d>();
    auto t_ocv = ros2opencv_position(t_ros);
    return { t_ocv[0], t_ocv[1], t_ocv[2] };
}

auto compute_distance2cam_x(const Transform& cam, const Point3d& point) -> double {
    const auto ct = cam.translation.make<Eigen::Vector3d>();
    const auto cq = cam.orientation.make<Eigen::Quaterniond>();
    const auto pt = point.make<Eigen::Vector3d>();

    const auto to_target = pt - ct;
    const auto q_inv     = cq.inverse();
    const auto in_cam    = q_inv * to_target;

    return std::sqrt(in_cam.y() * in_cam.y() + in_cam.z() * in_cam.z());
}

// 计算目标中心点到相机光轴的偏离角度
auto compute_angle2cam_x(const Transform& cam, const Point3d& point) -> double {
    const auto ct = cam.translation.make<Eigen::Vector3d>();
    const auto cq = cam.orientation.make<Eigen::Quaterniond>();
    const auto pt = point.make<Eigen::Vector3d>();

    const auto to_target = pt - ct;
    const auto q_inv     = cq.inverse();
    const auto in_cam    = q_inv * to_target;

    return std::atan2(std::hypot(in_cam.y(), in_cam.z()), in_cam.x());
}

// 计算装甲板板面外法向与「板心 → 观察位置」连线的夹角
auto compute_armor_facing(const Translation& eye, const Armor3d& armor) -> double {
    const auto eye_position = eye.make<Eigen::Vector3d>();
    const auto position     = armor.translation.make<Eigen::Vector3d>();

    // 约定：orientation 的 X 轴 = 板心 → 旋转中心，取反得到板面外法向
    const auto normal = armor.orientation.make<Eigen::Quaterniond>() * -Eigen::Vector3d::UnitX();

    const auto to_eye = (eye_position - position).normalized();

    return std::acos(std::clamp(normal.dot(to_eye), -1.0, 1.0));
}
}
