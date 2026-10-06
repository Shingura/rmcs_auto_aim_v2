#pragma once

namespace rmcs::util {

/// @brief 二维轴对齐包围盒，单位为像素
struct BoundingBox2d {
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
};

/// @brief 判断包围盒是否离图像四边都不小于 margin
///
/// @NOTE:
///  贴在画面边缘的装甲板可能被裁掉一部分，此时检测到的角点不再是真实角点，
///  PnP 会解出错误的三维位置，进而污染跟踪器的滤波器。
///
/// @param box          待判断的包围盒
/// @param image_width  图像宽；<= 0 表示尺寸未知
/// @param image_height 图像高；<= 0 表示尺寸未知
/// @param margin       允许的最小边距
/// @return 尺寸未知时无法判断，一律放行
constexpr auto within_image_margin(
    const BoundingBox2d& box, double image_width, double image_height, double margin) noexcept
    -> bool {
    if (image_width <= 0.0 || image_height <= 0.0) return true;

    return box.min_x >= margin && box.max_x <= image_width - margin && box.min_y >= margin
        && box.max_y <= image_height - margin;
}

}
