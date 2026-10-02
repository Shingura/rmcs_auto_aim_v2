#pragma once

#include <limits>

namespace rmcs {

/// @brief 是否放弃当前锁定的目标
///
/// 在同时满足下列两种情况时自动解锁：
///   1. 火控解算超时；
///   2. 存在更优目标，且分数差大于 switch_margin
///
/// @param unsolved_seconds 火控解算失败时间（秒）
/// @param give_up_seconds  放弃所需的持续时间（秒）
/// @param locked_score     当前锁定目标的分数；解算失败时为 max
/// @param other_score      其他候选里最好的分数；没有其他候选时为 max
/// @param switch_margin    分数差阈值
inline auto should_unlock(double unsolved_seconds, double give_up_seconds, double locked_score,
    double other_score, double switch_margin) noexcept -> bool {
    if (unsolved_seconds <= give_up_seconds) return false;

    return other_score + switch_margin < locked_score;
}

}
