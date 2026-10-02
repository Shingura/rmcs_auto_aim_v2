#pragma once

#include "utility/robot/id.hpp"
#include <unordered_map>

// 存放不同目标的优先级
namespace rmcs {

using PriorityMode = std::unordered_map<DeviceId, double>;

// 取该兵种的优先级权重；表里没有这一项时退回 0，空表即等价于不带偏好
inline auto lookup_priority(const PriorityMode& table, DeviceId id) noexcept -> double {
    const auto it = table.find(id);
    return (it != table.end()) ? it->second : 0.0;
}

}
