#include "tracker.hpp"

#include "module/tracker/model/outpost.hpp"
#include "module/tracker/model/robot.hpp"
#include "module/tracker/model/rune.hpp"
#include "module/tracker/selection.hpp"
#include "utility/image/frame_bounds.hpp"
#include "utility/logging/printer.hpp"
#include "utility/math/angle.hpp"
#include "utility/math/camera.hpp"
#include "utility/robot/priority.hpp"
#include "utility/serializable.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <ranges>
#include <string>
#include <unordered_map>

using namespace rmcs::kernel;
using namespace rmcs::util;

struct Tracker::Impl {
    // 哨兵和其他兵种分开使用兵种优先级表
    PriorityMode priority_autonomous;
    PriorityMode priority_teleoperated;
    // 当前生效的表
    PriorityMode priority_table;

    struct Config : Serializable {
        std::string fallback_color         = "RED";
        double observation_timeout_seconds = 1.5;
        double image_margin                = 20.0;
        double fire_timeout_seconds        = 1.0;

        // 换人所需的分数差余量，配置里写度，构造里转成弧度
        double switch_margin = 0.0;

        std::map<std::string, double> priority_autonomous;
        std::map<std::string, double> priority_teleoperated;

        static constexpr std::tuple metas {
            // clang-format off
            &Config::fallback_color, "fallback_color",
            &Config::observation_timeout_seconds, "observation_timeout_seconds",
            &Config::image_margin, "image_margin",
            &Config::priority_autonomous, "priority_autonomous",
            &Config::priority_teleoperated, "priority_teleoperated",
            &Config::fire_timeout_seconds, "fire_timeout_seconds",
            &Config::switch_margin, "switch_margin",
            // clang-format on
        };
    } config;

    struct RobotConfig : RobotModel::Config, Serializable {
        static constexpr std::tuple metas {
            // clang-format off
            &RobotConfig::noise_x, "noise_x",
            &RobotConfig::noise_y, "noise_y",
            &RobotConfig::noise_z, "noise_z",
            &RobotConfig::noise_vx, "noise_vx",
            &RobotConfig::noise_vy, "noise_vy",
            &RobotConfig::noise_vz, "noise_vz",
            &RobotConfig::noise_rotation_angle, "noise_rotation_angle",
            &RobotConfig::noise_rotation_speed, "noise_rotation_speed",
            &RobotConfig::noise_observation, "noise_observation",
            // clang-format on
        };
    } robot_config;

    struct OutpostConfig : OutpostModel::Config, Serializable {
        static constexpr std::tuple metas {
            // clang-format off
            &OutpostConfig::process_noise_xy, "process_noise_xy",
            &OutpostConfig::process_noise_z, "process_noise_z",
            &OutpostConfig::process_noise_speed, "process_noise_speed",
            &OutpostConfig::process_noise_angle, "process_noise_angle",
            &OutpostConfig::observation_noise_xy, "observation_noise_xy",
            &OutpostConfig::observation_noise_z, "observation_noise_z",
            &OutpostConfig::observation_noise_yaw, "observation_noise_yaw",
            &OutpostConfig::plate_switch_yaw_min, "plate_switch_yaw_min",
            // clang-format on
        };
    } outpost_config;

    struct RuneConfig : RuneModel::Config, Serializable {
        static constexpr std::tuple metas {
            // clang-format off
            &RuneConfig::noise_x, "noise_x",
            &RuneConfig::noise_y, "noise_y",
            &RuneConfig::noise_z, "noise_z",
            &RuneConfig::noise_rotation_angle, "noise_rotation_angle",
            &RuneConfig::noise_rotation_speed, "noise_rotation_speed",
            &RuneConfig::noise_face_yaw, "noise_face_yaw",
            &RuneConfig::noise_observation, "noise_observation",
            &RuneConfig::gate_threshold, "gate_threshold",
            &RuneConfig::init_seed_mean_error, "init_seed_mean_error",
            &RuneConfig::init_seed_max_error, "init_seed_max_error",
            &RuneConfig::init_center_gate, "init_center_gate",
            &RuneConfig::init_pitch_bound, "init_pitch_bound",
            &RuneConfig::diverge_face_angle, "diverge_face_angle",
            // clang-format on
        };
    } rune_config;

    struct {
        Armor2ds armor2ds;
        Armor3ds armor3ds;
        Lightbar2ds lightbars;
        std::vector<RuneIcon> icons;
        std::vector<RuneBullseye> bullseyes;
    } stored;

    bool aim_intent      = false;
    bool aim_cleanup     = false;
    bool autonomous_mode = false;
    bool aim_solved      = false;
    // 记录上一次火控成功解算的时间
    Timestamp aim_solved_stamp;

    DeviceIds track_devices = DeviceIds::Full();
    DeviceId track_genre    = DeviceId::UNKNOWN;
    ArmorColor track_color  = ArmorColor::DARK;
    CameraFeature camera;

    // 当前帧的图像尺寸，仅用于判断装甲板是否贴边
    struct {
        double width  = 0.0;
        double height = 0.0;
    } image_size;

    std::unordered_map<DeviceId, Timestamp> robot_stamps;
    std::unordered_map<DeviceId, RobotModel> robot_models;

    Timestamp outpost_stamp;
    std::unique_ptr<OutpostModel> outpost;

    Timestamp rune_stamp, rune_corrected_stamp;
    std::unique_ptr<RuneModel> rune;

    Printer logging { "TrackerV2" };
    Addition addition;

    explicit Impl(const YAML::Node& yaml) {
        auto compat_yaml = YAML::Clone(yaml);
        if (!compat_yaml["fallback_color"] && compat_yaml["enemy_color"]) {
            compat_yaml["fallback_color"] = compat_yaml["enemy_color"];
        }

        if (auto ret = config.serialize(compat_yaml); !ret) {
            logging.error("TrackerV2 初始化错误: {}", ret.error());
            throw std::runtime_error { "无法构造 TrackerV2" };
        }

        config.switch_margin = util::deg2rad(config.switch_margin);

        // 从 yaml 读取目标优先级，写入 priority_table
        const auto fill = [&](const std::map<std::string, double>& source, PriorityMode& target) {
            for (const auto& [name, value] : source) {
                const auto id = rmcs::from_string(name);
                if (id == DeviceId::UNKNOWN) {
                    logging.error("未知的目标类型：{}", name);
                    continue;
                }
                target[id] = value;
            }
        };

        fill(config.priority_autonomous, priority_autonomous);
        fill(config.priority_teleoperated, priority_teleoperated);
        // 默认为有人操控车，即无兵种偏好
        priority_table = priority_teleoperated;

        if (auto ret = robot_config.serialize(yaml["robot"]); !ret) {
            logging.error("RobotModel config error: {}", ret.error());
            throw std::runtime_error { "无法构造 RobotModel Config" };
        }
        if (auto ret = outpost_config.serialize(yaml["outpost"]); !ret) {
            logging.error("OutpostModel config error: {}", ret.error());
            throw std::runtime_error { "无法构造 OutpostModel Config" };
        }
        if (const auto rune_node = yaml["rune"]; rune_node && !rune_node.IsNull()) {
            if (auto ret = rune_config.serialize(rune_node); !ret) {
                logging.error("RuneModel config error: {}", ret.error());
                throw std::runtime_error { "无法构造 RuneModel Config" };
            }
        }

        /*^^*/ if (config.fallback_color == "RED") {
            track_color = ArmorColor::RED;
        } else if (config.fallback_color == "BLUE") {
            track_color = ArmorColor::BLUE;
        } else {
            logging.error("invalid color {}", config.fallback_color);
        }
    }

    // 根据 autonomous_mode 决定使用表
    auto update_autonomous_mode(bool on) -> void {
        autonomous_mode = on;
        priority_table  = on ? priority_autonomous : priority_teleoperated;
    }

    auto update_aim_solved(bool solved) -> void { aim_solved = solved; }
    auto clean() noexcept {
        stored.armor2ds.clear();
        stored.armor3ds.clear();
        stored.lightbars.clear();
        stored.icons.clear();
        stored.bullseyes.clear();
    }

    auto store(std::span<const Armor2d> items) {
        for (const auto& item : items) {
            if (item.color == track_color && track_devices.contains(item.genre)) {
                const auto min_x = std::min({ item.tl.x, item.tr.x, item.bl.x, item.br.x });
                const auto max_x = std::max({ item.tl.x, item.tr.x, item.bl.x, item.br.x });
                const auto min_y = std::min({ item.tl.y, item.tr.y, item.bl.y, item.br.y });
                const auto max_y = std::max({ item.tl.y, item.tr.y, item.bl.y, item.br.y });

                // 贴边的装甲板可能被裁掉一部分，角点不再是真实角点，不能进滤波器
                const auto box = BoundingBox2d { min_x, max_x, min_y, max_y };
                if (!within_image_margin(
                        box, image_size.width, image_size.height, config.image_margin))
                    continue;

                stored.armor2ds.push_back(item);
            }
        }
    }
    auto store(std::span<const Armor3d> items) {
        for (const auto& item : items) {
            if (item.color == track_color && track_devices.contains(item.genre)) {
                stored.armor3ds.push_back(item);
            }
        }
    }
    auto store(std::span<const Lightbar2d> items) {
        for (const auto& item : items) {
            if (item.color == track_color && track_devices.contains(item.genre)) {
                stored.lightbars.push_back(item);
            }
        }
    }
    auto store(std::span<const RuneIcon> items) {
        if (!track_devices.contains(DeviceId::RUNE)) return;
        std::ranges::copy(items, std::back_inserter(stored.icons));
    }
    auto store(std::span<const RuneBullseye> items) {
        if (!track_devices.contains(DeviceId::RUNE)) return;
        std::ranges::copy(items, std::back_inserter(stored.bullseyes));
    }

    auto execute(Timestamp timestamp) {
        addition.tracked2d.clear();
        addition.tracked3d.clear();
        addition.lightbars.clear();
        addition.rune_features.clear();
        addition.rune_polygon.reset();
        addition.infos.clear();

        { // 前哨站观测超时
            const auto dt = std::chrono::duration<double> {
                timestamp - outpost_stamp,
            };
            if (dt.count() > config.observation_timeout_seconds) {
                outpost       = nullptr;
                outpost_stamp = timestamp;

                if (track_genre == DeviceId::OUTPOST && aim_intent && aim_cleanup) {
                    track_genre = DeviceId::UNKNOWN;
                }
            }
        }
        { // 大符超时处理
            if (rune != nullptr) {
                const auto dt = std::chrono::duration<double> {
                    timestamp - rune_corrected_stamp,
                };
                if (dt.count() > config.observation_timeout_seconds) {
                    rune = nullptr;

                    if (track_genre == DeviceId::RUNE && aim_intent && aim_cleanup) {
                        track_genre = DeviceId::UNKNOWN;
                    }
                }
            }
        }
        // 机器人观测超时处理
        std::erase_if(robot_stamps, [&](const auto& item) {
            const auto [id, stamp] = item;

            const auto dt = std::chrono::duration<double> { timestamp - stamp };
            if (dt.count() > config.observation_timeout_seconds) {
                robot_models.erase(id);

                if (track_genre == id && aim_intent && aim_cleanup) {
                    track_genre = DeviceId::UNKNOWN;
                }
                return true;
            }
            return false;
        });
        struct Target final {
            std::vector<Armor2d> armor2ds;
            std::vector<Armor3d> armor3ds;
            std::vector<Lightbar2d> bars;
        };
        auto seen = std::unordered_map<DeviceId, Target> { };

        for (const auto& armor : stored.armor2ds) {
            seen[armor.genre].armor2ds.push_back(armor);
        }
        for (const auto& armor : stored.armor3ds) {
            seen[armor.genre].armor3ds.push_back(armor);
        }
        for (const auto& bar : stored.lightbars) {
            seen[bar.genre].bars.push_back(bar);
        }

        { // 迭代前哨站 Model
            constexpr auto kId = DeviceId::OUTPOST;
            if (seen.contains(kId)) {
                const auto& armors = seen[kId].armor3ds;
                /// @NOTE:
                ///  前哨站为单装甲板输入(120 度，也只能看到一块)，邻侧灯条的信息
                ///  已经在距离优化那个步骤消费掉了，所以这里不需要传入，有时间这
                ///  里也会换成纯 2d 观测，到时候就完全不需要 Armor3d 了
                if (!armors.empty()) {
                    if (outpost == nullptr) {
                        outpost = std::make_unique<OutpostModel>(armors.front());
                        outpost->configure(outpost_config);
                    } else {
                        const auto dt = std::chrono::duration<double> {
                            timestamp - outpost_stamp,
                        };
                        outpost->predict(dt.count());
                        outpost->correct(armors.front());

                        if (outpost->diverged()) {
                            outpost = nullptr;
                            logging.warn("{} is diverged", get_enum_name(kId));
                        }
                    }
                    outpost_stamp = timestamp;
                }
            }
        }
        { // 迭代大符
            if (!stored.icons.empty() || !stored.bullseyes.empty()) {
                if (rune == nullptr) {
                    auto model = std::make_unique<RuneModel>(rune_config);
                    model->update_camera(std::bit_cast<std::array<double, 9>>(camera.camera_matrix),
                        camera.distort_coeff);
                    model->update_transform({
                        .translation = camera.translation,
                        .orientation = camera.orientation,
                    });
                    if (model->init(stored.icons, stored.bullseyes, timestamp)) {
                        rune                 = std::move(model);
                        rune_stamp           = timestamp;
                        rune_corrected_stamp = timestamp;
                        logging.info("Init OK with {}", get_enum_name(DeviceId::RUNE));
                    }
                } else {
                    const auto dt = std::chrono::duration<double> {
                        timestamp - rune_stamp,
                    };

                    rune->update_transform({
                        .translation = camera.translation,
                        .orientation = camera.orientation,
                    });
                    rune->predict(dt.count(), timestamp);
                    rune_stamp           = timestamp;
                    const auto corrected = rune->correct(stored.icons, stored.bullseyes);

                    if (rune->diverged()) {
                        rune = nullptr;
                        logging.warn("{} is diverged", get_enum_name(DeviceId::RUNE));
                    } else if (corrected) {
                        rune_corrected_stamp = timestamp;
                    }
                }
            }
        }
        // 迭代机器人 Model
        for (const auto& [id, target] : seen) {
            if (id == DeviceId::OUTPOST) {
                continue;
            }
            if (robot_models.contains(id) == false) {
                robot_models.try_emplace(id, robot_config);
                robot_models[id].update_camera(
                    std::bit_cast<std::array<double, 9>>(camera.camera_matrix),
                    camera.distort_coeff);
                robot_models[id].update_transform({
                    .translation = camera.translation,
                    .orientation = camera.orientation,
                });
                // 模型初始化失败时处理
                if (!robot_models[id].init(target.armor2ds)) {
                    robot_models.erase(id);
                    // 移除了 robot_stamps.erase(id); 防止超时清理失效
                    continue;
                } else {
                    logging.info("Init OK with {}", get_enum_name(id));
                }
            } else {
                const auto dt = std::chrono::duration<double> {
                    timestamp - robot_stamps[id],
                };

                auto& model = robot_models[id];
                model.update_transform({
                    .translation = camera.translation,
                    .orientation = camera.orientation,
                });
                model.predict(dt.count());
                model.correct(target.armor2ds, target.bars);
                // 模型发散时处理
                if (model.diverged()) {
                    robot_models.erase(id);
                    // 移除了 robot_stamps.erase(id); 防止超时清理失效
                    logging.warn("{} is diverged", get_enum_name(id));
                    continue;
                }
            }
            robot_stamps[id] = timestamp;
        }

        // 选择目标并填充调试信息
        // 除哨兵以外的兵种，只启用偏离角度
        const auto calculate = [&](DeviceId id, const Point3d& center,
                                   std::span<const Armor3d> armors = { }) -> double {
            // 1. 使用代表点确定偏离角度
            const auto deviation =
                compute_angle2cam_x({ camera.translation, camera.orientation }, center);

            // 2. 查兵种优先级，表里没有这一项就退回 0（空表等价于不带偏好）
            const auto priority = lookup_priority(priority_table, id);

            // 3. 从候选点中选择最正对相机的板
            auto facing = 0.0;
            if (autonomous_mode && !armors.empty()) {
                facing = std::numeric_limits<double>::max();

                for (const auto& armor : armors) {
                    facing = std::min(facing, compute_armor_facing(camera.translation, armor));
                }
            }
            return deviation + priority + facing;
        };

        const auto locked = aim_intent && track_genre != DeviceId::UNKNOWN;

        auto result = Trackable::Unique { };
        auto better = std::numeric_limits<double>::max();
        auto device = DeviceId::UNKNOWN;

        // 锁定时其他候选里的最好者，用于判断是否存在值得切换的更优目标。
        auto other_score  = std::numeric_limits<double>::max();
        auto other_device = DeviceId::UNKNOWN;
        auto other_result = Trackable::Unique { };

        /// @NOTE:
        ///  锁定期间只有锁定目标参与选择，其他候选只登记分数：哨兵据此判断
        ///  画面里有没有明显更优的目标。返回该候选是否参与选择
        const auto register_candidate = [&](DeviceId id, double score, Timestamp stamp,
                                            const auto& state) -> bool {
            if (locked && id != track_genre) {
                if (score < other_score) {
                    other_score  = score;
                    other_device = id;
                    other_result = make_trackable(stamp, state, id);
                }
                return false;
            }
            if (better > score) {
                better = score;
                result = make_trackable(stamp, state, id);

                device = id;
            }
            return true;
        };

        {
            if (outpost && outpost->converge()) {
                const auto state  = outpost->state();
                const auto armors = outpost->full();
                const auto score  = calculate(DeviceId::OUTPOST, state.get_direction(), armors);

                if (register_candidate(DeviceId::OUTPOST, score, outpost_stamp, state)) {
                    std::ranges::copy(armors, std::back_inserter(addition.tracked3d));

                    const auto a = state.rotation_angle;
                    const auto v = state.rotation_speed;
                    addition.infos.push_back({
                        .text  = std::format("a: {:+.1f} | v: {:+2.2f}", a, v),
                        .point = Point3d { state.x, state.y, state.z },
                    });
                }
            }
            if (rune && rune->converge()) {
                const auto state = rune->state();
                // 大符的五片符叶都在同一个平面上，不需要考虑法向
                const auto score = calculate(DeviceId::RUNE, state.get_direction());

                if (register_candidate(DeviceId::RUNE, score, rune_stamp, state)) {
                    std::ranges::copy(
                        rune->addition().predicted | std::views::transform([](const auto& item) {
                            return Addition::RuneFeature { item.feature_id, item.point };
                        }),
                        std::back_inserter(addition.rune_features));

                    if (rune->addition().predicted.size() == 6) {
                        auto polygon = Addition::RunePolygon { };
                        auto ok      = true;
                        for (const auto& item : rune->addition().predicted) {
                            if (item.feature_id == 0) {
                                polygon.icon = item.point;
                            } else if (item.feature_id >= 1 && item.feature_id <= 5) {
                                polygon.blades[static_cast<std::size_t>(item.feature_id - 1)] =
                                    item.point;
                            } else {
                                ok = false;
                            }
                        }
                        if (ok) addition.rune_polygon = polygon;
                    }

                    const auto a = state.rotation_angle;
                    const auto v = state.rotation_speed;

                    const auto text_large_rune = [&] {
                        return std::format(
                            "spd_{}(t)={:+.2f}{:+.2f}*sin({:+.2f}{:+.2f}t), e={:.3f}",
                            state.update_count, state.sine_v, state.sine_a, state.sine_phase,
                            state.sine_omega, state.prediction_cost);
                    };
                    const auto text_small_rune = [&] {
                        return std::format("spd_{}(t)={:+.2f}, e={:.3f}", state.update_count, v,
                            state.prediction_cost);
                    };
                    const auto text_fallback = [&] { return std::format("theta_ekf={:+.2f}", a); };

                    addition.infos.push_back({
                        .text  = state.sine_valid
                            ? text_large_rune()
                            : (state.use_prediction_speed ? text_small_rune() : text_fallback()),
                        .point = Point3d { state.x, state.y, state.z },
                    });
                }
            }
            for (const auto& [id, model] : robot_models) {
                if (!model.converge()) continue;

                const auto state = model.state();
                const auto score = calculate(id, state.get_direction(), model.full());

                // 锁定时，不回传其他的目标，只登记分数供换人判断
                if (!register_candidate(id, score, robot_stamps.at(id), state)) continue;

                std::ranges::copy( // Armor 2d
                    model.addition().armors, std::back_inserter(addition.tracked2d));
                std::ranges::copy( // Armor 3d
                    model.full(), std::back_inserter(addition.tracked3d));
                std::ranges::copy(
                    model.addition().tracked | std::views::transform([](const auto& item) {
                        return Addition::Lightbar { item.lightbar_id, item.point };
                    }),
                    std::back_inserter(addition.lightbars));

                const auto rv = state.rotation_speed;
                const auto vx = state.vx;
                const auto vy = state.vy;
                addition.infos.push_back({
                    .text  = std::format("rv: {:+2.2f} | v: {:+2.2f}, {:+2.2f}", rv, vx, vy),
                    .point = Point3d { state.x, state.y, state.z },
                });
            }
        }

        { // 哨兵：连续一段时间没有可执行的瞄准，且画面里还有明显更优的目标时，放弃当前锁定
            if (autonomous_mode && aim_intent && track_genre != DeviceId::UNKNOWN) {
                if (aim_solved) {
                    aim_solved_stamp = timestamp;
                } else {
                    const auto dt = std::chrono::duration<double> { timestamp - aim_solved_stamp };
                    // 只有登记到更优目标才会解锁，所以这里一定有目标可以换过去，
                    // 直接在本帧完成切换：locked 是解锁前的旧值，下面的更新分支
                    // 不会覆盖这次切换，火控本帧就能拿到新目标
                    if (should_unlock(dt.count(), config.fire_timeout_seconds, better, other_score,
                            config.switch_margin)) {
                        track_genre      = other_device;
                        result           = std::move(other_result);
                        aim_solved_stamp = timestamp;
                    }
                }
            }
        }

        /// @NOTE:
        ///  未锁定时更新目标；一旦自瞄意图按下且已有锁定目标，就保持该目标
        ///  最高优先级，即使目标暂时丢失。开启锁定超时清理时，目标超时后会
        ///  解除锁定并允许重新选择
        if (!locked) {
            // 对于哨兵，记录火控解算的时间
            if (autonomous_mode && track_genre != device) {
                aim_solved_stamp = timestamp;
            }
            track_genre = device;
        }
        return result;
    }
};

Tracker::Tracker(const YAML::Node& yaml)
    : pimpl { std::make_unique<Impl>(yaml) } { }

Tracker::~Tracker() noexcept = default;

auto Tracker::update_aim_intent(bool intent) -> void { pimpl->aim_intent = intent; }
auto Tracker::update_aim_cleanup(bool on) -> void { pimpl->aim_cleanup = on; }
auto Tracker::update_autonomous_mode(bool on) -> void { pimpl->update_autonomous_mode(on); }
auto Tracker::update_aim_solved(bool solved) -> void { pimpl->aim_solved = solved; }

auto Tracker::update_track_color(CampColor camp) -> void {
    /*^^*/ if (camp == CampColor::RED) {
        pimpl->track_color = ArmorColor::RED;
    } else if (camp == CampColor::BLUE) {
        pimpl->track_color = ArmorColor::BLUE;
    }
    // Do nothing for unknown camp
}
auto Tracker::update_track_genre(DeviceIds ids) -> void { pimpl->track_devices = ids; }

auto Tracker::update_camera(const Transform& t) noexcept -> void {
    pimpl->camera.translation = t.translation;
    pimpl->camera.orientation = t.orientation;
}
auto Tracker::update_camera(const std::array<double, 9>& param) noexcept -> void {
    pimpl->camera.from(param);
}
auto Tracker::update_camera(const std::array<double, 5>& param) noexcept -> void {
    pimpl->camera.from(param);
}

auto Tracker::update_image_size(double width, double height) noexcept -> void {
    pimpl->image_size = { width, height };
}

auto Tracker::clean() noexcept -> void { pimpl->clean(); }

auto Tracker::store(std::span<const Armor2d> item) -> void { pimpl->store(item); }
auto Tracker::store(std::span<const Armor3d> item) -> void { pimpl->store(item); }
auto Tracker::store(std::span<const Lightbar2d> item) -> void { pimpl->store(item); }
auto Tracker::store(std::span<const RuneIcon> item) -> void { pimpl->store(item); }
auto Tracker::store(std::span<const RuneBullseye> item) -> void { pimpl->store(item); }

auto Tracker::execute(Timestamp stamp) -> Trackable::Unique { return pimpl->execute(stamp); }

auto Tracker::addition() const -> const Addition& { return pimpl->addition; }
