#include "servo_backends/feetech_scs_driver.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "feetech_scs_ros2_control/scs_params.hpp"
#include "feetech_scs_ros2_control/scs_register_map.hpp"
#include "servo_core/registry.hpp"

namespace servo_backends {
    namespace {

        namespace scs = feetech_scs_ros2_control;
        namespace core = servo_core;

        // SCS 系列最大有效角度（内存表“最大有效角度”），机械行程上限。
        constexpr double kMaxRangeDeg = 300.0;

        int clamp_ticks(const int ticks, const ServoConfig& config) {
            const int lower = std::max(0, static_cast<int>(std::lround(config.min_ticks)));
            const int upper = std::min(static_cast<int>(scs::kPositionTicks) - 1,
                                       static_cast<int>(std::lround(config.max_ticks)));
            return std::clamp(ticks, lower, upper);
        }

        core::ParamAccess to_access(const scs::ParamAccess access) {
            switch (access) {
            case scs::ParamAccess::kReadOnly:
                return core::ParamAccess::kReadOnly;
            case scs::ParamAccess::kRam:
                return core::ParamAccess::kRam;
            case scs::ParamAccess::kEeprom:
                return core::ParamAccess::kEeprom;
            }
            return core::ParamAccess::kRam;
        }

        core::ParamArea to_area(const scs::ParamArea area) {
            switch (area) {
            case scs::ParamArea::kEeprom:
                return core::ParamArea::kEeprom;
            case scs::ParamArea::kSram:
                return core::ParamArea::kSram;
            case scs::ParamArea::kDefault:
                return core::ParamArea::kDefault;
            }
            return core::ParamArea::kSram;
        }

        // 驱动侧 Result → 中间层 Result。
        core::Result to_result(const scs::Result& result) {
            return result ? core::Result::success() : core::Result::failure(result.error());
        }

    } // namespace

    FeetechScsDriver::~FeetechScsDriver() {
        close();
    }

    Result FeetechScsDriver::open(const core::BusConfig& config) {
        close();
        if (const auto result = port_.open(config.port, config.baud_rate); !result) {
            return Result::failure(result.error());
        }
        protocol_ = std::make_unique<scs::ScsProtocol>(
            port_, std::chrono::milliseconds(config.response_timeout_ms));
        if (trace_hook_) {
            protocol_->set_trace_hook(
                [this](const char* direction, const std::span<const uint8_t> frame) {
                    trace_hook_(direction, std::vector<uint8_t>(frame.begin(), frame.end()));
                });
        }
        return Result::success();
    }

    void FeetechScsDriver::set_trace_hook(TraceHook hook) {
        trace_hook_ = std::move(hook);
        if (protocol_) {
            if (trace_hook_) {
                protocol_->set_trace_hook(
                    [this](const char* direction, const std::span<const uint8_t> frame) {
                        trace_hook_(direction, std::vector<uint8_t>(frame.begin(), frame.end()));
                    });
            }
            else {
                protocol_->set_trace_hook(nullptr);
            }
        }
    }

    void FeetechScsDriver::close() {
        protocol_.reset();
        port_.close();
    }

    bool FeetechScsDriver::is_open() const {
        return port_.is_open();
    }

    Result FeetechScsDriver::set_servo_config(const ServoConfig& config) {
        if (config.range_deg <= 0.0) {
            return Result::failure("range_deg must be positive");
        }
        if (config.range_deg > kMaxRangeDeg) {
            return Result::failure("range_deg must not exceed " + std::to_string(kMaxRangeDeg) +
                                   " for SCS servos");
        }
        if (config.min_ticks > config.max_ticks) {
            return Result::failure("min_ticks must not exceed max_ticks");
        }
        configs_[config.id] = config;
        return Result::success();
    }

    const ServoConfig& FeetechScsDriver::config_for(const ServoId id) const {
        const static ServoConfig kDefault{};
        const auto it = configs_.find(id);
        return it == configs_.end() ? kDefault : it->second;
    }

    ServoInfo FeetechScsDriver::to_info(const ServoId id,
                                        const scs::ScsServoIdentity& identity) {
        ServoInfo info;
        info.id = id;
        info.model_number = identity.model;
        info.model = std::string(scs::model_name(identity.model));
        info.firmware_major = identity.firmware_major;
        info.firmware_minor = identity.firmware_minor;
        info.max_range_deg = kMaxRangeDeg;
        return info;
    }

    Expected<std::vector<ServoInfo>> FeetechScsDriver::scan(const int first_id, const int last_id) {
        if (!protocol_) {
            return Expected<std::vector<ServoInfo>>::failure("Serial port is not open");
        }
        const auto ids = protocol_->scan(static_cast<uint8_t>(first_id), static_cast<uint8_t>(last_id));
        if (!ids) {
            return Expected<std::vector<ServoInfo>>::failure(ids.error());
        }
        std::vector<ServoInfo> infos;
        infos.reserve(ids.value().size());
        for (const auto id : ids.value()) {
            const auto identity = protocol_->read_identity(id);
            if (identity) {
                infos.push_back(to_info(id, identity.value()));
            }
            else {
                // PING 有应答但身份读取失败：保留条目，型号标记 unknown。
                ServoInfo info;
                info.id = id;
                info.model = "unknown";
                infos.push_back(info);
            }
        }
        return Expected<std::vector<ServoInfo>>(infos);
    }

    Expected<ServoInfo> FeetechScsDriver::identify(const ServoId id) {
        if (!protocol_) {
            return Expected<ServoInfo>::failure("Serial port is not open");
        }
        const auto identity = protocol_->read_identity(id);
        if (!identity) {
            return Expected<ServoInfo>::failure(identity.error());
        }
        return Expected<ServoInfo>(to_info(id, identity.value()));
    }

    Expected<ServoState> FeetechScsDriver::read_state(const ServoId id) {
        if (!protocol_) {
            return Expected<ServoState>::failure("Serial port is not open");
        }
        const auto raw = protocol_->read_state(id);
        if (!raw) {
            return Expected<ServoState>::failure(raw.error());
        }
        const auto& config = config_for(id);
        ServoState state;
        state.id = id;
        state.position_ticks = raw.value().position_ticks;
        state.position_deg = core::ticks_to_degrees(config, raw.value().position_ticks);
        state.velocity_ticks_s = raw.value().velocity_ticks_per_s;
        state.velocity_deg_s = core::ticks_s_to_deg_s(config, raw.value().velocity_ticks_per_s);
        state.load = raw.value().load;
        state.voltage = raw.value().voltage;
        state.temperature = raw.value().temperature;
        state.current = raw.value().current;
        state.moving = raw.value().moving;
        return Expected<ServoState>(state);
    }

    Result FeetechScsDriver::write_goal(const ServoId id, const int position_ticks, const int time_ms,
                                     const int speed_ticks_s) {
        if (!protocol_) {
            return Result::failure("Serial port is not open");
        }
        const auto position = static_cast<uint16_t>(clamp_ticks(position_ticks, config_for(id)));
        return to_result(protocol_->write_goal_position(id, position,
                                                        static_cast<uint16_t>(std::max(0, time_ms)),
                                                        static_cast<uint16_t>(std::clamp(
                                                            speed_ticks_s, 0, 0xFFFF))));
    }

    Result FeetechScsDriver::sync_write_goals(const std::vector<ServoId>& ids,
                                           const std::vector<int>& positions_ticks,
                                           const int time_ms,
                                           const std::vector<int>& speeds_ticks_s) {
        if (!protocol_) {
            return Result::failure("Serial port is not open");
        }
        if (ids.size() != positions_ticks.size() || ids.size() != speeds_ticks_s.size()) {
            return Result::failure("sync_write_goals requires matching ids, positions and speeds");
        }
        std::vector<uint16_t> positions;
        std::vector<uint16_t> speeds;
        positions.reserve(ids.size());
        speeds.reserve(ids.size());
        for (size_t i = 0; i < ids.size(); ++i) {
            positions.push_back(static_cast<uint16_t>(clamp_ticks(positions_ticks[i], config_for(ids[i]))));
            speeds.push_back(static_cast<uint16_t>(std::clamp(speeds_ticks_s[i], 0, 0xFFFF)));
        }
        return to_result(protocol_->sync_write_goal_positions(
            ids, positions, static_cast<uint16_t>(std::max(0, time_ms)), speeds));
    }

    Result FeetechScsDriver::set_torque(const ServoId id, const bool enabled) {
        if (!protocol_) {
            return Result::failure("Serial port is not open");
        }
        return to_result(protocol_->set_torque_enabled(id, enabled));
    }

    std::vector<ParamDescriptor> FeetechScsDriver::parameters() const {
        std::vector<ParamDescriptor> descriptors;
        descriptors.reserve(scs::kParameterDescriptors.size());
        for (const auto& descriptor : scs::kParameterDescriptors) {
            descriptors.push_back(ParamDescriptor{
                .key = descriptor.key,
                .label = descriptor.label,
                .zh = descriptor.zh,
                .unit = descriptor.unit,
                .access = to_access(descriptor.access),
                .area = to_area(descriptor.area),
                .address = descriptor.address,
                .min = descriptor.min,
                .max = descriptor.max,
                .choices = descriptor.choices != nullptr
                    ? std::vector<int>(descriptor.choices,
                                       descriptor.choices + descriptor.choice_count)
                    : std::vector<int>{},
            });
        }
        return descriptors;
    }

    Expected<int> FeetechScsDriver::read_param(const ServoId id, const std::string& key) {
        if (!protocol_) {
            return Expected<int>::failure("Serial port is not open");
        }
        const auto* descriptor = scs::find_parameter(key);
        if (descriptor == nullptr) {
            return Expected<int>::failure("Unknown parameter '" + key + "'");
        }
        if (descriptor->width == scs::ParamWidth::kWord) {
            const auto word = protocol_->read_word(id, descriptor->address);
            if (!word) {
                return Expected<int>::failure(word.error());
            }
            return Expected<int>(word.value());
        }
        std::array<uint8_t, 1> data{};
        if (const auto result = protocol_->read_registers(id, descriptor->address, data); !result) {
            return Expected<int>::failure(result.error());
        }
        return Expected<int>(data[0]);
    }

    Result FeetechScsDriver::write_param(const ServoId id, const std::string& key, const int value) {
        if (!protocol_) {
            return Result::failure("Serial port is not open");
        }
        const auto* descriptor = scs::find_parameter(key);
        if (descriptor == nullptr) {
            return Result::failure("Unknown parameter '" + key + "'");
        }
        if (descriptor->access == scs::ParamAccess::kReadOnly) {
            return Result::failure("Parameter '" + key + "' is read-only");
        }
        if (value < descriptor->min || value > descriptor->max) {
            return Result::failure("Parameter '" + key + "' value " + std::to_string(value) +
                                   " is outside [" + std::to_string(descriptor->min) + ", " +
                                   std::to_string(descriptor->max) + "]");
        }

        std::array<uint8_t, 2> data{};
        size_t length = 1;
        if (descriptor->width == scs::ParamWidth::kWord) {
            scs::encode_word(data.data(), static_cast<uint16_t>(value));
            length = 2;
        }
        else {
            data[0] = static_cast<uint8_t>(value);
        }

        if (descriptor->access == scs::ParamAccess::kEeprom) {
            return to_result(protocol_->write_eeprom(id, descriptor->address,
                                                     std::span<const uint8_t>(data.data(), length)));
        }
        return to_result(protocol_->write_registers(id, descriptor->address,
                                                    std::span<const uint8_t>(data.data(), length)));
    }

    Expected<std::vector<uint8_t>> FeetechScsDriver::read_registers(const ServoId id,
                                                                 const uint8_t address,
                                                                 const size_t length) {
        if (!protocol_) {
            return Expected<std::vector<uint8_t>>::failure("Serial port is not open");
        }
        if (length == 0 || length > 250) {
            return Expected<std::vector<uint8_t>>::failure("Invalid register read length " +
                                                          std::to_string(length));
        }
        std::vector<uint8_t> data(length);
        if (const auto result = protocol_->read_registers(id, address, data); !result) {
            return Expected<std::vector<uint8_t>>::failure(result.error());
        }
        return Expected<std::vector<uint8_t>>(data);
    }

    Result FeetechScsDriver::write_registers(const ServoId id, const uint8_t address,
                                          const std::vector<uint8_t>& data) {
        if (!protocol_) {
            return Result::failure("Serial port is not open");
        }
        return to_result(protocol_->write_registers(id, address, data));
    }

    void FeetechScsDriver::register_driver() {
        core::DriverRegistry::instance().add("feetech_scs",
                                             [] { return std::make_unique<FeetechScsDriver>(); });
    }

} // namespace servo_backends
