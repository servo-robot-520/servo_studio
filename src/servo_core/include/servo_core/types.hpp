#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace servo_core {

    /// @brief 舵机 id（总线地址）。
    using ServoId = uint8_t;

    /// @brief 总线连接配置。
    struct BusConfig {
        /// 串口设备，例如 /dev/ttyUSB0。
        std::string port;
        /// 波特率。
        int baud_rate{1000000};
        /// 单次应答超时（毫秒）。
        int response_timeout_ms{10};
    };

    /// @brief 单个舵机的换算与钳位配置。
    ///
    /// 换算约定：0 度 = center_ticks，range_deg = 全量程（full_scale_ticks）
    /// 对应的机械行程。编码器几何随型号而异（如 SCS 电位器版 1024/中位 511，
    /// 磁编码器版 4096/中位 2048），因此以配置表达而非写死。
    struct ServoConfig {
        /// 舵机 id。
        ServoId id{};
        /// 用户可读名称。
        std::string label;
        /// 机械行程（度）。
        double range_deg{300.0};
        /// 命令位置钳位下限（原始计数）。
        double min_ticks{0.0};
        /// 命令位置钳位上限（原始计数）。
        double max_ticks{1023.0};
        /// 0 度对应的编码器计数。
        double center_ticks{511.0};
        /// 编码器全量程计数。
        double full_scale_ticks{1024.0};
    };

    /// @brief 原始计数 → 度。
    inline double ticks_to_degrees(const ServoConfig& config, const double ticks) {
        return (ticks - config.center_ticks) * config.range_deg / config.full_scale_ticks;
    }

    /// @brief 速度（计数/s）→ 度/s。
    inline double ticks_s_to_deg_s(const ServoConfig& config, const double ticks_s) {
        return ticks_s * config.range_deg / config.full_scale_ticks;
    }

    /// @brief 度 → 原始计数（按 min/max_ticks 与编码器量程钳位）。
    inline int degrees_to_ticks(const ServoConfig& config, const double degrees) {
        const long ticks = std::lround(degrees * config.full_scale_ticks / config.range_deg +
                                       config.center_ticks);
        const long lower = std::max<long>(0, std::lround(config.min_ticks));
        const long upper = std::min<long>(std::lround(config.full_scale_ticks) - 1,
                                          std::lround(config.max_ticks));
        return static_cast<int>(std::clamp(ticks, lower, upper));
    }

    /// @brief 遥测通道支持集（监控状态表/折线图的数据列）。
    ///
    /// 不支持的通道 read_state 填 NaN，界面显示 "--"（图表断点）；
    /// 由 IServoDriver::telemetry_channels() 按舵机查询，扫描结果随
    /// ServoInfo 下发给界面作掩码依据。
    struct TelemetryChannels {
        bool position{true};
        bool velocity{true};
        bool load{true};
        bool voltage{true};
        bool temperature{true};
        bool current{true};

        /// 无任何遥测通道（如 UBTECH 拆机协议）。
        static TelemetryChannels none() {
            return {.position = false,
                    .velocity = false,
                    .load = false,
                    .voltage = false,
                    .temperature = false,
                    .current = false};
        }
    };

    /// @brief 驱动能力（总线级）：界面按数据决定显示与行为，不做协议名分支。
    struct DriverCapabilities {
        /// 扭矩是否支持阻尼档（0/1/2；不支持则界面收成两态）。
        bool torque_damping{true};
        /// read_state 可否用于自动轮询（无副作用）；
        /// false 时界面禁用轮询，仅允许手动单次读取。
        bool polling_safe{true};
        /// 是否使用协议专属操作面板（如 UBTECH 的角度/运动/锁定时间面板）
        /// 而非通用检查器。
        bool uses_protocol_panel{false};
    };

    /// @brief 舵机身份信息。
    struct ServoInfo {
        /// 舵机 id。
        ServoId id{};
        /// 型号名称，未知型号为 "unknown"。
        std::string model;
        /// 原始型号标识。
        uint16_t model_number{};
        /// 固件主版本号。
        uint8_t firmware_major{};
        /// 固件次版本号。
        uint8_t firmware_minor{};
        /// 型号允许的最大机械行程（度）。
        double max_range_deg{360.0};
        /// 默认换算/钳位配置（型号几何：ticks↔度、命令钳位），
        /// 扫描后由界面播种每舵机配置；full_scale_ticks ≤ 0 表示未提供。
        ServoConfig defaults{};
        /// 支持的遥测通道（界面掩码依据）。
        TelemetryChannels channels{};
    };

    /// @brief 实时状态；物理量用常用单位，位置/速度同时给出原始计数。
    struct ServoState {
        /// 舵机 id。
        ServoId id{};
        /// 当前位置（原始计数）。
        double position_ticks{};
        /// 当前位置（度），0 为中位。
        double position_deg{};
        /// 当前速度（原始计数/s）。
        double velocity_ticks_s{};
        /// 当前速度（度/s）。
        double velocity_deg_s{};
        /// 负载，-1..1。
        double load{};
        /// 电压（V）。
        double voltage{};
        /// 温度（°C）。
        double temperature{};
        /// 电流（A）。
        double current{};
        /// 移动标志：运动中为 true。
        bool moving{};
    };

    /// @brief 参数存储区/访问方式。
    enum class ParamAccess {
        /// 只读。
        kReadOnly,
        /// SRAM，实时可写。
        kRam,
        /// EEPROM，掉电保存，写入需安全流程。
        kEeprom,
    };

    /// @brief 存储区域（显示用，与访问方式正交）。
    enum class ParamArea {
        kEeprom,
        kSram,
        kDefault,
    };

    /// @brief 中立参数元数据；厂商差异（寄存器地址、字宽）由后端内部处理。
    struct ParamDescriptor {
        /// 键名（snake_case，跨厂商稳定）。
        std::string key;
        /// 英文显示名。
        std::string label;
        /// 中文显示名。
        std::string zh;
        /// 单位；无单位为空字符串。
        std::string unit;
        /// 存储区/访问方式。
        ParamAccess access;
        /// 存储区域。
        ParamArea area;
        /// 寄存器地址（显示用）。
        int address{-1};
        /// 最小取值。
        int min{};
        /// 最大取值。
        int max{};
        /// 枚举取值表（参数值为索引时给出实际含义）；空为自由取值。
        std::vector<int> choices;
    };

} // namespace servo_core
