#pragma once

#include <cstdint>
#include <string>

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
    struct ServoConfig {
        /// 舵机 id。
        ServoId id{};
        /// 用户可读名称。
        std::string label;
        /// 机械行程（度），对应编码器整个量程。
        double range_deg{300.0};
        /// 命令位置钳位下限（原始计数）。
        double min_ticks{0.0};
        /// 命令位置钳位上限（原始计数）。
        double max_ticks{1023.0};
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

    /// @brief 中立参数元数据；厂商差异（寄存器地址、字宽）由后端内部处理。
    struct ParamDescriptor {
        /// 键名（snake_case，跨厂商稳定）。
        std::string key;
        /// 英文显示名。
        std::string label;
        /// 单位；无单位为空字符串。
        std::string unit;
        /// 存储区/访问方式。
        ParamAccess access;
        /// 最小取值。
        int min{};
        /// 最大取值。
        int max{};
    };

} // namespace servo_core
