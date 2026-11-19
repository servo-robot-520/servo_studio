#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "feetech_scs_ros2_control/scs_serial_port.hpp"
#include "servo_core/driver.hpp"

namespace servo_backends {

    using servo_core::BusConfig;
    using servo_core::Expected;
    using servo_core::IServoDriver;
    using servo_core::ParamDescriptor;
    using servo_core::Result;
    using servo_core::ServoConfig;
    using servo_core::ServoId;
    using servo_core::ServoInfo;
    using servo_core::ServoState;

    /// @brief UBTECH 拆机舵机 + 白菜总线舵机（MicroDuck 固件）后端。
    ///
    /// 固件项目：ubtech_servo_firmware（gitee.com/liqiulong123），作者 **liqiulong123**。
    /// 刷在舵机原厂驱动板上，对上位机完整表现为飞特 SCS 舵机（帧/指令/控制表骨架一致），
    /// 但以下语义与飞特不同：
    ///  - 角度 0–4095（12 位）；版本在寄存器 3/4（飞特的 0/1/2 未实现）；
    ///  - 应答状态字节是实时错误位（欠压/过压/过热/堵转）——本驱动**宽容处理**：
    ///    命令照常完成，非零状态缓存进 last_error 参数并在帧跟踪中可见；
    ///  - 改 ID 后应答回**新 ID**；0x06 是恢复出厂（飞特为参数恢复）；
    ///  - 写配置寄存器需先写 48=0 解锁（锁定时固件静默拒写 cfg 寄存器，
    ///    40/33/41/48 例外；42-47 运行时寄存器不受锁门）。
    ///
    /// 两种硬件共用本后端，按版本字（寄存器 3/4）识别变体并在驱动内分支：
    ///  - SAMD10（0x010B，v1.11）：指令齐全（REG_WRITE/ACTION/SYNC_READ）、
    ///    波特率 5 档可切（码 4 = 921600）、反馈真值、自动 Flash 保存；
    ///  - Mega8（0x0203，v2.3）：波特率写死 921600（reg6 虚设）、无 REG_WRITE/ACTION、
    ///    速度/负载为补码、电压/温度/电流为假值（本驱动填 NaN，UI 显示 "--"）、
    ///    配置需写 49=0xA5 才落 EEPROM（本驱动写配置后自动补）。
    ///
    /// 单位约定：position_ticks 即寄存器计数（0–4095）；ServoConfig 换算默认
    /// center 2048 / full_scale 4096 / range 360°（钳位按变体默认）。
    class UbtechLiqiulong123Driver : public IServoDriver {
    public:
        ~UbtechLiqiulong123Driver() override;

        void set_trace_hook(TraceHook hook) override;

        Result open(const BusConfig& config) override;
        void close() override;
        [[nodiscard]] bool is_open() const override;

        Result set_servo_config(const ServoConfig& config) override;

        Expected<std::vector<ServoInfo>> scan(int first_id, int last_id) override;
        Expected<ServoInfo> identify(ServoId id) override;
        Expected<ServoState> read_state(ServoId id) override;

        Result write_goal(ServoId id, int position_ticks, int time_ms, int speed_ticks_s,
                          int acc, int lock_ms) override;
        Result sync_write_goals(const std::vector<ServoId>& ids,
                                const std::vector<int>& positions_ticks,
                                const std::vector<int>& times_ms,
                                const std::vector<int>& speeds_ticks_s,
                                int acc) override;

        [[nodiscard]] bool supports_staged_write() const override;
        Result stage_goal(ServoId id, int position_ticks, int time_ms, int speed_ticks_s,
                          int acc) override;
        Result trigger_staged() override;

        Result set_torque(ServoId id, bool enabled) override;

        [[nodiscard]] std::vector<ParamDescriptor> parameters() const override;
        Expected<int> read_param(ServoId id, const std::string& key) override;
        Result write_param(ServoId id, const std::string& key, int value) override;

        Expected<std::vector<uint8_t>> read_registers(ServoId id, uint8_t address,
                                                      size_t length) override;
        Result write_registers(ServoId id, uint8_t address,
                               const std::vector<uint8_t>& data) override;

        /// @brief 注册到 DriverRegistry，名称 "ubtech_liqiulong123"。
        static void register_driver();

    private:
        /// 固件变体（按寄存器 3/4 版本字识别）。
        enum class Variant {
            kSamd10,
            kMega8,
        };

        /// 一次请求-应答的解析结果。
        struct Reply {
            uint8_t id{0};
            uint8_t status{0};
            std::vector<uint8_t> params;
        };

        /// 发送指令帧并（按命令语义）接收应答帧。
        /// @param ack_required PING/READ 必须有应答；写类命令超时视为已执行
        ///        （应答开关 reg8=0 时写命令本就无 ACK）。
        /// @param extra_ids 额外可接受的应答 ID（改 ID 时回新 ID）。
        Expected<Reply> transact(ServoId id, uint8_t instruction,
                                 const std::vector<uint8_t>& params, bool ack_required,
                                 const std::vector<ServoId>& extra_ids = {});

        /// 读 1/2 字节寄存器（大端）。
        Expected<int> read_reg(ServoId id, uint8_t address, int bytes);
        /// 写 1/2 字节寄存器（大端）。
        Result write_reg(ServoId id, uint8_t address, int value, int bytes);

        /// cfg 寄存器写流程：48=0 解锁 → 写 → 48=1 回锁（mega8 再补 49=0xA5）。
        Result write_cfg_reg(ServoId id, uint8_t address, int value, int bytes);
        /// mega8：写 49=0xA5 触发 EEPROM 保存（no-op on samd10）。
        Result persist_if_mega8(ServoId id);

        /// 识别/缓存变体（未知时按 samd10 超集处理）。
        Expected<Variant> variant_for(ServoId id);
        [[nodiscard]] const ServoConfig& config_for(ServoId id) const;
        /// 按变体的换算默认值。
        [[nodiscard]] static ServoConfig default_config(Variant variant, ServoId id);

        void trace(const char* direction, const uint8_t* data, size_t size) const;

        feetech_scs_ros2_control::ScsSerialPort port_;
        TraceHook trace_hook_;
        std::map<ServoId, ServoConfig> configs_;
        std::map<ServoId, Variant> variants_;
        uint8_t last_error_{0};
        int timeout_ms_{10};
    };

} // namespace servo_backends
