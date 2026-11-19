#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

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

    /// @brief UBTECH（优必选拆机关节舵机）后端：非官方协议适配到 IServoDriver。
    ///
    /// 协议为 10 字节定长帧（FA AF / FC CF 帧头，字节 2..7 累加和，ED 帧尾），
    /// 功能极少且与常规总线舵机差异大：
    ///  - 目标角度 0–240°（绝对角，1 tick = 1 度），运动时间 0–255×20ms，
    ///    锁定时间 0–3270×20ms（锁定期内舵机忽略新的 0x01 命令）；
    ///  - 角度回读（0x02）返回目标角+实际角，但回读后舵机失电、仅靠齿轮阻尼
    ///    维持位置——禁止接入自动轮询，只能手动单次读取；
    ///  - 无速度/扭矩/温度/电压/电流遥测；无寄存器概念（读写寄存器接口不支持）；
    ///  - 强制中止（0x01 FF）= 立即停转并失电，映射为 set_torque(id,false)；
    ///    set_torque(id,true) 不支持（无法凭命令上电锁紧，下发目标即上电）；
    ///  - 角度偏移（1/3 度，±90=±30°）为易失参数，掉电清零；
    ///  - 固件版本（FC CF 0x01）回应 4 个语义未知的版本字节，按原样暴露为
    ///    firmware_v1..v4 参数。
    ///
    /// 单位约定：position_ticks 即协议角度（度）；time_ms/lock_ms 按 20ms 量化。
    class UbtechDriver : public IServoDriver {
    public:
        ~UbtechDriver() override;

        void set_trace_hook(TraceHook hook) override;

        Result open(const BusConfig& config) override;
        void close() override;
        [[nodiscard]] bool is_open() const override;

        Result set_servo_config(const ServoConfig& config) override;

        Expected<std::vector<ServoInfo>> scan(int first_id, int last_id) override;
        Expected<ServoInfo> identify(ServoId id) override;

        /// @brief 角度回读（0x02）。
        /// @warning 舵机回应后即失电（仅靠齿轮阻尼维持位置），严禁高频调用。
        Expected<ServoState> read_state(ServoId id) override;

        Result write_goal(ServoId id, int position_ticks, int time_ms, int speed_ticks_s,
                          int acc, int lock_ms) override;
        Result sync_write_goals(const std::vector<ServoId>& ids,
                                const std::vector<int>& positions_ticks,
                                const std::vector<int>& times_ms,
                                const std::vector<int>& speeds_ticks_s,
                                int acc) override;

        Result set_torque(ServoId id, bool enabled) override;

        [[nodiscard]] std::vector<ParamDescriptor> parameters() const override;
        Expected<int> read_param(ServoId id, const std::string& key) override;
        Result write_param(ServoId id, const std::string& key, int value) override;

        Expected<std::vector<uint8_t>> read_registers(ServoId id, uint8_t address,
                                                      size_t length) override;
        Result write_registers(ServoId id, uint8_t address,
                               const std::vector<uint8_t>& data) override;

        /// @brief 注册到 DriverRegistry，名称 "ubtech"。
        static void register_driver();

    private:
        using Frame = std::array<uint8_t, 10>;

        /// 组帧并计算校验和（字节 2..7 累加取低 8 位）。
        [[nodiscard]] static Frame build_frame(uint8_t header1, uint8_t header2, ServoId id,
                                               uint8_t command, uint8_t p1h, uint8_t p1l,
                                               uint8_t p2h, uint8_t p2l);

        Result send_frame(const Frame& frame);
        /// 读取完整回应帧（跳过噪声，校验帧头/帧尾/校验和）。
        Expected<Frame> receive_frame(uint8_t header1, uint8_t header2);
        /// 读取短回应（0xAA [+ID]）；超时无回应即失败（协议规定失败不回应）。
        Result receive_short_ack();
        void trace(const char* direction, const uint8_t* data, size_t size) const;

        /// 命令封装：0x01 转动（angle 度、move/lock 为 20ms 量化值）。
        Result cmd_move(ServoId id, uint8_t angle_deg, uint8_t move_units, uint16_t lock_units);
        /// 0x01 FF 强制中止（停转并失电）。
        Result cmd_stop(ServoId id);
        /// 0x02 角度回读：目标角 + 实际角（度）。副作用：舵机失电。
        Expected<std::pair<int, int>> cmd_read_angle(ServoId id);
        /// 0xCD 修改 ID（立即生效）；回应字节 5 为修改前 ID。
        Result cmd_set_id(ServoId id, ServoId new_id);
        /// 0xD2 设置角度偏移（1/3 度，±90；掉电清零）。
        Result cmd_set_offset(ServoId id, int offset);
        /// 0xD4 读取角度偏移。
        Expected<int> cmd_read_offset(ServoId id);
        /// FC CF 0x01 读取固件版本（4 个语义未知的版本字节）。
        Expected<std::array<uint8_t, 4>> cmd_read_version(ServoId id);

        [[nodiscard]] const ServoConfig& config_for(ServoId id) const;

        int fd_{-1};
        int timeout_ms_{10};
        TraceHook trace_hook_;
        std::map<ServoId, ServoConfig> configs_;
    };

} // namespace servo_backends
