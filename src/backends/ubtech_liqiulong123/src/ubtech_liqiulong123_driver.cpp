#include "servo_backends/ubtech_liqiulong123_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <utility>

#include "servo_core/registry.hpp"

namespace servo_backends {
    namespace {
        // 指令码（与飞特 SCS 一致；0x06/0x0A 语义不同，见类注释）。
        constexpr uint8_t kPing = 0x01;
        constexpr uint8_t kRead = 0x02;
        constexpr uint8_t kWrite = 0x03;
        constexpr uint8_t kRegWrite = 0x04;
        constexpr uint8_t kAction = 0x05;
        constexpr uint8_t kSyncRead = 0x82;
        constexpr uint8_t kSyncWrite = 0x83;

        constexpr uint8_t kBroadcastId = 0xFE;

        // 帧总长 = 4（FF FF ID LEN）+ LEN（状态/参数/校验和）。
        constexpr size_t kReplyHeader = 4;
        constexpr size_t kMaxReadData = 32; // 固件 READ 应答数据上限
        constexpr int kSyncWriteChunk = 16; // mega8 RX 128B ≈ 16 颗/帧

        // 控制表地址（两种变体一致；★=MicroDuck 扩展）。
        constexpr uint8_t kRegFwMajor = 3;
        constexpr uint8_t kRegFwMinor = 4;
        constexpr uint8_t kRegId = 5;
        constexpr uint8_t kRegBaud = 6;
        constexpr uint8_t kRegResponse = 8;
        constexpr uint8_t kRegAngleMin = 9;
        constexpr uint8_t kRegAngleMax = 11;
        constexpr uint8_t kRegMaxTemp = 13;
        constexpr uint8_t kRegMaxVolt = 14;
        constexpr uint8_t kRegMinVolt = 15;
        constexpr uint8_t kRegMaxTorque = 16;
        constexpr uint8_t kRegPGain = 21;
        constexpr uint8_t kRegDGain = 22;
        constexpr uint8_t kRegIGain = 23;
        constexpr uint8_t kRegStartupForce = 24;
        constexpr uint8_t kRegDeadbandPos = 26;
        constexpr uint8_t kRegDeadbandNeg = 27;
        constexpr uint8_t kRegDirection = 29;
        constexpr uint8_t kRegMode = 33;
        constexpr uint8_t kRegTorqueEnable = 40;
        constexpr uint8_t kRegAccel = 41;
        constexpr uint8_t kRegGoal = 42; // 42-47：位置/时间/速度（模式 3 时 42-43=目标电流）
        constexpr uint8_t kRegLock = 48;
        constexpr uint8_t kRegSaveTrigger = 49; // ★mega8：0xA5 = 落 EEPROM
        constexpr uint8_t kRegPresentPos = 56;
        constexpr uint8_t kRegMoving = 66;
        constexpr uint8_t kRegCollisionMirror = 67;
        constexpr uint8_t kRegPowerOnGoal = 73;
        constexpr uint8_t kRegDirSelfTest = 75;
        constexpr uint8_t kRegBusDrive = 76;
        constexpr uint8_t kRegBaudTrim = 77;
        constexpr uint8_t kRegRemapEnable = 82;
        constexpr uint8_t kRegRemapLow = 83;
        constexpr uint8_t kRegRemapHigh = 85;
        constexpr uint8_t kRegPwmFreq = 87;
        constexpr uint8_t kRegBlanking = 88;
        constexpr uint8_t kRegBrakeInterlock = 89;
        constexpr uint8_t kRegLedMask = 107;
        constexpr uint8_t kRegLedEffect = 119;
        constexpr uint8_t kRegCollisionTh = 124;
        constexpr uint8_t kRegCollisionWin = 125;
        constexpr uint8_t kRegTorqueSpeedLimit = 126;

        // 波特率码 0..4（⚠码 4 = 921600，与飞特的 115200 不同）。
        constexpr int kBaudCodes[] = {1000000, 500000, 250000, 128000, 921600};

        uint8_t checksum(const uint8_t id, const uint8_t length, const uint8_t* body) {
            auto sum = static_cast<uint8_t>(id + length);
            for (int i = 0; i < length - 1; ++i) { // body 末字节是校验和本身
                sum = static_cast<uint8_t>(sum + body[i]);
            }
            return static_cast<uint8_t>(~sum);
        }
    } // namespace

    UbtechLiqiulong123Driver::~UbtechLiqiulong123Driver() {
        UbtechLiqiulong123Driver::close();
    }

    void UbtechLiqiulong123Driver::set_trace_hook(TraceHook hook) {
        trace_hook_ = std::move(hook);
    }

    Result UbtechLiqiulong123Driver::open(const BusConfig& config) {
        // 传输层复用飞特串口（8N1 原始模式，任意波特率；mega8 = 921600）。
        if (const auto result = port_.open(config.port, config.baud_rate); !result) {
            return Result::failure(result.error());
        }
        timeout_ms_ = std::max(1, config.response_timeout_ms);
        return Result::success();
    }

    void UbtechLiqiulong123Driver::close() {
        port_.close();
        variants_.clear();
    }

    bool UbtechLiqiulong123Driver::is_open() const {
        return port_.is_open();
    }

    void UbtechLiqiulong123Driver::trace(const char* direction, const uint8_t* data,
                                         const size_t size) const {
        if (trace_hook_) {
            trace_hook_(direction, std::vector<uint8_t>(data, data + size));
        }
    }

    Expected<UbtechLiqiulong123Driver::Reply> UbtechLiqiulong123Driver::transact(
        const ServoId id, const uint8_t instruction, const std::vector<uint8_t>& params,
        const bool ack_required, const std::vector<ServoId>& extra_ids) {
        if (!port_.is_open()) {
            return Expected<Reply>::failure("Serial port is not open");
        }
        // 组帧：FF FF ID LEN | INS 参数… 校验和；LEN = INS+参数+校验和。
        std::vector<uint8_t> body;
        body.push_back(instruction);
        body.insert(body.end(), params.begin(), params.end());
        body.push_back(0);
        const auto length = static_cast<uint8_t>(body.size());
        body.back() = checksum(id, length, body.data());
        std::vector<uint8_t> frame = {0xFF, 0xFF, id, length};
        frame.insert(frame.end(), body.begin(), body.end());

        port_.flush_input(); // 丢弃上次超时残留
        trace("tx", frame.data(), frame.size());
        if (const auto result = port_.write(frame); !result) {
            return Expected<Reply>::failure(result.error());
        }

        // 写类命令：超时视为已执行（应答开关 reg8=0 时本就无 ACK）。
        const auto timeout = std::chrono::milliseconds(timeout_ms_);
        std::vector<uint8_t> header(kReplyHeader);
        if (const auto result = port_.read_exact(header, timeout); !result) {
            if (ack_required) {
                return Expected<Reply>::failure("No response: " + result.error());
            }
            return Expected<Reply>(Reply{});
        }
        if (header[0] != 0xFF || header[1] != 0xFF) {
            return Expected<Reply>::failure("Response framing error");
        }
        const uint8_t reply_id = header[2];
        const auto reply_length = header[3];
        if (reply_length < 2) {
            return Expected<Reply>::failure("Response length too small");
        }
        std::vector<uint8_t> reply_body(reply_length);
        if (const auto result = port_.read_exact(reply_body, timeout); !result) {
            return Expected<Reply>::failure("Truncated response: " + result.error());
        }
        {
            std::vector<uint8_t> full = header;
            full.insert(full.end(), reply_body.begin(), reply_body.end());
            trace("rx", full.data(), full.size());
        }
        if (checksum(reply_id, reply_length, reply_body.data()) != reply_body.back()) {
            return Expected<Reply>::failure("Response checksum mismatch");
        }
        bool id_ok = reply_id == id;
        for (const auto extra : extra_ids) {
            id_ok = id_ok || reply_id == extra;
        }
        if (!id_ok) {
            return Expected<Reply>::failure("Unexpected response id " +
                                            std::to_string(reply_id));
        }
        Reply reply;
        reply.id = reply_id;
        reply.status = reply_body.front();
        reply.params.assign(reply_body.begin() + 1, reply_body.end() - 1);
        // 宽容策略（用户确认）：状态字节是实时错误位（欠压/过压/过热/堵转），
        // 不判失败，只缓存供 last_error 参数查看。
        if (reply.status != 0) {
            last_error_ = reply.status;
        }
        return Expected<Reply>(reply);
    }

    Expected<int> UbtechLiqiulong123Driver::read_reg(const ServoId id, const uint8_t address,
                                                     const int bytes) {
        const auto reply =
            transact(id, kRead, {address, static_cast<uint8_t>(bytes)}, true);
        if (!reply) {
            return Expected<int>::failure(reply.error());
        }
        if (static_cast<int>(reply.value().params.size()) < bytes) {
            return Expected<int>::failure("Short read response");
        }
        // 大端：高字节在前。
        int value = 0;
        for (int i = 0; i < bytes; ++i) {
            value = (value << 8) | reply.value().params[static_cast<size_t>(i)];
        }
        return Expected<int>(value);
    }

    Result UbtechLiqiulong123Driver::write_reg(const ServoId id, const uint8_t address,
                                               const int value, const int bytes) {
        std::vector<uint8_t> params{address};
        for (int i = bytes - 1; i >= 0; --i) {
            params.push_back(static_cast<uint8_t>((value >> (8 * i)) & 0xFF));
        }
        const auto reply = transact(id, kWrite, params, false);
        return reply ? Result::success() : Result::failure(reply.error());
    }

    Result UbtechLiqiulong123Driver::write_cfg_reg(const ServoId id, const uint8_t address,
                                                   const int value, const int bytes) {
        // 固件锁门：48=1 时 cfg 寄存器写入被静默拒写（40/33/41/48 例外），
        // 与 setreg.py 同流程：解锁 → 写 → 回锁。
        if (const auto result = write_reg(id, kRegLock, 0, 1); !result) {
            return result;
        }
        if (const auto result = write_reg(id, address, value, bytes); !result) {
            return result;
        }
        if (const auto result = write_reg(id, kRegLock, 1, 1); !result) {
            return result;
        }
        return persist_if_mega8(id);
    }

    Result UbtechLiqiulong123Driver::persist_if_mega8(const ServoId id) {
        if (const auto variant = variant_for(id); variant && variant.value() == Variant::kMega8) {
            // mega8：写 49=0xA5 才落 EEPROM，否则掉电即丢。
            return write_reg(id, kRegSaveTrigger, 0xA5, 1);
        }
        return Result::success();
    }

    Expected<UbtechLiqiulong123Driver::Variant>
    UbtechLiqiulong123Driver::variant_for(const ServoId id) {
        if (const auto it = variants_.find(id); it != variants_.end()) {
            return Expected<Variant>(it->second);
        }
        const auto version = read_reg(id, kRegFwMajor, 2); // 3/4：固件主/次版本
        if (!version) {
            return Expected<Variant>::failure(version.error());
        }
        // 版本字主版本 = 2 → Mega8 固件（v2.x）；其余按 SAMD10 超集处理。
        const Variant variant = ((version.value() >> 8) & 0xFF) == 2 ? Variant::kMega8
                                                                    : Variant::kSamd10;
        variants_[id] = variant;
        return Expected<Variant>(variant);
    }

    ServoConfig UbtechLiqiulong123Driver::default_config(const Variant variant,
                                                         const ServoId id) {
        ServoConfig config;
        config.id = id;
        config.range_deg = 360.0; // 4096 counts/圈
        config.center_ticks = 2048.0;
        config.full_scale_ticks = 4096.0;
        if (variant == Variant::kMega8) {
            config.min_ticks = 24.0;
            config.max_ticks = 4080.0;
        }
        else {
            config.min_ticks = 540.0;
            config.max_ticks = 3800.0;
        }
        return config;
    }

    const ServoConfig& UbtechLiqiulong123Driver::config_for(const ServoId id) const {
        static const ServoConfig kFallback{
            .id = 0,
            .label = "",
            .range_deg = 360.0,
            .min_ticks = 0.0,
            .max_ticks = 4095.0,
            .center_ticks = 2048.0,
            .full_scale_ticks = 4096.0,
        };
        if (const auto it = configs_.find(id); it != configs_.end()) {
            return it->second;
        }
        return kFallback;
    }

    Result UbtechLiqiulong123Driver::set_servo_config(const ServoConfig& config) {
        configs_[config.id] = config;
        return Result::success();
    }

    TelemetryChannels UbtechLiqiulong123Driver::telemetry_channels(const ServoId id) const {
        TelemetryChannels channels;
        if (const auto it = variants_.find(id); it != variants_.end() && it->second == Variant::kMega8) {
            // mega8 板无电压/温度/电流采样（寄存器为假值）。
            channels.voltage = false;
            channels.temperature = false;
            channels.current = false;
        }
        return channels;
    }

    DriverCapabilities UbtechLiqiulong123Driver::capabilities() const {
        // 无阻尼档；read_state 无副作用可轮询；使用通用检查器。
        return {.torque_damping = false, .polling_safe = true, .uses_protocol_panel = false};
    }

    namespace {
        // 参数规格表：地址/宽度/符号/范围（两种变体同一骨架）。
        struct ParamSpec {
            const char* key;
            const char* label;
            const char* zh;
            const char* unit;
            servo_core::ParamAccess access;
            servo_core::ParamArea area;
            int address;
            int bytes;
            bool is_signed;
            int min;
            int max;
            std::vector<int> choices;
        };

        using servo_core::ParamAccess;
        using servo_core::ParamArea;

        const std::vector<ParamSpec>& param_specs() {
            static const std::vector<ParamSpec> specs = {
                {.key = "baud_rate",
                 .label = "Baud rate",
                 .zh = "波特率",
                 .unit = "",
                 .access = ParamAccess::kEeprom,
                 .area = ParamArea::kEeprom,
                 .address = 6,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 4,
                 .choices = {1000000, 500000, 250000, 128000, 921600}},
                {.key = "response_level",
                 .label = "Response level",
                 .zh = "应答开关",
                 .unit = "",
                 .access = ParamAccess::kRam,
                 .area = ParamArea::kSram,
                 .address = 8,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 1,
                 .choices = {0, 1}},
                {.key = "angle_min",
                 .label = "Angle min limit",
                 .zh = "最小角度限制",
                 .unit = "step",
                 .access = ParamAccess::kEeprom,
                 .area = ParamArea::kEeprom,
                 .address = 9,
                 .bytes = 2,
                 .is_signed = false,
                 .min = 0,
                 .max = 4095,
                 .choices = {}},
                {.key = "angle_max",
                 .label = "Angle max limit",
                 .zh = "最大角度限制",
                 .unit = "step",
                 .access = ParamAccess::kEeprom,
                 .area = ParamArea::kEeprom,
                 .address = 11,
                 .bytes = 2,
                 .is_signed = false,
                 .min = 0,
                 .max = 4095,
                 .choices = {}},
                {.key = "max_temperature",
                 .label = "Max temperature",
                 .zh = "最高温度上限",
                 .unit = "°C",
                 .access = ParamAccess::kEeprom,
                 .area = ParamArea::kEeprom,
                 .address = 13,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 255,
                 .choices = {}},
                {.key = "max_input_voltage",
                 .label = "Max input voltage",
                 .zh = "最高输入电压",
                 .unit = "0.1V",
                 .access = ParamAccess::kRam,
                 .area = ParamArea::kSram,
                 .address = 14,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 255,
                 .choices = {}},
                {.key = "min_input_voltage",
                 .label = "Min input voltage",
                 .zh = "最低输入电压",
                 .unit = "0.1V",
                 .access = ParamAccess::kRam,
                 .area = ParamArea::kSram,
                 .address = 15,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 255,
                 .choices = {}},
                {.key = "max_torque",
                 .label = "Max torque",
                 .zh = "最大力矩",
                 .unit = "0.1%",
                 .access = ParamAccess::kEeprom,
                 .area = ParamArea::kEeprom,
                 .address = 16,
                 .bytes = 2,
                 .is_signed = false,
                 .min = 0,
                 .max = 1000,
                 .choices = {}},
                {.key = "p_gain",
                 .label = "P gain",
                 .zh = "位置环 P",
                 .unit = "",
                 .access = ParamAccess::kEeprom,
                 .area = ParamArea::kEeprom,
                 .address = 21,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 255,
                 .choices = {}},
                {.key = "d_gain",
                 .label = "D gain",
                 .zh = "位置环 D",
                 .unit = "",
                 .access = ParamAccess::kEeprom,
                 .area = ParamArea::kEeprom,
                 .address = 22,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 255,
                 .choices = {}},
                {.key = "i_gain",
                 .label = "I gain",
                 .zh = "位置环 I",
                 .unit = "",
                 .access = ParamAccess::kEeprom,
                 .area = ParamArea::kEeprom,
                 .address = 23,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 255,
                 .choices = {}},
                {.key = "min_startup_force",
                 .label = "Min startup force",
                 .zh = "最小启动力",
                 .unit = "",
                 .access = ParamAccess::kEeprom,
                 .area = ParamArea::kEeprom,
                 .address = 24,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 255,
                 .choices = {}},
                {.key = "deadband_pos",
                 .label = "Deadband positive",
                 .zh = "正向不灵敏区",
                 .unit = "step",
                 .access = ParamAccess::kEeprom,
                 .area = ParamArea::kEeprom,
                 .address = 26,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 255,
                 .choices = {}},
                {.key = "deadband_neg",
                 .label = "Deadband negative",
                 .zh = "负向不灵敏区",
                 .unit = "step",
                 .access = ParamAccess::kEeprom,
                 .area = ParamArea::kEeprom,
                 .address = 27,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 255,
                 .choices = {}},
                {.key = "direction_invert",
                 .label = "Direction invert",
                 .zh = "方向开关",
                 .unit = "",
                 .access = ParamAccess::kEeprom,
                 .area = ParamArea::kEeprom,
                 .address = 29,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 1,
                 .choices = {0, 1}},
                {.key = "work_mode",
                 .label = "Work mode",
                 .zh = "工作模式（0=位置 1=轮速 3=力矩）",
                 .unit = "",
                 .access = ParamAccess::kEeprom,
                 .area = ParamArea::kEeprom,
                 .address = 33,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 3,
                 .choices = {}},
                {.key = "torque_enable",
                 .label = "Torque enable",
                 .zh = "扭矩使能（0=松力 1=锁紧）",
                 .unit = "",
                 .access = ParamAccess::kRam,
                 .area = ParamArea::kSram,
                 .address = 40,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 1,
                 .choices = {0, 1}},
                {.key = "acceleration",
                 .label = "Acceleration",
                 .zh = "加速度",
                 .unit = "",
                 .access = ParamAccess::kRam,
                 .area = ParamArea::kSram,
                 .address = 41,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 255,
                 .choices = {}},
                {.key = "torque_target_current",
                 .label = "Torque target current",
                 .zh = "力矩模式目标电流（42-43）",
                 .unit = "mA",
                 .access = ParamAccess::kRam,
                 .area = ParamArea::kSram,
                 .address = 42,
                 .bytes = 2,
                 .is_signed = true,
                 .min = -1000,
                 .max = 1000,
                 .choices = {}},
                {.key = "power_on_position",
                 .label = "Power-on goal",
                 .zh = "上电目标位置（0xFFFF=保持不动）",
                 .unit = "step",
                 .access = ParamAccess::kEeprom,
                 .area = ParamArea::kEeprom,
                 .address = 73,
                 .bytes = 2,
                 .is_signed = false,
                 .min = 0,
                 .max = 65535,
                 .choices = {}},
                {.key = "direction_selftest",
                 .label = "Direction self-test",
                 .zh = "方向自检结果",
                 .unit = "",
                 .access = ParamAccess::kReadOnly,
                 .area = ParamArea::kSram,
                 .address = 75,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 2,
                 .choices = {}},
                {.key = "bus_drive_mode",
                 .label = "Bus drive mode",
                 .zh = "总线驱动模式（1=高阻，多机必设）",
                 .unit = "",
                 .access = ParamAccess::kEeprom,
                 .area = ParamArea::kEeprom,
                 .address = 76,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 1,
                 .choices = {0, 1}},
                {.key = "baud_trim",
                 .label = "Baud trim",
                 .zh = "波特率微调",
                 .unit = "",
                 .access = ParamAccess::kRam,
                 .area = ParamArea::kSram,
                 .address = 77,
                 .bytes = 1,
                 .is_signed = true,
                 .min = -12,
                 .max = 12,
                 .choices = {}},
                {.key = "remap_enable",
                 .label = "Remap enable",
                 .zh = "位置重映射开关",
                 .unit = "",
                 .access = ParamAccess::kEeprom,
                 .area = ParamArea::kEeprom,
                 .address = 82,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 1,
                 .choices = {0, 1}},
                {.key = "remap_low",
                 .label = "Remap window low",
                 .zh = "行程窗口低",
                 .unit = "step",
                 .access = ParamAccess::kEeprom,
                 .area = ParamArea::kEeprom,
                 .address = 83,
                 .bytes = 2,
                 .is_signed = false,
                 .min = 0,
                 .max = 65535,
                 .choices = {}},
                {.key = "remap_high",
                 .label = "Remap window high",
                 .zh = "行程窗口高",
                 .unit = "step",
                 .access = ParamAccess::kEeprom,
                 .area = ParamArea::kEeprom,
                 .address = 85,
                 .bytes = 2,
                 .is_signed = false,
                 .min = 0,
                 .max = 65535,
                 .choices = {}},
                {.key = "pwm_freq",
                 .label = "PWM frequency",
                 .zh = "PWM 频率档",
                 .unit = "",
                 .access = ParamAccess::kEeprom,
                 .area = ParamArea::kEeprom,
                 .address = 87,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 3,
                 .choices = {0, 1, 2, 3}},
                {.key = "blanking_ticks",
                 .label = "Commutation blanking",
                 .zh = "换向消隐拍数",
                 .unit = "",
                 .access = ParamAccess::kEeprom,
                 .area = ParamArea::kEeprom,
                 .address = 88,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 255,
                 .choices = {}},
                {.key = "brake_interlock",
                 .label = "Brake interlock",
                 .zh = "刹车联锁",
                 .unit = "",
                 .access = ParamAccess::kEeprom,
                 .area = ParamArea::kEeprom,
                 .address = 89,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 1,
                 .choices = {0, 1}},
                {.key = "led_mask",
                 .label = "LED mask",
                 .zh = "LED 掩码",
                 .unit = "",
                 .access = ParamAccess::kRam,
                 .area = ParamArea::kSram,
                 .address = 107,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 7,
                 .choices = {}},
                {.key = "led_effect",
                 .label = "LED effect",
                 .zh = "LED 效果（0-8）",
                 .unit = "",
                 .access = ParamAccess::kRam,
                 .area = ParamArea::kSram,
                 .address = 119,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 8,
                 .choices = {}},
                {.key = "collision_flags",
                 .label = "Collision flags",
                 .zh = "碰撞/堵转标志（123 镜像）",
                 .unit = "",
                 .access = ParamAccess::kReadOnly,
                 .area = ParamArea::kSram,
                 .address = 67,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 7,
                 .choices = {}},
                {.key = "collision_threshold",
                 .label = "Collision threshold",
                 .zh = "碰撞阈值",
                 .unit = "step",
                 .access = ParamAccess::kRam,
                 .area = ParamArea::kSram,
                 .address = 124,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 255,
                 .choices = {}},
                {.key = "collision_window",
                 .label = "Collision window",
                 .zh = "碰撞窗口",
                 .unit = "",
                 .access = ParamAccess::kRam,
                 .area = ParamArea::kSram,
                 .address = 125,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 255,
                 .choices = {}},
                {.key = "torque_speed_limit",
                 .label = "Torque speed limit",
                 .zh = "力矩软限速",
                 .unit = "",
                 .access = ParamAccess::kRam,
                 .area = ParamArea::kSram,
                 .address = 126,
                 .bytes = 1,
                 .is_signed = false,
                 .min = 0,
                 .max = 255,
                 .choices = {}},
            };
            return specs;
        }

        const ParamSpec* find_spec(const std::string& key) {
            for (const auto& spec : param_specs()) {
                if (key == spec.key) {
                    return &spec;
                }
            }
            return nullptr;
        }
    } // namespace

    std::vector<ParamDescriptor> UbtechLiqiulong123Driver::parameters() const {
        std::vector<ParamDescriptor> descriptors;
        // 特殊项：id（走确认+重扫流程）、last_error（驱动缓存的最近非零状态字节）。
        descriptors.push_back({.key = "id",
                               .label = "ID",
                               .zh = "舵机 ID",
                               .unit = "",
                               .access = ParamAccess::kEeprom,
                               .area = ParamArea::kEeprom,
                               .address = kRegId,
                               .min = 1,
                               .max = 253,
                               .choices = {}});
        for (const auto& spec : param_specs()) {
            descriptors.push_back({.key = spec.key,
                                   .label = spec.label,
                                   .zh = spec.zh,
                                   .unit = spec.unit,
                                   .access = spec.access,
                                   .area = spec.area,
                                   .address = spec.address,
                                   .min = spec.min,
                                   .max = spec.max,
                                   .choices = spec.choices});
        }
        descriptors.push_back({.key = "last_error",
                               .label = "Last error",
                               .zh = "最近错误位（读取即清）",
                               .unit = "",
                               .access = ParamAccess::kReadOnly,
                               .area = ParamArea::kSram,
                               .address = -1,
                               .min = 0,
                               .max = 255,
                               .choices = {}});
        return descriptors;
    }

    Expected<int> UbtechLiqiulong123Driver::read_param(const ServoId id, const std::string& key) {
        if (!port_.is_open()) {
            return Expected<int>::failure("Serial port is not open");
        }
        if (key == "id") {
            return Expected<int>(id); // 协议无 ID 查询命令，回读即点名 ID
        }
        if (key == "last_error") {
            const int value = last_error_;
            last_error_ = 0;
            return Expected<int>(value);
        }
        const ParamSpec* spec = find_spec(key);
        if (spec == nullptr) {
            return Expected<int>::failure("Unknown parameter '" + key + "'");
        }
        auto value = read_reg(id, static_cast<uint8_t>(spec->address), spec->bytes);
        if (!value) {
            return Expected<int>::failure(value.error());
        }
        if (!spec->is_signed) {
            return value;
        }
        // 有符号量按补码解码（baud_trim / torque_target_current）。
        if (spec->bytes == 1) {
            return Expected<int>(static_cast<int>(static_cast<int8_t>(value.value() & 0xFF)));
        }
        return Expected<int>(static_cast<int>(static_cast<int16_t>(value.value() & 0xFFFF)));
    }

    Result UbtechLiqiulong123Driver::write_param(const ServoId id, const std::string& key,
                                                 const int value) {
        if (!port_.is_open()) {
            return Result::failure("Serial port is not open");
        }
        if (key == "id") {
            if (value < 1 || value > 253) {
                return Result::failure("Servo ID out of range 1..253");
            }
            // 改 ID 流程（对齐 setid.py）：解锁 → 写 5 → 用新 ID 回锁。
            // 固件对改 ID 的应答回新 ID，故显式放宽 ID 匹配。
            if (const auto result = write_reg(id, kRegLock, 0, 1); !result) {
                return result;
            }
            const auto new_id = static_cast<ServoId>(value);
            const auto reply =
                transact(id, kWrite, {kRegId, static_cast<uint8_t>(value)}, false, {new_id});
            if (!reply) {
                return Result::failure(reply.error());
            }
            if (const auto result = write_reg(new_id, kRegLock, 1, 1); !result) {
                return result;
            }
            // 本地缓存随 ID 迁移。
            if (const auto it = variants_.find(id); it != variants_.end()) {
                variants_[new_id] = it->second;
                variants_.erase(it);
            }
            if (const auto it = configs_.find(id); it != configs_.end()) {
                auto config = it->second;
                config.id = new_id;
                configs_.erase(it);
                configs_[new_id] = config;
            }
            return Result::success();
        }
        if (key == "last_error") {
            return Result::failure("Parameter 'last_error' is read-only");
        }
        const ParamSpec* spec = find_spec(key);
        if (spec == nullptr) {
            return Result::failure("Unknown parameter '" + key + "'");
        }
        if (spec->access == ParamAccess::kReadOnly) {
            return Result::failure("Parameter '" + key + "' is read-only");
        }
        if (key == "torque_enable" && value > 1) {
            return Result::failure("Damping mode (2) is not supported by this firmware");
        }
        if (value < spec->min || value > spec->max) {
            return Result::failure("Parameter '" + key + "' value out of range");
        }
        if (key == "work_mode" && value == 2) {
            return Result::failure("Work mode 2 is not supported (0=position 1=wheel 3=torque)");
        }
        if (key == "baud_rate") {
            if (const auto variant = variant_for(id); variant && variant.value() == Variant::kMega8) {
                return Result::failure("Mega8 firmware runs at fixed 921600; baud code is inert");
            }
            // ⚠ 码 4 = 921600（非飞特的 115200）；写入即生效，需按新波特率重连。
            return write_cfg_reg(id, static_cast<uint8_t>(spec->address), value, spec->bytes);
        }
        if (spec->access == ParamAccess::kEeprom) {
            return write_cfg_reg(id, static_cast<uint8_t>(spec->address), value, spec->bytes);
        }
        return write_reg(id, static_cast<uint8_t>(spec->address), value, spec->bytes);
    }

    Expected<ServoInfo> UbtechLiqiulong123Driver::identify(const ServoId id) {
        const auto version = read_reg(id, kRegFwMajor, 2);
        if (!version) {
            return Expected<ServoInfo>::failure(version.error());
        }
        const auto major = static_cast<uint8_t>((version.value() >> 8) & 0xFF);
        const auto minor = static_cast<uint8_t>(version.value() & 0xFF);
        const Variant variant = major == 2 ? Variant::kMega8 : Variant::kSamd10;
        variants_[id] = variant;
        ServoInfo info;
        info.id = id;
        info.model = variant == Variant::kMega8 ? "UBTECH-LQL Mega8" : "UBTECH-LQL SAMD10";
        info.model_number = 0; // 协议无型号寄存器
        info.firmware_major = major;
        info.firmware_minor = minor;
        info.max_range_deg = 360.0;
        info.defaults = default_config(variant, id);
        info.channels = telemetry_channels(id);
        return Expected<ServoInfo>(info);
    }

    Expected<std::vector<ServoInfo>> UbtechLiqiulong123Driver::scan(const int first_id,
                                                                   const int last_id) {
        if (!port_.is_open()) {
            return Expected<std::vector<ServoInfo>>::failure("Serial port is not open");
        }
        std::vector<ServoInfo> infos;
        // ID 0 可寻址（SAMD10 出厂默认），254 为广播——统一扫 0..253。
        for (int raw = std::max(0, first_id); raw <= std::min(253, last_id); ++raw) {
            const auto id = static_cast<ServoId>(raw);
            if (!transact(id, kPing, {}, true)) {
                continue;
            }
            if (const auto info = identify(id)) {
                infos.push_back(info.value());
            }
            else {
                // PING 通但版本读失败：保留行，型号未知。
                ServoInfo fallback;
                fallback.id = id;
                fallback.model = "UBTECH-LQL";
                fallback.max_range_deg = 360.0;
                fallback.defaults = default_config(Variant::kSamd10, id);
                fallback.channels = telemetry_channels(id);
                infos.push_back(fallback);
            }
        }
        return Expected<std::vector<ServoInfo>>(infos);
    }

    Expected<ServoState> UbtechLiqiulong123Driver::read_state(const ServoId id) {
        // 一次读 56..70（15 字节）：位置/速度/负载/电压/温度/电流/移动/碰撞。
        const auto variant = variant_for(id);
        const Variant v = variant ? variant.value() : Variant::kSamd10;
        const auto reply = transact(id, kRead, {kRegPresentPos, 15}, true);
        if (!reply) {
            return Expected<ServoState>::failure(reply.error());
        }
        const auto& p = reply.value().params;
        if (p.size() < 15) {
            return Expected<ServoState>::failure("Short feedback block");
        }
        const auto u16 = [&p](const size_t i) {
            return static_cast<uint16_t>((p[i] << 8) | p[i + 1]);
        };
        const int position = u16(0);
        // mega8 的速度/负载是补码（带符号）；samd10 是幅值。
        const int speed = v == Variant::kMega8
                              ? static_cast<int>(static_cast<int16_t>(u16(2)))
                              : static_cast<int>(u16(2));
        double load;
        if (v == Variant::kMega8) {
            load = std::clamp(static_cast<int>(static_cast<int16_t>(u16(4))) / 255.0, -1.0,
                              1.0);
        }
        else {
            load = std::clamp(u16(4) / 200.0, 0.0, 1.0);
        }
        constexpr double nan = std::numeric_limits<double>::quiet_NaN();
        // 不支持的通道按能力填 NaN（mega8 电压/温度/电流为假值），UI 显示 "--"。
        const auto channels = telemetry_channels(id);
        const double voltage = channels.voltage ? p[6] / 10.0 : nan;
        const double temperature = channels.temperature ? p[7] : nan;
        const double current = channels.current ? u16(13) / 100.0 : nan;

        const auto& config = config_for(id);
        ServoState state;
        state.id = id;
        state.position_ticks = position;
        state.position_deg = servo_core::ticks_to_degrees(config, position);
        state.velocity_ticks_s = speed;
        state.velocity_deg_s = servo_core::ticks_s_to_deg_s(config, speed);
        state.load = load;
        state.voltage = voltage;
        state.temperature = temperature;
        state.current = current;
        state.moving = p[10] != 0;
        return Expected<ServoState>(state);
    }

    Result UbtechLiqiulong123Driver::write_goal(const ServoId id, const int position_ticks,
                                                const int time_ms, const int speed_ticks_s,
                                                const int acc, const int lock_ms) {
        (void)lock_ms; // 协议无锁定时间概念
        const auto& config = config_for(id);
        const int lower = std::clamp(static_cast<int>(std::lround(config.min_ticks)), 0, 4095);
        const int upper = std::clamp(static_cast<int>(std::lround(config.max_ticks)), 0, 4095);
        const int position = std::clamp(position_ticks, lower, upper);
        const int time = std::clamp(time_ms, 0, 65535);
        const int speed = std::clamp(speed_ticks_s, 0, 1000);
        if (acc > 0) {
            // 41 为锁门例外，无需解锁。
            if (const auto result = write_reg(id, kRegAccel, std::clamp(acc, 0, 255), 1);
                !result) {
                return result;
            }
        }
        // 42..47 必须一帧写（位置+时间+速度），否则速度会被清。
        const std::vector<uint8_t> params{
            kRegGoal,
            static_cast<uint8_t>(position >> 8), static_cast<uint8_t>(position & 0xFF),
            static_cast<uint8_t>(time >> 8),     static_cast<uint8_t>(time & 0xFF),
            static_cast<uint8_t>(speed >> 8),    static_cast<uint8_t>(speed & 0xFF)};
        const auto reply = transact(id, kWrite, params, false);
        return reply ? Result::success() : Result::failure(reply.error());
    }

    Result UbtechLiqiulong123Driver::sync_write_goals(const std::vector<ServoId>& ids,
                                                      const std::vector<int>& positions_ticks,
                                                      const std::vector<int>& times_ms,
                                                      const std::vector<int>& speeds_ticks_s,
                                                      const int acc) {
        if (ids.size() != positions_ticks.size() || ids.size() != times_ms.size() ||
            ids.size() != speeds_ticks_s.size()) {
            return Result::failure(
                "sync_write_goals requires matching ids, positions, times and speeds");
        }
        if (ids.empty()) {
            return Result::success();
        }
        if (acc > 0) {
            for (const auto id : ids) {
                if (const auto result = write_reg(id, kRegAccel, std::clamp(acc, 0, 255), 1);
                    !result) {
                    return result;
                }
            }
        }
        // SYNC_WRITE 分批：mega8 RX 128B ≈ 16 颗/帧。
        for (size_t begin = 0; begin < ids.size(); begin += kSyncWriteChunk) {
            const size_t end = std::min(ids.size(), begin + kSyncWriteChunk);
            std::vector<uint8_t> params{kRegGoal, 6};
            for (size_t i = begin; i < end; ++i) {
                const auto& config = config_for(ids[i]);
                const int lower =
                    std::clamp(static_cast<int>(std::lround(config.min_ticks)), 0, 4095);
                const int upper =
                    std::clamp(static_cast<int>(std::lround(config.max_ticks)), 0, 4095);
                const int position = std::clamp(positions_ticks[i], lower, upper);
                const int time = std::clamp(times_ms[i], 0, 65535);
                const int speed = std::clamp(speeds_ticks_s[i], 0, 1000);
                params.push_back(ids[i]);
                params.push_back(static_cast<uint8_t>(position >> 8));
                params.push_back(static_cast<uint8_t>(position & 0xFF));
                params.push_back(static_cast<uint8_t>(time >> 8));
                params.push_back(static_cast<uint8_t>(time & 0xFF));
                params.push_back(static_cast<uint8_t>(speed >> 8));
                params.push_back(static_cast<uint8_t>(speed & 0xFF));
            }
            if (const auto reply = transact(kBroadcastId, kSyncWrite, params, false); !reply) {
                return Result::failure(reply.error());
            }
        }
        return Result::success();
    }

    bool UbtechLiqiulong123Driver::supports_staged_write() const {
        // REG_WRITE/ACTION 仅 SAMD10 固件实现；Mega8 静默忽略。
        if (variants_.empty()) {
            return true;
        }
        return std::ranges::any_of(variants_,
                                   [](const auto& entry) { return entry.second == Variant::kSamd10; });
    }

    Result UbtechLiqiulong123Driver::stage_goal(const ServoId id, const int position_ticks,
                                                const int time_ms, const int speed_ticks_s,
                                                const int acc) {
        if (const auto variant = variant_for(id); variant && variant.value() == Variant::kMega8) {
            return Result::failure("Mega8 firmware does not implement REG_WRITE/ACTION");
        }
        const auto& config = config_for(id);
        const int lower = std::clamp(static_cast<int>(std::lround(config.min_ticks)), 0, 4095);
        const int upper = std::clamp(static_cast<int>(std::lround(config.max_ticks)), 0, 4095);
        const int position = std::clamp(position_ticks, lower, upper);
        const int time = std::clamp(time_ms, 0, 65535);
        const int speed = std::clamp(speed_ticks_s, 0, 1000);
        if (acc > 0) {
            if (const auto result = write_reg(id, kRegAccel, std::clamp(acc, 0, 255), 1);
                !result) {
                return result;
            }
        }
        const std::vector<uint8_t> params{
            kRegGoal,
            static_cast<uint8_t>(position >> 8), static_cast<uint8_t>(position & 0xFF),
            static_cast<uint8_t>(time >> 8),     static_cast<uint8_t>(time & 0xFF),
            static_cast<uint8_t>(speed >> 8),    static_cast<uint8_t>(speed & 0xFF)};
        const auto reply = transact(id, kRegWrite, params, false);
        return reply ? Result::success() : Result::failure(reply.error());
    }

    Result UbtechLiqiulong123Driver::trigger_staged() {
        const auto reply = transact(kBroadcastId, kAction, {}, false);
        return reply ? Result::success() : Result::failure(reply.error());
    }

    Result UbtechLiqiulong123Driver::set_torque(const ServoId id, const bool enabled) {
        // 40 为锁门例外（0=松力 1=锁紧；固件无阻尼档）。
        return write_reg(id, kRegTorqueEnable, enabled ? 1 : 0, 1);
    }

    Expected<std::vector<uint8_t>> UbtechLiqiulong123Driver::read_registers(const ServoId id,
                                                                           const uint8_t address,
                                                                           const size_t length) {
        if (length == 0 || length > kMaxReadData) {
            return Expected<std::vector<uint8_t>>::failure(
                "Read length must be 1..32 (firmware reply cap)");
        }
        const auto reply =
            transact(id, kRead, {address, static_cast<uint8_t>(length)}, true);
        if (!reply) {
            return Expected<std::vector<uint8_t>>::failure(reply.error());
        }
        return Expected<std::vector<uint8_t>>(reply.value().params);
    }

    Result UbtechLiqiulong123Driver::write_registers(const ServoId id, const uint8_t address,
                                                     const std::vector<uint8_t>& data) {
        if (data.empty()) {
            return Result::failure("Empty register write");
        }
        // 对齐 setreg.py 口径：原始写也自动解锁（锁门下固件静默拒写 cfg 寄存器）。
        const bool touch_lock = address != kRegLock;
        if (touch_lock) {
            if (const auto result = write_reg(id, kRegLock, 0, 1); !result) {
                return result;
            }
        }
        std::vector<uint8_t> params{address};
        params.insert(params.end(), data.begin(), data.end());
        if (const auto reply = transact(id, kWrite, params, false); !reply) {
            return Result::failure(reply.error());
        }
        if (touch_lock) {
            if (const auto result = write_reg(id, kRegLock, 1, 1); !result) {
                return result;
            }
            return persist_if_mega8(id);
        }
        return Result::success();
    }

    void UbtechLiqiulong123Driver::register_driver() {
        servo_core::DriverRegistry::instance().add("ubtech_liqiulong123", [] {
            return std::make_unique<UbtechLiqiulong123Driver>();
        });
    }

} // namespace servo_backends
