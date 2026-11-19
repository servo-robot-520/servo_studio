#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "servo_core/result.hpp"
#include "servo_core/types.hpp"

namespace servo_core {

    /// @brief 舵机驱动抽象：一个实例对应一条串口总线，负责全部厂商差异。
    ///
    /// 实现不要求线程安全；总线访问的串行化由 BusSession 承担。
    /// 错误信息一律使用英文。
    class IServoDriver {
    public:
        /// @brief 帧级调试钩子：方向（"tx"/"rx"）+ 原始字节。
        using TraceHook =
            std::function<void(const char* direction, const std::vector<uint8_t>& frame)>;

        virtual ~IServoDriver() = default;

        /// @brief 设置帧级调试钩子（可为空表示清除）；应在 start 前安装。
        virtual void set_trace_hook(TraceHook hook) {
            (void)hook;
        }

        /// @brief 打开总线。
        virtual Result open(const BusConfig& config) = 0;

        /// @brief 关闭总线；未打开时为空操作。
        virtual void close() = 0;

        /// @brief 总线是否已打开。
        [[nodiscard]] virtual bool is_open() const = 0;

        /// @brief 注册/更新参与换算与钳位的舵机配置。
        virtual Result set_servo_config(const ServoConfig& config) = 0;

        /// @brief 扫描区间内在线的舵机并读取身份。
        /// @param first_id 起始 id（含）。
        /// @param last_id 结束 id（含）。
        virtual Expected<std::vector<ServoInfo>> scan(int first_id, int last_id) = 0;

        /// @brief 读取单个舵机身份。
        virtual Expected<ServoInfo> identify(ServoId id) = 0;

        /// @brief 读取实时状态（位置/速度含原始计数与换算值）。
        virtual Expected<ServoState> read_state(ServoId id) = 0;

        /// @brief 写目标位置（原始计数，按配置钳位）、运行时间、速度与加速度。
        ///        acc 仅对带加速度寄存器的系列有意义（SMS/STS）；SCS 系列忽略。
        virtual Result write_goal(ServoId id, int position_ticks, int time_ms,
                                  int speed_ticks_s, int acc) = 0;

        /// @brief 多舵机同步下发目标位置（SYNC WRITE 类操作）。
        ///        acc 语义同 write_goal。
        virtual Result sync_write_goals(const std::vector<ServoId>& ids,
                                        const std::vector<int>& positions_ticks, int time_ms,
                                        const std::vector<int>& speeds_ticks_s,
                                        int acc) = 0;

        /// @brief 扭矩开关。
        virtual Result set_torque(ServoId id, bool enabled) = 0;

        /// @brief 参数描述表（键、显示名、单位、范围、存储区）。
        [[nodiscard]] virtual std::vector<ParamDescriptor> parameters() const = 0;

        /// @brief 按键读取参数原始值。
        virtual Expected<int> read_param(ServoId id, const std::string& key) = 0;

        /// @brief 按键写入参数；EEPROM 参数自动走安全写流程（关扭矩→解锁→写→回锁）。
        virtual Result write_param(ServoId id, const std::string& key, int value) = 0;

        /// @brief 原始寄存器读（高级功能）。
        virtual Expected<std::vector<uint8_t>> read_registers(ServoId id, uint8_t address,
                                                              size_t length) = 0;

        /// @brief 原始寄存器写（高级功能，不做安全流程）。
        virtual Result write_registers(ServoId id, uint8_t address,
                                       const std::vector<uint8_t>& data) = 0;
    };

} // namespace servo_core
