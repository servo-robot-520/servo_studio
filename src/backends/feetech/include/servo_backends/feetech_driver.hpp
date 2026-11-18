#pragma once

#include <map>
#include <memory>

#include "feetech_servo_ros2_control/scs_protocol.hpp"
#include "feetech_servo_ros2_control/scs_serial_port.hpp"
#include "servo_core/driver.hpp"

namespace servo_backends {

    using servo_core::Expected;
    using servo_core::IServoDriver;
    using servo_core::ParamDescriptor;
    using servo_core::Result;
    using servo_core::ServoConfig;
    using servo_core::ServoId;
    using servo_core::ServoInfo;
    using servo_core::ServoState;

    /// @brief 飞特 SCS 后端：把 feetech_servo_driver 的串口/协议层
    ///        适配到 servo_core::IServoDriver。
    ///
    /// 单位换算（tick ↔ 度）复用驱动侧 scs_register_map 的换算函数；
    /// 参数读写按 scs_params.hpp 的描述表透传，EEPROM 参数自动走安全写流程。
    class FeetechDriver : public IServoDriver {
    public:
        ~FeetechDriver() override;

        Result open(const servo_core::BusConfig& config) override;
        void close() override;
        [[nodiscard]] bool is_open() const override;

        Result set_servo_config(const ServoConfig& config) override;

        Expected<std::vector<ServoInfo>> scan(int first_id, int last_id) override;
        Expected<ServoInfo> identify(ServoId id) override;
        Expected<ServoState> read_state(ServoId id) override;

        Result write_goal(ServoId id, int position_ticks, int time_ms, int speed_ticks_s) override;
        Result sync_write_goals(const std::vector<ServoId>& ids,
                                const std::vector<int>& positions_ticks, int time_ms,
                                const std::vector<int>& speeds_ticks_s) override;

        Result set_torque(ServoId id, bool enabled) override;

        [[nodiscard]] std::vector<ParamDescriptor> parameters() const override;
        Expected<int> read_param(ServoId id, const std::string& key) override;
        Result write_param(ServoId id, const std::string& key, int value) override;

        Expected<std::vector<uint8_t>> read_registers(ServoId id, uint8_t address,
                                                      size_t length) override;
        Result write_registers(ServoId id, uint8_t address,
                               const std::vector<uint8_t>& data) override;

        /// @brief 注册到 DriverRegistry，名称 "feetech_scs"。
        static void register_driver();

    private:
        [[nodiscard]] const ServoConfig& config_for(ServoId id) const;
        static ServoInfo to_info(ServoId id,
                                 const feetech_servo_ros2_control::ScsServoIdentity& identity);

        feetech_servo_ros2_control::ScsSerialPort port_;
        std::unique_ptr<feetech_servo_ros2_control::ScsProtocol> protocol_;
        std::map<ServoId, ServoConfig> configs_;
    };

} // namespace servo_backends
