#include "fake_ubtech_servo.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "servo_backends/ubtech_driver.hpp"
#include "servo_core/registry.hpp"

namespace {

    using servo_backends::UbtechDriver;
    using servo_backends::test::FakeUbtechBus;
    using servo_core::BusConfig;
    using servo_core::ServoConfig;

    uint8_t checksum(const std::vector<uint8_t>& frame) {
        uint8_t sum = 0;
        for (int i = 2; i <= 7; ++i) {
            sum = static_cast<uint8_t>(sum + frame[i]);
        }
        return sum;
    }

    /// 构造期望的主机命令帧（含校验和与帧尾）。
    std::vector<uint8_t> expect_frame(const uint8_t h1, const uint8_t h2, const uint8_t id,
                                      const uint8_t cmd, const uint8_t p1h, const uint8_t p1l,
                                      const uint8_t p2h, const uint8_t p2l) {
        std::vector<uint8_t> frame = {h1, h2, id, cmd, p1h, p1l, p2h, p2l, 0, 0xED};
        frame[8] = checksum(frame);
        return frame;
    }

    class UbtechDriverTest : public ::testing::Test {
    protected:
        void open_driver(std::vector<FakeUbtechBus::Servo> servos) {
            bus_ = std::make_unique<FakeUbtechBus>(std::move(servos));
            driver_ = std::make_unique<UbtechDriver>();
            BusConfig config;
            config.port = bus_->slave_path();
            config.baud_rate = 115200;
            config.response_timeout_ms = 60;
            ASSERT_TRUE(driver_->open(config));
            driver_->set_trace_hook(
                [this](const char* direction, const std::vector<uint8_t>& frame) {
                    if (std::string(direction) == "tx") {
                        tx_frames_.push_back(frame);
                    }
                });
        }

        static FakeUbtechBus::Servo plain_servo(const uint8_t id) {
            FakeUbtechBus::Servo servo;
            servo.id = id;
            return servo;
        }

        std::unique_ptr<FakeUbtechBus> bus_;
        std::unique_ptr<UbtechDriver> driver_;
        std::vector<std::vector<uint8_t>> tx_frames_;
    };

    TEST_F(UbtechDriverTest, MoveEncodesAngleMoveTimeAndLock) {
        open_driver({plain_servo(1)});
        ASSERT_TRUE(driver_->write_goal(1, 180, 1000, 0, 0, 500));
        // 角度 180=0xB4；运动 1000ms/20=50=0x32；锁定 500ms/20=25=0x0019。
        EXPECT_EQ(tx_frames_.back(),
                  expect_frame(0xFA, 0xAF, 0x01, 0x01, 0xB4, 0x32, 0x00, 0x19));
        const auto servo = bus_->servo_state(1);
        EXPECT_EQ(servo.last_angle, 180);
        EXPECT_EQ(servo.last_move_units, 50);
        EXPECT_EQ(servo.last_lock_units, 25);
    }

    TEST_F(UbtechDriverTest, MoveClampsToProtocolLimits) {
        open_driver({plain_servo(1)});
        ASSERT_TRUE(driver_->write_goal(1, 999, 999999, 0, 0, 999999));
        // 角度钳位 240；运动时间钳位 255×20ms；锁定钳位 3270×20ms（0x0CC6）。
        EXPECT_EQ(tx_frames_.back(),
                  expect_frame(0xFA, 0xAF, 0x01, 0x01, 240, 255, 0x0C, 0xC6));
        ASSERT_TRUE(driver_->write_goal(1, -5, 0, 0, 0, 0));
        EXPECT_EQ(tx_frames_.back(), expect_frame(0xFA, 0xAF, 0x01, 0x01, 0, 0, 0, 0));
    }

    TEST_F(UbtechDriverTest, MoveHonorsServoConfigClamp) {
        open_driver({plain_servo(1)});
        ServoConfig config;
        config.id = 1;
        config.min_ticks = 10;
        config.max_ticks = 100;
        ASSERT_TRUE(driver_->set_servo_config(config));
        ASSERT_TRUE(driver_->write_goal(1, 200, 0, 0, 0, 0));
        EXPECT_EQ(tx_frames_.back(), expect_frame(0xFA, 0xAF, 0x01, 0x01, 100, 0, 0, 0));
    }

    TEST_F(UbtechDriverTest, MoveAcceptsBothAckForms) {
        auto plain = plain_servo(1);
        auto with_id = plain_servo(2);
        with_id.ack_with_id = true;
        open_driver({plain, with_id});
        EXPECT_TRUE(driver_->write_goal(1, 10, 0, 0, 0, 0));
        EXPECT_TRUE(driver_->write_goal(2, 10, 0, 0, 0, 0));
    }

    TEST_F(UbtechDriverTest, MoveSilentServoFails) {
        auto silent = plain_servo(3);
        silent.silent = true;
        open_driver({silent});
        const auto result = driver_->write_goal(3, 10, 0, 0, 0, 0);
        ASSERT_FALSE(result);
        EXPECT_NE(result.error().find("No response"), std::string::npos);
    }

    TEST_F(UbtechDriverTest, StopSendsStopMarkerAndMapsToTorqueRelease) {
        open_driver({plain_servo(1)});
        ASSERT_TRUE(driver_->set_torque(1, false));
        EXPECT_EQ(tx_frames_.back(), expect_frame(0xFA, 0xAF, 0x01, 0x01, 0xFF, 0, 0, 0));
        EXPECT_TRUE(bus_->servo_state(1).stopped);
        // 无法凭命令上电锁紧。
        const auto result = driver_->set_torque(1, true);
        ASSERT_FALSE(result);
        EXPECT_NE(result.error().find("torque"), std::string::npos);
    }

    TEST_F(UbtechDriverTest, CapabilitiesDeclareNoTelemetryAndProtocolPanel) {
        open_driver({plain_servo(1)});
        const auto channels = driver_->telemetry_channels(1);
        EXPECT_FALSE(channels.position);
        EXPECT_FALSE(channels.velocity);
        EXPECT_FALSE(channels.load);
        EXPECT_FALSE(channels.voltage);
        EXPECT_FALSE(channels.temperature);
        EXPECT_FALSE(channels.current);
        const auto caps = driver_->capabilities();
        EXPECT_FALSE(caps.torque_damping);
        EXPECT_FALSE(caps.polling_safe);
        EXPECT_TRUE(caps.uses_protocol_panel);
    }

    TEST_F(UbtechDriverTest, ReadAngleFillsStateAndMovingFlag) {
        auto servo = plain_servo(1);
        servo.target_angle = 200;
        servo.actual_angle = 190;
        open_driver({servo});
        const auto state = driver_->read_state(1);
        ASSERT_TRUE(state);
        EXPECT_EQ(state.value().position_ticks, 190);
        EXPECT_EQ(state.value().position_deg, 190);
        EXPECT_TRUE(state.value().moving);
        // 其余通道无遥测，填 NaN（UI 显示 "--"）。
        EXPECT_TRUE(std::isnan(state.value().velocity_deg_s));
        EXPECT_TRUE(std::isnan(state.value().load));
        EXPECT_TRUE(std::isnan(state.value().voltage));
        EXPECT_TRUE(std::isnan(state.value().temperature));
        EXPECT_TRUE(std::isnan(state.value().current));
    }

    TEST_F(UbtechDriverTest, ReadAngleSettledClearsMovingFlag) {
        auto servo = plain_servo(1);
        servo.target_angle = 90;
        servo.actual_angle = 90;
        open_driver({servo});
        const auto state = driver_->read_state(1);
        ASSERT_TRUE(state);
        EXPECT_FALSE(state.value().moving);
    }

    TEST_F(UbtechDriverTest, ReadAngleSilentFails) {
        auto silent = plain_servo(1);
        silent.silent = true;
        open_driver({silent});
        EXPECT_FALSE(driver_->read_state(1));
    }

    TEST_F(UbtechDriverTest, SetIdSendsNewIdInParameterSlot) {
        open_driver({plain_servo(1)});
        ASSERT_TRUE(driver_->write_param(1, "id", 7));
        EXPECT_EQ(tx_frames_.back(), expect_frame(0xFA, 0xAF, 0x01, 0xCD, 0x00, 0x07, 0, 0));
    }

    TEST_F(UbtechDriverTest, SetIdRejectedFails) {
        auto servo = plain_servo(1);
        servo.reject = true;
        open_driver({servo});
        const auto result = driver_->write_param(1, "id", 7);
        ASSERT_FALSE(result);
        EXPECT_NE(result.error().find("rejected"), std::string::npos);
    }

    TEST_F(UbtechDriverTest, SetIdValidatesRangeWithoutSending) {
        open_driver({plain_servo(1)});
        EXPECT_FALSE(driver_->write_param(1, "id", 0));
        EXPECT_FALSE(driver_->write_param(1, "id", 241));
        EXPECT_TRUE(tx_frames_.empty());
    }

    TEST_F(UbtechDriverTest, OffsetUsesTwosComplementAndToleratesGarbage) {
        open_driver({plain_servo(1)});
        ASSERT_TRUE(driver_->write_param(1, "angle_offset", -30));
        // -30 的 16 位补码 0xFFE2 位于参数 2（字节 6-7）。
        EXPECT_EQ(tx_frames_.back(), expect_frame(0xFA, 0xAF, 0x01, 0xD2, 0, 0, 0xFF, 0xE2));
        // 回读：模拟器在字节 4-5 填充噪声（协议 XX），仍应正确解析字节 6-7。
        const auto offset = driver_->read_param(1, "angle_offset");
        ASSERT_TRUE(offset);
        EXPECT_EQ(offset.value(), -30);
    }

    TEST_F(UbtechDriverTest, OffsetValidatesRange) {
        open_driver({plain_servo(1)});
        EXPECT_FALSE(driver_->write_param(1, "angle_offset", 91));
        EXPECT_FALSE(driver_->write_param(1, "angle_offset", -91));
    }

    TEST_F(UbtechDriverTest, FirmwareVersionParamsAreReadOnlyBytes) {
        open_driver({plain_servo(1)});
        for (int i = 0; i < 4; ++i) {
            const auto value =
                driver_->read_param(1, "firmware_v" + std::to_string(i + 1));
            ASSERT_TRUE(value) << "firmware_v" << i + 1;
            EXPECT_EQ(value.value(), i + 1);
        }
        const auto result = driver_->write_param(1, "firmware_v1", 9);
        ASSERT_FALSE(result);
        EXPECT_NE(result.error().find("read-only"), std::string::npos);
    }

    TEST_F(UbtechDriverTest, ReadParamIdEchoesAddressedId) {
        open_driver({plain_servo(1)});
        const auto value = driver_->read_param(1, "id");
        ASSERT_TRUE(value);
        EXPECT_EQ(value.value(), 1);
    }

    TEST_F(UbtechDriverTest, UnknownParameterFails) {
        open_driver({plain_servo(1)});
        EXPECT_FALSE(driver_->read_param(1, "nope"));
        EXPECT_FALSE(driver_->write_param(1, "nope", 1));
    }

    TEST_F(UbtechDriverTest, ScanFindsOnlyPresentServos) {
        open_driver({plain_servo(5), plain_servo(9)});
        const auto infos = driver_->scan(0, 253);
        ASSERT_TRUE(infos);
        ASSERT_EQ(infos.value().size(), 2u);
        EXPECT_EQ(infos.value()[0].id, 5);
        EXPECT_EQ(infos.value()[1].id, 9);
        EXPECT_EQ(infos.value()[0].model, "UBTECH");
        EXPECT_EQ(infos.value()[0].firmware_major, 1);
        EXPECT_EQ(infos.value()[0].firmware_minor, 2);
        EXPECT_EQ(infos.value()[0].max_range_deg, 240.0);
        // 区间过滤：6..8 内无舵机。
        const auto none = driver_->scan(6, 8);
        ASSERT_TRUE(none);
        EXPECT_TRUE(none.value().empty());
    }

    TEST_F(UbtechDriverTest, IdentifyReportsVersion) {
        open_driver({plain_servo(5)});
        const auto info = driver_->identify(5);
        ASSERT_TRUE(info);
        EXPECT_EQ(info.value().firmware_major, 1);
        EXPECT_EQ(info.value().firmware_minor, 2);
        EXPECT_FALSE(driver_->identify(7));
    }

    TEST_F(UbtechDriverTest, CorruptResponseChecksumFails) {
        open_driver({plain_servo(1)});
        bus_->set_corrupt_response(true);
        const auto state = driver_->read_state(1);
        ASSERT_FALSE(state);
        EXPECT_NE(state.error().find("checksum"), std::string::npos);
    }

    TEST_F(UbtechDriverTest, SyncWriteSendsSequentialMoves) {
        open_driver({plain_servo(1), plain_servo(2)});
        ASSERT_TRUE(driver_->sync_write_goals({1, 2}, {10, 20}, {100, 200}, {0, 0}, 0));
        ASSERT_EQ(tx_frames_.size(), 2u);
        EXPECT_EQ(tx_frames_[0], expect_frame(0xFA, 0xAF, 0x01, 0x01, 10, 5, 0, 0));
        EXPECT_EQ(tx_frames_[1], expect_frame(0xFA, 0xAF, 0x02, 0x01, 20, 10, 0, 0));
        EXPECT_FALSE(driver_->sync_write_goals({1, 2}, {10}, {100, 200}, {0, 0}, 0));
    }

    TEST_F(UbtechDriverTest, RegisterDriverIsCreatableByName) {
        UbtechDriver::register_driver();
        auto driver = servo_core::DriverRegistry::instance().create("ubtech");
        ASSERT_NE(driver, nullptr);
    }

} // namespace
