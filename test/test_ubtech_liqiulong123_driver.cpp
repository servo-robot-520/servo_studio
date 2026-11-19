#include "fake_liqiulong_servo.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "servo_backends/ubtech_liqiulong123_driver.hpp"
#include "servo_core/registry.hpp"

namespace {

    using servo_backends::UbtechLiqiulong123Driver;
    using servo_backends::test::FakeLiqiulongBus;
    using servo_core::BusConfig;
    using servo_core::ServoConfig;

    FakeLiqiulongBus::Servo samd10_servo(const uint8_t id) {
        FakeLiqiulongBus::Servo servo;
        servo.id = id;
        servo.samd10 = true;
        servo.version = {1, 11};
        return servo;
    }

    FakeLiqiulongBus::Servo mega8_servo(const uint8_t id) {
        FakeLiqiulongBus::Servo servo;
        servo.id = id;
        servo.samd10 = false;
        servo.version = {2, 3};
        return servo;
    }

    class LqDriverTest : public ::testing::Test {
    protected:
        void open_driver(std::vector<FakeLiqiulongBus::Servo> servos) {
            bus_ = std::make_unique<FakeLiqiulongBus>(std::move(servos));
            driver_ = std::make_unique<UbtechLiqiulong123Driver>();
            BusConfig config;
            config.port = bus_->slave_path();
            config.baud_rate = 1000000;
            config.response_timeout_ms = 60;
            ASSERT_TRUE(driver_->open(config));
            driver_->set_trace_hook(
                [this](const char* direction, const std::vector<uint8_t>& frame) {
                    if (std::string(direction) == "tx") {
                        tx_frames_.push_back(frame);
                    }
                });
        }

        std::unique_ptr<FakeLiqiulongBus> bus_;
        std::unique_ptr<UbtechLiqiulong123Driver> driver_;
        std::vector<std::vector<uint8_t>> tx_frames_;
    };

    TEST_F(LqDriverTest, ScanDetectsVariantsAndDefaultGeometry) {
        open_driver({samd10_servo(1), mega8_servo(2)});
        const auto infos = driver_->scan(0, 10);
        ASSERT_TRUE(infos);
        ASSERT_EQ(infos.value().size(), 2u);

        const auto& samd10 = infos.value()[0];
        EXPECT_EQ(samd10.id, 1);
        EXPECT_EQ(samd10.model, "UBTECH-LQ SAMD10");
        EXPECT_EQ(samd10.firmware_major, 1);
        EXPECT_EQ(samd10.firmware_minor, 11);
        EXPECT_EQ(samd10.max_range_deg, 360.0);
        EXPECT_EQ(samd10.defaults.center_ticks, 2048.0);
        EXPECT_EQ(samd10.defaults.full_scale_ticks, 4096.0);
        EXPECT_EQ(samd10.defaults.min_ticks, 540.0);
        EXPECT_EQ(samd10.defaults.max_ticks, 3800.0);

        const auto& mega8 = infos.value()[1];
        EXPECT_EQ(mega8.model, "UBTECH-LQ Mega8");
        EXPECT_EQ(mega8.firmware_major, 2);
        EXPECT_EQ(mega8.firmware_minor, 3);
        EXPECT_EQ(mega8.defaults.min_ticks, 24.0);
        EXPECT_EQ(mega8.defaults.max_ticks, 4080.0);
    }

    TEST_F(LqDriverTest, ScanSkipsAbsentIds) {
        open_driver({samd10_servo(5)});
        const auto infos = driver_->scan(0, 10);
        ASSERT_TRUE(infos);
        ASSERT_EQ(infos.value().size(), 1u);
        EXPECT_EQ(infos.value()[0].id, 5);
    }

    TEST_F(LqDriverTest, IdentifyFailsWhenSilent) {
        auto silent = samd10_servo(1);
        silent.silent = true;
        open_driver({silent});
        EXPECT_FALSE(driver_->identify(1));
    }

    TEST_F(LqDriverTest, WriteGoalUsesSingleFrame42To47) {
        open_driver({samd10_servo(1)});
        ASSERT_TRUE(driver_->write_goal(1, 3000, 500, 800, 0, 0));
        const auto servo = bus_->servo_state(1);
        // 42..47 一帧写入：位置 3000、时间 500ms、速度 800。
        ASSERT_EQ(servo.writes.size(), 1u);
        const std::vector<uint8_t> expected = {42, 0x0B, 0xB8, 0x01, 0xF4, 0x03, 0x20};
        EXPECT_EQ(servo.writes[0], expected);
    }

    TEST_F(LqDriverTest, WriteGoalClampsToConfigAndWritesAccelFirst) {
        open_driver({samd10_servo(1)});
        ServoConfig config;
        config.id = 1;
        config.min_ticks = 540;
        config.max_ticks = 3800;
        config.center_ticks = 2048;
        config.full_scale_ticks = 4096;
        ASSERT_TRUE(driver_->set_servo_config(config));
        ASSERT_TRUE(driver_->write_goal(1, 9999, 99999, 99999, 5, 0));
        const auto servo = bus_->servo_state(1);
        ASSERT_EQ(servo.writes.size(), 2u);
        // 先写加速度 41=5（锁门豁免），再写目标帧（位置钳 3800，时间钳 65535，速度钳 1000）。
        EXPECT_EQ(servo.writes[0], std::vector<uint8_t>({41, 5}));
        EXPECT_EQ(servo.writes[1],
                  std::vector<uint8_t>({42, 0x0E, 0xD8, 0xFF, 0xFF, 0x03, 0xE8}));

        ASSERT_TRUE(driver_->write_goal(1, -5, 0, 0, 0, 0));
        EXPECT_EQ(bus_->servo_state(1).writes.back(),
                  std::vector<uint8_t>({42, 0x02, 0x1C, 0x00, 0x00, 0x00, 0x00}));
    }

    TEST_F(LqDriverTest, TolerantStatusBitsCachedInLastError) {
        auto servo = samd10_servo(1);
        servo.status_inject = 0x20; // 堵转位
        open_driver({servo});
        // 非零状态字节不判失败。
        const auto state = driver_->read_state(1);
        ASSERT_TRUE(state);
        const auto error = driver_->read_param(1, "last_error");
        ASSERT_TRUE(error);
        EXPECT_EQ(error.value(), 0x20);
        const auto cleared = driver_->read_param(1, "last_error");
        ASSERT_TRUE(cleared);
        EXPECT_EQ(cleared.value(), 0);
    }

    TEST_F(LqDriverTest, ReadStateDecodesSamd10Feedback) {
        open_driver({samd10_servo(1)});
        // 位置 3000、速度 800、负载 100、电压 90、温度 41、电流 150、移动中。
        bus_->set_regs(1, 56, {0x0B, 0xB8, 0x03, 0x20, 0x00, 0x64, 90, 41, 0, 0, 1, 0, 0, 0x00,
                               150});
        const auto state = driver_->read_state(1);
        ASSERT_TRUE(state);
        EXPECT_EQ(state.value().position_ticks, 3000);
        EXPECT_NEAR(state.value().position_deg, (3000 - 2048) * 360.0 / 4096, 1e-9);
        EXPECT_EQ(state.value().velocity_ticks_s, 800);
        EXPECT_DOUBLE_EQ(state.value().load, 0.5);
        EXPECT_DOUBLE_EQ(state.value().voltage, 9.0);
        EXPECT_DOUBLE_EQ(state.value().temperature, 41.0);
        EXPECT_DOUBLE_EQ(state.value().current, 1.5);
        EXPECT_TRUE(state.value().moving);
    }

    TEST_F(LqDriverTest, ReadStateDecodesMega8SignedAndFakesAsNaN) {
        open_driver({mega8_servo(2)});
        // mega8：速度/负载为补码（-100、-128），电压/温度/电流为假值。
        bus_->set_regs(2, 56, {0x0B, 0xB8, 0xFF, 0x9C, 0xFF, 0x80, 74, 25, 0, 0, 0, 0, 0, 0x00,
                               0x00});
        const auto state = driver_->read_state(2);
        ASSERT_TRUE(state);
        EXPECT_EQ(state.value().velocity_ticks_s, -100);
        EXPECT_NEAR(state.value().load, -128 / 255.0, 1e-9);
        EXPECT_TRUE(std::isnan(state.value().voltage));
        EXPECT_TRUE(std::isnan(state.value().temperature));
        EXPECT_TRUE(std::isnan(state.value().current));
        EXPECT_FALSE(state.value().moving);
    }

    TEST_F(LqDriverTest, CfgWriteUnlocksRelocksAndPersistsOnMega8) {
        open_driver({samd10_servo(1), mega8_servo(2)});
        ASSERT_TRUE(driver_->write_param(1, "p_gain", 55));
        const auto samd10 = bus_->servo_state(1);
        ASSERT_EQ(samd10.writes.size(), 3u);
        EXPECT_EQ(samd10.writes[0], std::vector<uint8_t>({48, 0}));
        EXPECT_EQ(samd10.writes[1], std::vector<uint8_t>({21, 55}));
        EXPECT_EQ(samd10.writes[2], std::vector<uint8_t>({48, 1}));
        EXPECT_EQ(samd10.save_count, 0); // samd10 自动 Flash，无 49 触发

        ASSERT_TRUE(driver_->write_param(2, "p_gain", 55));
        const auto mega8 = bus_->servo_state(2);
        EXPECT_EQ(mega8.save_count, 1); // mega8 自动补 49=0xA5
    }

    TEST_F(LqDriverTest, IdChangeFlowAcceptsNewIdReplyAndMigratesCache) {
        open_driver({samd10_servo(1)});
        ASSERT_TRUE(driver_->write_param(1, "id", 9));
        EXPECT_EQ(bus_->servo_state(9).id, 9);
        // 回锁发给新 ID；后续用新 ID 访问。
        const auto value = driver_->read_param(9, "p_gain");
        ASSERT_TRUE(value);
    }

    TEST_F(LqDriverTest, BaudRateMapsCodeFourTo921600AndMega8IsFixed) {
        open_driver({samd10_servo(1), mega8_servo(2)});
        for (const auto& descriptor : driver_->parameters()) {
            if (descriptor.key == "baud_rate") {
                ASSERT_EQ(descriptor.choices.size(), 5u);
                EXPECT_EQ(descriptor.choices[4], 921600); // ⚠非飞特的 115200
            }
        }
        ASSERT_TRUE(driver_->write_param(1, "baud_rate", 4));
        EXPECT_EQ(bus_->servo_state(1).regs[6], 4);
        const auto result = driver_->write_param(2, "baud_rate", 1);
        ASSERT_FALSE(result);
        EXPECT_NE(result.error().find("921600"), std::string::npos);
    }

    TEST_F(LqDriverTest, StagedWriteSupportedOnlyOnSamd10) {
        open_driver({mega8_servo(2)});
        ASSERT_TRUE(driver_->identify(2));
        EXPECT_FALSE(driver_->supports_staged_write());
        EXPECT_FALSE(driver_->stage_goal(2, 1000, 0, 0, 0));
        EXPECT_EQ(bus_->servo_state(2).regwrite_count, 0);
    }

    TEST_F(LqDriverTest, StagedWriteWorksOnSamd10) {
        open_driver({samd10_servo(1)});
        ASSERT_TRUE(driver_->identify(1));
        EXPECT_TRUE(driver_->supports_staged_write());
        ASSERT_TRUE(driver_->stage_goal(1, 2000, 100, 0, 0));
        EXPECT_EQ(bus_->servo_state(1).regwrite_count, 1);
        ASSERT_TRUE(driver_->trigger_staged());
        EXPECT_EQ(bus_->servo_state(1).action_count, 1);
    }

    TEST_F(LqDriverTest, SyncWriteChunksAtSixteenServos) {
        std::vector<FakeLiqiulongBus::Servo> servos;
        std::vector<servo_core::ServoId> ids;
        std::vector<int> positions;
        for (int i = 1; i <= 20; ++i) {
            servos.push_back(samd10_servo(static_cast<uint8_t>(i)));
            ids.push_back(static_cast<uint8_t>(i));
            positions.push_back(1500 + i);
        }
        open_driver(servos);
        ASSERT_TRUE(driver_->sync_write_goals(ids, positions, std::vector<int>(20, 0),
                                              std::vector<int>(20, 0), 0));
        const auto [frames, max_ids] = bus_->sync_stats();
        EXPECT_EQ(frames, 2);   // 16 + 4
        EXPECT_EQ(max_ids, 16); // mega8 RX 128B 上限
        EXPECT_EQ(bus_->servo_state(20).regs[42], 0x05); // 1520 = 0x05F0 高字节
        EXPECT_EQ(bus_->servo_state(20).regs[43], 0xF0);
    }

    TEST_F(LqDriverTest, TorqueIsTwoStateAndSetTorqueMapsCleanly) {
        open_driver({samd10_servo(1)});
        const auto result = driver_->write_param(1, "torque_enable", 2);
        ASSERT_FALSE(result);
        EXPECT_NE(result.error().find("Damping"), std::string::npos);
        ASSERT_TRUE(driver_->set_torque(1, true));
        EXPECT_EQ(bus_->servo_state(1).regs[40], 1);
        ASSERT_TRUE(driver_->set_torque(1, false));
        EXPECT_EQ(bus_->servo_state(1).regs[40], 0);
    }

    TEST_F(LqDriverTest, ReadOnlyAndUnknownParamsRejected) {
        open_driver({samd10_servo(1)});
        EXPECT_FALSE(driver_->write_param(1, "last_error", 5));
        EXPECT_FALSE(driver_->write_param(1, "direction_selftest", 1));
        EXPECT_FALSE(driver_->write_param(1, "collision_flags", 1));
        EXPECT_FALSE(driver_->write_param(1, "nope", 1));
        EXPECT_FALSE(driver_->read_param(1, "nope"));
    }

    TEST_F(LqDriverTest, ReadRegistersEnforcesReplyCapAndWriteUnlocks) {
        open_driver({samd10_servo(1)});
        EXPECT_FALSE(driver_->read_registers(1, 0, 33)); // 固件 READ 上限 32 字节
        const auto version = driver_->read_registers(1, 3, 2);
        ASSERT_TRUE(version);
        EXPECT_EQ(version.value(), std::vector<uint8_t>({1, 11}));

        ASSERT_TRUE(driver_->write_registers(1, 21, {77}));
        const auto servo = bus_->servo_state(1);
        EXPECT_EQ(servo.regs[21], 77);
        // 原始写也走解锁/回锁（对齐 setreg.py）。
        EXPECT_EQ(servo.writes.front(), std::vector<uint8_t>({48, 0}));
        EXPECT_EQ(servo.writes.back(), std::vector<uint8_t>({48, 1}));
    }

    TEST_F(LqDriverTest, CorruptChecksumFailsRead) {
        auto servo = samd10_servo(1);
        servo.corrupt = true;
        open_driver({servo});
        const auto result = driver_->read_param(1, "p_gain");
        ASSERT_FALSE(result);
        EXPECT_NE(result.error().find("checksum"), std::string::npos);
    }

    TEST_F(LqDriverTest, RegisterDriverIsCreatableByName) {
        UbtechLiqiulong123Driver::register_driver();
        auto driver = servo_core::DriverRegistry::instance().create("ubtech_liqiulong123");
        ASSERT_NE(driver, nullptr);
    }

} // namespace
