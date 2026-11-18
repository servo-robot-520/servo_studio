#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "fake_scs_servo.hpp"
#include "servo_backends/feetech_driver.hpp"

namespace servo_backends {
    namespace {

        using feetech_servo_ros2_control::kModelSc215;
        using feetech_servo_ros2_control::test::FakeScsBus;
        using servo_core::ServoConfig;

        constexpr double kPi = 3.14159265358979323846;

        servo_core::BusConfig bus_config(const std::string& port) {
            servo_core::BusConfig config;
            config.port = port;
            config.baud_rate = 1000000;
            config.response_timeout_ms = 50;
            return config;
        }

        TEST(FeetechDriverTest, ScanAndIdentify) {
            FakeScsBus bus({1, 2});
            FeetechDriver driver;
            ASSERT_TRUE(driver.open(bus_config(bus.slave_path())));

            const auto infos = driver.scan(0, 5);
            ASSERT_TRUE(infos);
            ASSERT_EQ(infos.value().size(), 2u);
            EXPECT_EQ(infos.value()[0].id, 1);
            EXPECT_EQ(infos.value()[0].model, "SC215");
            EXPECT_EQ(infos.value()[0].firmware_major, 3);
            EXPECT_EQ(infos.value()[0].firmware_minor, 24);
            EXPECT_EQ(infos.value()[1].id, 2);

            const auto identity = driver.identify(2);
            ASSERT_TRUE(identity);
            EXPECT_EQ(identity.value().model_number, kModelSc215);
        }

        TEST(FeetechDriverTest, ReadStateConvertsUnits) {
            FakeScsBus bus({1});
            bus.set_present_position(1, 600);
            bus.set_present_speed(1, 100);
            bus.set_present_load(1, static_cast<uint16_t>((1U << 10) | 300U));
            bus.set_present_current(1, 20);
            FeetechDriver driver;
            ASSERT_TRUE(driver.open(bus_config(bus.slave_path())));

            ServoConfig config;
            config.id = 1;
            config.range_deg = 300.0;
            ASSERT_TRUE(driver.set_servo_config(config));

            const auto state = driver.read_state(1);
            ASSERT_TRUE(state);
            EXPECT_DOUBLE_EQ(state.value().position_ticks, 600.0);
            EXPECT_NEAR(state.value().position_deg, (600 - 511) * 300.0 / 1024.0, 1e-9);
            EXPECT_DOUBLE_EQ(state.value().velocity_ticks_s, 100.0);
            EXPECT_NEAR(state.value().velocity_deg_s, 100.0 * 300.0 / 1024.0, 1e-9);
            EXPECT_DOUBLE_EQ(state.value().load, -0.3);
            EXPECT_NEAR(state.value().voltage, 7.4, 1e-9);
            EXPECT_NEAR(state.value().temperature, 30.0, 1e-9);
            EXPECT_NEAR(state.value().current, 20 * 0.0065, 1e-9);
            EXPECT_FALSE(state.value().moving);
        }

        TEST(FeetechDriverTest, WriteGoalClampsToConfig) {
            FakeScsBus bus({1});
            FeetechDriver driver;
            ASSERT_TRUE(driver.open(bus_config(bus.slave_path())));

            ServoConfig config;
            config.id = 1;
            config.min_ticks = 100;
            config.max_ticks = 900;
            ASSERT_TRUE(driver.set_servo_config(config));

            ASSERT_TRUE(driver.write_goal(1, 50, 0, 1000));
            EXPECT_EQ(bus.present_position(1), 100);
            ASSERT_TRUE(driver.write_goal(1, 950, 0, 1000));
            EXPECT_EQ(bus.present_position(1), 900);
            ASSERT_TRUE(driver.write_goal(1, 512, 0, 1000));
            EXPECT_EQ(bus.present_position(1), 512);
        }

        TEST(FeetechDriverTest, SyncWriteGoalsBroadcasts) {
            FakeScsBus bus({1, 2});
            FeetechDriver driver;
            ASSERT_TRUE(driver.open(bus_config(bus.slave_path())));

            ASSERT_TRUE(driver.sync_write_goals({1, 2}, {200, 300}, 0, {500, 600}));
            // SYNC WRITE 无应答，等待模拟总线消费该帧。
            ASSERT_TRUE(feetech_servo_ros2_control::test::wait_for([&] {
                return bus.present_position(1) == 200 && bus.present_position(2) == 300;
            }));

            // id/位置/速度数量不匹配必须被拒绝。
            EXPECT_FALSE(driver.sync_write_goals({1, 2}, {200}, 0, {500, 600}));
        }

        TEST(FeetechDriverTest, WriteParamUsesEepromSafeFlow) {
            FakeScsBus bus({1});
            FeetechDriver driver;
            ASSERT_TRUE(driver.open(bus_config(bus.slave_path())));

            ASSERT_TRUE(driver.write_param(1, "p_coefficient", 20));
            EXPECT_EQ(bus.register_value(1, feetech_servo_ros2_control::kRegisterPCoefficient), 20);
            EXPECT_FALSE(bus.torque_enabled(1));
            EXPECT_EQ(bus.register_value(1, feetech_servo_ros2_control::kRegisterEepromLock), 1);

            // 范围外与未知键必须被拒绝。
            EXPECT_FALSE(driver.write_param(1, "max_temperature", 200));
            EXPECT_FALSE(driver.write_param(1, "no_such_param", 0));
        }

        TEST(FeetechDriverTest, ReadParamReadsBytesAndWords) {
            FakeScsBus bus({1});
            FeetechDriver driver;
            ASSERT_TRUE(driver.open(bus_config(bus.slave_path())));

            const auto p_gain = driver.read_param(1, "p_coefficient");
            ASSERT_TRUE(p_gain);
            EXPECT_EQ(p_gain.value(), 15);

            const auto max_torque = driver.read_param(1, "max_torque");
            ASSERT_TRUE(max_torque);
            EXPECT_EQ(max_torque.value(), 1000);

            EXPECT_FALSE(driver.read_param(1, "no_such_param"));
        }

        TEST(FeetechDriverTest, ParametersExposeDescriptorTable) {
            FeetechDriver driver;
            const auto parameters = driver.parameters();
            ASSERT_EQ(parameters.size(), 26u);
            // 描述表含 ID 与波特率，供 GUI 参数编辑器渲染。
            bool has_id = false;
            bool has_baud = false;
            for (const auto& parameter : parameters) {
                has_id |= parameter.key == "id";
                has_baud |= parameter.key == "baud_rate";
            }
            EXPECT_TRUE(has_id);
            EXPECT_TRUE(has_baud);
        }

    } // namespace
} // namespace servo_backends
