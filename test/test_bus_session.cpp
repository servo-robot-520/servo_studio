#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "servo_core/bus_session.hpp"
#include "servo_core/driver.hpp"

namespace servo_core {
    namespace {

        /// @brief 记录调用的假驱动，验证会话的串行化与回调行为。
        class FakeDriver : public IServoDriver {
        public:
            Result open(const BusConfig&) override {
                if (fail_open) {
                    return Result::failure("open failed");
                }
                open_ = true;
                return Result::success();
            }

            void close() override {
                open_ = false;
            }

            [[nodiscard]] bool is_open() const override {
                return open_;
            }

            Result set_servo_config(const ServoConfig&) override {
                return Result::success();
            }

            Expected<std::vector<ServoInfo>> scan(int, int) override {
                std::vector<ServoInfo> infos{{.id = 1, .model = "fake", .model_number = 1,
                                              .firmware_major = 1, .firmware_minor = 0}};
                return Expected<std::vector<ServoInfo>>(infos);
            }

            Expected<ServoInfo> identify(ServoId id) override {
                ServoInfo info;
                info.id = id;
                info.model = "fake";
                return Expected<ServoInfo>(info);
            }

            Expected<ServoState> read_state(ServoId id) override {
                record_call();
                ServoState state;
                state.id = id;
                state.position_ticks = static_cast<double>(++state_counter);
                return Expected<ServoState>(state);
            }

            Result write_goal(ServoId, int, int, int, int) override {
                record_call();
                return Result::success();
            }

            Result sync_write_goals(const std::vector<ServoId>&, const std::vector<int>&, int,
                                    const std::vector<int>&, int) override {
                record_call();
                return Result::success();
            }

            Result set_torque(ServoId, bool) override {
                record_call();
                return Result::success();
            }

            [[nodiscard]] std::vector<ParamDescriptor> parameters() const override {
                return {};
            }

            Expected<int> read_param(ServoId, const std::string&) override {
                return Expected<int>(0);
            }

            Result write_param(ServoId, const std::string&, int) override {
                return Result::success();
            }

            Expected<std::vector<uint8_t>> read_registers(ServoId, uint8_t, size_t) override {
                return Expected<std::vector<uint8_t>>(std::vector<uint8_t>{});
            }

            Result write_registers(ServoId, uint8_t, const std::vector<uint8_t>&) override {
                return Result::success();
            }

            /// 所有驱动调用必须发生在同一个线程（工作线程串行化）。
            void record_call() {
                std::lock_guard<std::mutex> lock(mutex);
                const auto id = std::this_thread::get_id();
                if (!call_thread_valid) {
                    call_thread = id;
                    call_thread_valid = true;
                }
                else {
                    EXPECT_EQ(call_thread, id);
                }
                ++call_count;
            }

            std::mutex mutex;
            std::thread::id call_thread{};
            bool call_thread_valid{false};
            std::atomic<int> call_count{0};
            std::atomic<int> state_counter{0};
            bool fail_open{false};

        private:
            std::atomic<bool> open_{false};
        };

        /// @brief 轮询谓词，直到其为真或超时。
        template <typename Predicate>
        bool wait_for(Predicate predicate,
                      const std::chrono::milliseconds timeout = std::chrono::milliseconds(2000)) {
            const auto deadline = std::chrono::steady_clock::now() + timeout;
            while (std::chrono::steady_clock::now() < deadline) {
                if (predicate()) {
                    return true;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            return predicate();
        }

        TEST(BusSessionTest, CommandsRunOnWorkerThreadWithCallbacks) {
            auto driver = std::make_unique<FakeDriver>();
            FakeDriver* const raw = driver.get();
            BusSession session(std::move(driver), BusConfig{.port = "fake"});
            ASSERT_TRUE(session.start());

            std::atomic<int> done_count{0};
            std::atomic<int> order{0};
            std::atomic<int> first_seen{-1};
            std::atomic<int> second_seen{-1};
            session.submit([&order, &first_seen](IServoDriver& driver) {
                             first_seen = ++order;
                             return driver.set_torque(1, true);
                         },
                           [&done_count](const Result& result) {
                               EXPECT_TRUE(result);
                               ++done_count;
                           });
            session.submit([&order, &second_seen](IServoDriver& driver) {
                             second_seen = ++order;
                             return driver.set_torque(1, false);
                         },
                           [&done_count](const Result& result) {
                               EXPECT_TRUE(result);
                               ++done_count;
                           });

            ASSERT_TRUE(wait_for([&] { return done_count == 2; }));
            // 命令按提交顺序串行执行。
            EXPECT_EQ(first_seen, 1);
            EXPECT_EQ(second_seen, 2);
            EXPECT_GE(raw->call_count, 2);
            session.stop();
        }

        TEST(BusSessionTest, PollingDeliversStates) {
            auto driver = std::make_unique<FakeDriver>();
            BusSession session(std::move(driver), BusConfig{.port = "fake"});
            ASSERT_TRUE(session.start());

            std::mutex mutex;
            std::vector<std::vector<ServoState>> batches;
            session.set_states_handler([&mutex, &batches](const std::vector<ServoState>& states) {
                std::lock_guard<std::mutex> lock(mutex);
                batches.push_back(states);
            });
            session.set_polling({1, 2}, 50);

            ASSERT_TRUE(wait_for([&] {
                std::lock_guard<std::mutex> lock(mutex);
                return batches.size() >= 3;
            }));
            std::lock_guard<std::mutex> lock(mutex);
            ASSERT_FALSE(batches.empty());
            // 每批包含全部轮询 id。
            EXPECT_EQ(batches.front().size(), 2u);
            session.stop();
        }

        TEST(BusSessionTest, LateSetPollingStartsPolling) {
            // 回归：工作线程已处于空闲等待后才设置轮询，
            // 配置变化通知不得被等待谓词吞掉（否则轮询永不启动）。
            auto driver = std::make_unique<FakeDriver>();
            BusSession session(std::move(driver), BusConfig{.port = "fake"});
            ASSERT_TRUE(session.start());
            std::atomic<int> batches{0};
            session.set_states_handler(
                [&batches](const std::vector<ServoState>&) { ++batches; });

            // 还原竞态窗口：先让工作线程进入空闲等待，再配置轮询。
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            session.set_polling({1}, 50);

            EXPECT_TRUE(wait_for([&] { return batches.load() >= 2; }));
            session.stop();
        }

        TEST(BusSessionTest, ScanPassesResultsThroughCallback) {
            auto driver = std::make_unique<FakeDriver>();
            BusSession session(std::move(driver), BusConfig{.port = "fake"});
            ASSERT_TRUE(session.start());

            std::atomic<bool> called{false};
            std::vector<ServoInfo> received;
            session.scan(0, 10, [&](const Expected<std::vector<ServoInfo>>& infos) {
                EXPECT_TRUE(infos);
                if (infos) {
                    received = infos.value();
                }
                called = true;
            });
            ASSERT_TRUE(wait_for([&] { return called.load(); }));
            ASSERT_EQ(received.size(), 1u);
            EXPECT_EQ(received.front().model, "fake");
            session.stop();
        }

        TEST(BusSessionTest, StartReportsOpenFailure) {
            auto driver = std::make_unique<FakeDriver>();
            driver->fail_open = true;
            BusSession session(std::move(driver), BusConfig{.port = "fake"});
            const auto result = session.start();
            EXPECT_FALSE(result);
            EXPECT_FALSE(session.running());
        }

        TEST(BusSessionTest, StopIsIdempotent) {
            auto driver = std::make_unique<FakeDriver>();
            BusSession session(std::move(driver), BusConfig{.port = "fake"});
            ASSERT_TRUE(session.start());
            session.stop();
            session.stop();
            EXPECT_FALSE(session.running());
        }

    } // namespace
} // namespace servo_core
