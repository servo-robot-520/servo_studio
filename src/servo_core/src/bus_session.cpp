#include "servo_core/bus_session.hpp"

#include <chrono>

namespace servo_core {

    BusSession::BusSession(std::unique_ptr<IServoDriver> driver, BusConfig config) :
        driver_(std::move(driver)), config_(std::move(config)) {
    }

    BusSession::~BusSession() {
        stop();
    }

    void BusSession::set_log_handler(LogHandler handler) {
        std::lock_guard<std::mutex> lock(mutex_);
        log_handler_ = std::move(handler);
    }

    void BusSession::set_states_handler(StatesHandler handler) {
        std::lock_guard<std::mutex> lock(mutex_);
        states_handler_ = std::move(handler);
    }

    Result BusSession::start() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (running_) {
            return Result::failure("BusSession is already running");
        }
        if (const auto result = driver_->open(config_); !result) {
            return result;
        }
        stop_requested_ = false;
        running_ = true;
        thread_ = std::thread([this] { thread_main(); });
        return Result::success();
    }

    void BusSession::stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!thread_.joinable()) {
                return;
            }
            stop_requested_ = true;
        }
        cv_.notify_all();
        if (thread_.joinable()) {
            thread_.join();
        }
        driver_->close();
        running_ = false;
    }

    bool BusSession::running() const {
        return running_;
    }

    void BusSession::submit(std::function<Result(IServoDriver&)> command, DoneHandler done) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            commands_.emplace_back([command = std::move(command), done = std::move(done)](IServoDriver& driver) mutable {
                    const auto result = command(driver);
                    if (done) {
                        done(result);
                    }
                });
        }
        cv_.notify_all();
    }

    void BusSession::set_polling(std::vector<ServoId> ids, const int hz) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            poll_ids_ = std::move(ids);
            poll_hz_ = (poll_ids_.empty() || hz <= 0) ? 0 : hz;
            poll_config_changed_ = true;
        }
        cv_.notify_all();
    }

    void BusSession::scan(const int first_id, const int last_id,
                          std::function<void(const Expected<std::vector<ServoInfo>>&)> done) {
        if (!done) {
            return;
        }
        submit([first_id, last_id, done = std::move(done)](IServoDriver& driver) {
            done(driver.scan(first_id, last_id));
            return Result::success();
        });
    }

    void BusSession::log(const std::string& level, const std::string& message) {
        LogHandler handler;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            handler = log_handler_;
        }
        if (handler) {
            handler(level, message);
        }
    }

    void BusSession::thread_main() {
        log("info", "bus worker thread started");
        auto next_poll = std::chrono::steady_clock::now();
        for (;;) {
            std::function<void(IServoDriver&)> command;
            std::vector<ServoId> poll_ids;
            int poll_hz = 0;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                if (stop_requested_) {
                    break;
                }
                poll_config_changed_ = false;
                if (!commands_.empty()) {
                    command = std::move(commands_.front());
                    commands_.pop_front();
                }
                else {
                    poll_ids = poll_ids_;
                    poll_hz = poll_hz_;
                    const auto wake = [this] {
                        return stop_requested_ || !commands_.empty() || poll_config_changed_;
                    };
                    if (poll_hz > 0) {
                        std::chrono::microseconds(1'000'000 / static_cast<long long>(poll_hz));
                        // 等到命令到达 / 停止 / 轮询配置变化 / 轮询到期；
                        // 被唤醒则回到循环头重新评估。
                        if (cv_.wait_until(lock, next_poll, wake)) {
                            continue;
                        }
                    }
                    else {
                        cv_.wait(lock, wake);
                        continue;
                    }
                }
            }

            if (command) {
                command(*driver_);
                continue;
            }

            // 轮询到期：逐个读取状态（同样在本线程串行执行）。
            std::vector<ServoState> states;
            states.reserve(poll_ids.size());
            for (const auto id : poll_ids) {
                if (const auto state = driver_->read_state(id)) {
                    states.push_back(state.value());
                }
                else {
                    log("warn", "poll read_state(" + std::to_string(static_cast<int>(id)) +
                            ") failed: " + state.error());
                }
            }
            StatesHandler handler;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                handler = states_handler_;
            }
            if (handler && !states.empty()) {
                handler(states);
            }

            const auto period =
                std::chrono::microseconds(1'000'000 / static_cast<long long>(poll_hz));
            next_poll += period;
            // 避免长时间阻塞后连发积压轮询。
            if (const auto now = std::chrono::steady_clock::now(); next_poll < now) {
                next_poll = now + period;
            }
        }
        log("info", "bus worker thread stopped");
    }

} // namespace servo_core
