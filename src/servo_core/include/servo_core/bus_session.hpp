#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "servo_core/driver.hpp"

namespace servo_core {

    /// @brief 总线会话：每条总线一个工作线程，串行执行全部驱动调用。
    ///
    /// 轮询（定频读状态）与用户命令共用同一个线程，天然避免总线争用。
    /// 所有回调都在工作线程触发；GUI 需自行跨线程投递
    /// （如 Qt queued connection）。
    class BusSession {
    public:
        /// 日志回调：级别 + 英文消息。
        using LogHandler = std::function<void(const std::string& level, const std::string& message)>;
        /// 轮询状态回调。
        using StatesHandler = std::function<void(const std::vector<ServoState>& states)>;
        /// 命令完成回调。
        using DoneHandler = std::function<void(const Result& result)>;

        /// @brief 构造函数。
        /// @param driver 驱动实例（取得所有权）。
        /// @param config 总线连接配置。
        BusSession(std::unique_ptr<IServoDriver> driver, BusConfig config);
        ~BusSession();

        BusSession(const BusSession&) = delete;
        BusSession& operator=(const BusSession&) = delete;

        /// @brief 设置日志回调。
        void set_log_handler(LogHandler handler);

        /// @brief 设置轮询状态回调。
        void set_states_handler(StatesHandler handler);

        /// @brief 打开总线并启动工作线程。
        Result start();

        /// @brief 停止工作线程并关闭总线；未启动时为空操作。
        void stop();

        /// @brief 会话是否在运行。
        [[nodiscard]] bool running() const;

        /// @brief 提交异步命令，在工作线程串行执行。
        /// @param command 命令体；返回 Result 表示成败。
        /// @param done 完成回调（可为空）。
        void submit(std::function<Result(IServoDriver&)> command, DoneHandler done = {});

        /// @brief 设置轮询集合与频率。
        /// @param ids 参与轮询的舵机 id。
        /// @param hz 轮询频率（Hz）；0 或空集合停止轮询。
        void set_polling(std::vector<ServoId> ids, int hz);

        /// @brief 扫描便捷命令：结果经回调返回（工作线程触发）。
        void scan(int first_id, int last_id,
                  std::function<void(const Expected<std::vector<ServoInfo>>&)> done);

    private:
        void thread_main();
        void log(const std::string& level, const std::string& message);

        std::unique_ptr<IServoDriver> driver_;
        BusConfig config_;
        LogHandler log_handler_;
        StatesHandler states_handler_;

        std::thread thread_;
        std::mutex mutex_;
        std::condition_variable cv_;
        std::deque<std::function<void(IServoDriver&)>> commands_;
        std::vector<ServoId> poll_ids_;
        int poll_hz_{0};
        bool stop_requested_{false};
        std::atomic<bool> running_{false};
    };

} // namespace servo_core
