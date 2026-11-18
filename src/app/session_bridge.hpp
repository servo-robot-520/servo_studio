#pragma once

#include <QObject>
#include <QString>
#include <QVector>
#include <functional>
#include <memory>

#include "servo_core/bus_session.hpp"

// Qt 元类型声明：跨线程 queued 投递所需。
Q_DECLARE_METATYPE(QVector<servo_core::ServoState>)
Q_DECLARE_METATYPE(QVector<servo_core::ServoInfo>)

namespace app {

    /// @brief 会话桥：把 BusSession 工作线程回调转成 Qt 信号，
    ///        由 Qt queued connection 投递到 GUI 线程。
    ///
    /// UI 层只面向本类与中立类型，不接触 IServoDriver；
    /// 析构时停止并 join 会话线程，保证回调不再触达本对象。
    class SessionBridge : public QObject {
        Q_OBJECT

    public:
        explicit SessionBridge(QObject* parent = nullptr);
        ~SessionBridge() override;

        /// @brief 绑定总线会话并安装回调。
        void attach(std::shared_ptr<servo_core::BusSession> session);

        /// @brief 取得会话。
        [[nodiscard]] std::shared_ptr<servo_core::BusSession> session() const;

        /// @brief 是否已绑定会话。
        [[nodiscard]] bool attached() const;

        /// @brief 发起总线扫描，结果经 scanFinished 信号返回。
        void scan(int first_id, int last_id);

        /// @brief 设置轮询集合与频率（Hz，0 停止）。
        void setPolling(const std::vector<servo_core::ServoId>& ids, int hz);

        /// @brief 提交异步命令，结果经 commandFinished 信号返回。
        void submit(std::function<servo_core::Result(servo_core::IServoDriver&)> command);

    signals:
        /// 日志（级别 + 英文消息），工作线程触发。
        void logReceived(const QString& level, const QString& message);
        /// 轮询状态批次，工作线程触发。
        void statesReceived(const QString& port, const QVector<servo_core::ServoState>& states);
        /// 扫描结果，工作线程触发；失败时列表为空并伴随 error 日志。
        void scanFinished(const QString& port, const QVector<servo_core::ServoInfo>& infos);
        /// 命令完成，工作线程触发。
        void commandFinished(bool success, const QString& error);
        /// 会话运行状态变化。
        void runningChanged(bool running);

    public slots:
        /// 停止会话（停止线程并关闭总线）。
        void stop();

    private:
        std::shared_ptr<servo_core::BusSession> session_;
        QString port_;
    };

} // namespace app
