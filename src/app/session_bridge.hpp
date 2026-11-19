#pragma once

#include <QObject>
#include <QString>
#include <QVector>
#include <functional>
#include <memory>

#include "app/qt_types.hpp"
#include "servo_core/bus_session.hpp"

namespace app {

    /// @brief 参数读取结果（含描述表元数据），供参数编辑器渲染。
    struct ParamValue {
        /// 键名。
        QString key;
        /// 英文显示名。
        QString label;
        /// 中文显示名。
        QString zh;
        /// 单位。
        QString unit;
        /// 存储区域（EPROM/SRAM/DEFAULT）。
        QString area;
        /// 寄存器地址。
        int address{-1};
        /// 最小取值。
        int min{};
        /// 最大取值。
        int max{};
        /// 当前值；读取失败时无效。
        int value{};
        /// 读取是否成功。
        bool valid{false};
        /// 是否只读。
        bool read_only{false};
        /// 是否 EEPROM 参数。
        bool eeprom{false};
        /// 枚举取值表；空为自由取值。
        QVector<int> choices;
    };

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
        void setPolling(const std::vector<servo_core::ServoId>& ids, int hz) const;

        /// @brief 提交异步命令；结果经 commandFinished 信号返回，
        ///        done 为可选的完成回调（工作线程触发）。
        void submit(std::function<servo_core::Result(servo_core::IServoDriver&)> command,
                    std::function<void(bool success, const QString& error)> done = {});

        /// @brief 读取全部参数，结果经 parametersReceived 信号返回。
        void readParameters(const QString& port, servo_core::ServoId id);

        /// @brief 备份整表参数到 JSON 文件（工作线程读取并写文件）。
        void backupParameters(const QString& port, servo_core::ServoId id, const QString& path);

        /// @brief 恢复参数（调用方已用 paramsForRestore 剔除 id/波特率）。
        void restoreParameters(const QString& port, servo_core::ServoId id,
                               const QMap<QString, int>& params);

        /// @brief 查询后端是否支持预存式下发，结果经 stagedSupportKnown 返回。
        void queryStagedSupport(const QString& port);

        /// @brief 帧级调试入口（工作线程/驱动钩子调用），转 frameTraced 信号。
        void traceFrame(const char* direction, const std::vector<uint8_t>& frame);

    signals:
        /// 日志（级别 + 英文消息），工作线程触发。
        void logReceived(const QString& level, const QString& message);
        /// 轮询状态批次，工作线程触发。
        void statesReceived(const QString& port, const QVector<servo_core::ServoState>& states);
        /// 扫描结果，工作线程触发；失败时列表为空并伴随 error 日志。
        void scanFinished(const QString& port, const QVector<servo_core::ServoInfo>& infos);
        /// 命令完成，工作线程触发。
        void commandFinished(bool success, const QString& error);
        /// 参数读取结果，工作线程触发。
        void parametersReceived(const QString& port, int id, const QVector<ParamValue>& values);
        /// 参数备份完成，工作线程触发。
        void backupFinished(const QString& path, bool success, const QString& error);
        /// 参数恢复完成，工作线程触发。
        void restoreFinished(const QString& port, int id, int written, bool success,
                             const QString& error);
        /// 帧级收发（tx/rx + 原始字节），工作线程触发。
        void frameTraced(const QString& port, const QString& direction, const QByteArray& frame);
        /// 预存能力探测结果，工作线程触发。
        void stagedSupportKnown(const QString& port, bool supported);
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

Q_DECLARE_METATYPE(QVector<app::ParamValue>)
