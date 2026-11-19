#pragma once

#include <QMainWindow>
#include <map>
#include <memory>

#include "app/session_bridge.hpp"
#include "app/panels/batch_dialog.hpp"
#include "app/panels/inspector_panel.hpp"
#include "servo_core/bus_session.hpp"

namespace app {

    class ConnectionPanel;
    class DeviceTree;
    class StatusTable;
    class ChartPanel;
    class InspectorPanel;
    class UbtechPanel;
    class LogPanel;
    class ParamsDialog;
    class BatchDialog;

    /// @brief 主窗口：顶部连接面板，左侧设备树，中间监控区（曲线/状态表），
    ///        右侧检查器，底部日志。
    ///
    /// 窗口只面向 SessionBridge 与中立类型；每条总线一个 BusSession +
    /// SessionBridge，回调经 queued 信号回到 GUI 线程。
    class MainWindow : public QMainWindow {
        Q_OBJECT

    public:
        explicit MainWindow(QWidget* parent = nullptr);
        ~MainWindow() override;

    protected:
        void closeEvent(QCloseEvent* event) override;

    private slots:
        void connectBus(const QString& backend, const QString& port, int baudRate,
                        int timeoutMs);
        void disconnectBus(const QString& port);
        void scanBus(const QString& port);
        void stopAll(const QString& port);
        void setPollRate(int hz);
        void onServoSelected();

        void handleGoal(const QString& port, int ticks, int timeMs, int speed, int acc,
                        GoalTarget target);
        void handleTorque(const QString& port, servo_core::ServoId id, int mode);
        void handleUbtechMove(const QString& port, servo_core::ServoId id, int angleDeg,
                              int moveMs, int lockMs);
        void handleUbtechStop(const QString& port, servo_core::ServoId id);
        void handleUbtechReadAngle(const QString& port, servo_core::ServoId id);
        void onBackendChanged(const QString& backend) const;
        void openParamsDialog();
        void handleParamWrite(const QString& port, servo_core::ServoId id, const QString& key,
                              int value, const QString& valueText);
        void handleParamWriteAll(const QString& port, const QString& key, int value);
        void handleServoConfig(const QString& port, servo_core::ServoId id, double rangeDeg,
                               double minTicks, double maxTicks);
        void handleBackup(const QString& port, servo_core::ServoId id);
        void handleRestore(const QString& port, servo_core::ServoId id);

        void handleBatchSync(const QVector<BatchGoal>& goals);
        void handleBatchStage(const QVector<BatchGoal>& goals);
        void handleBatchTrigger();

    private:
        struct BusEntry {
            std::shared_ptr<servo_core::BusSession> session;
            SessionBridge* bridge;
            /// 驱动名（协议族）：一条串口仅支持一种协议；
            /// 多协议并存时按此过滤“下发到所有舵机”的范围。
            QString driver_name;
        };

        void log(const QString& level, const QString& message) const;
        void applyPolling(const QString& port);
        /// 操作面板按协议二选一显示（UBTECH 专用面板 / 飞特检查器），
        /// UBTECH 无遥测，监控列随之隐藏。
        void applyProtocolPanel(bool is_ubtech) const;
        void refreshBatchDialog();
        [[nodiscard]] servo_core::ServoConfig configFor(const QString& port,
                                                        servo_core::ServoId id) const;

        ConnectionPanel* connection_panel_;
        DeviceTree* device_tree_;
        StatusTable* status_table_;
        ChartPanel* chart_panel_;
        /// 监控列（曲线 + 状态表）；UBTECH 无遥测，选中其舵机时隐藏。
        QWidget* monitor_;
        InspectorPanel* inspector_panel_;
        /// UBTECH 专用操作面板（与检查器同列，按协议二选一显示）。
        UbtechPanel* ubtech_panel_;
        LogPanel* log_panel_;
        ParamsDialog* params_dialog_;
        BatchDialog* batch_dialog_;
        int poll_hz_{50};
        std::map<QString, BusEntry> buses_;
        std::map<QString, QVector<servo_core::ServoInfo>> bus_infos_;
        std::map<QString, std::map<servo_core::ServoId, servo_core::ServoConfig>> configs_;

        // 当前检查器监控对象。
        bool has_subject_{false};
        QString subject_port_;
        servo_core::ServoId subject_id_{};
    };

} // namespace app
