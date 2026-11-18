#pragma once

#include <QMainWindow>
#include <map>
#include <memory>

#include "app/session_bridge.hpp"
#include "servo_core/bus_session.hpp"

namespace app {

    class ConnectionPanel;
    class DeviceTree;
    class StatusTable;
    class ChartPanel;

    /// @brief 主窗口：顶部连接面板，左侧设备树，右侧实时状态表。
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
        void connectBus(const QString& port, int baudRate, int timeoutMs);
        void disconnectBus(const QString& port);
        void scanBus(const QString& port);
        void stopAll(const QString& port);
        void setPollRate(int hz);
        void onServoSelected();

    private:
        struct BusEntry {
            std::shared_ptr<servo_core::BusSession> session;
            SessionBridge* bridge;
        };

        void log(const QString& level, const QString& message);
        void applyPolling(const QString& port);

        ConnectionPanel* connection_panel_;
        DeviceTree* device_tree_;
        StatusTable* status_table_;
        ChartPanel* chart_panel_;
        int poll_hz_{50};
        std::map<QString, BusEntry> buses_;
    };

} // namespace app
