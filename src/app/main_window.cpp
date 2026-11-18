#include "app/main_window.hpp"

#include <QCloseEvent>
#include <QSplitter>
#include <QStatusBar>
#include <QVBoxLayout>
#include <cstdio>

#include "app/panels/connection_panel.hpp"
#include "app/panels/device_tree.hpp"
#include "app/panels/status_table.hpp"
#include "servo_core/registry.hpp"

namespace app {

    namespace {
        // 状态表轮询频率（Hz）；曲线与频率调节在 M4 提供。
        constexpr int kPollHz = 50;
    }

    MainWindow::MainWindow(QWidget* parent) :
        QMainWindow(parent) {
        setWindowTitle(tr("servo_studio"));
        resize(1200, 720);

        auto* central = new QWidget(this);
        auto* layout = new QVBoxLayout(central);
        layout->setContentsMargins(10, 10, 10, 10);
        layout->setSpacing(10);

        connection_panel_ = new ConnectionPanel(central);
        layout->addWidget(connection_panel_);

        auto* splitter = new QSplitter(Qt::Horizontal, central);
        device_tree_ = new DeviceTree(splitter);
        device_tree_->setMinimumWidth(320);
        status_table_ = new StatusTable(splitter);
        splitter->addWidget(device_tree_);
        splitter->addWidget(status_table_);
        splitter->setStretchFactor(0, 0);
        splitter->setStretchFactor(1, 1);
        layout->addWidget(splitter, 1);

        setCentralWidget(central);
        statusBar()->showMessage(tr("就绪"));

        connect(connection_panel_, &ConnectionPanel::connectRequested, this, &MainWindow::connectBus);
        connect(connection_panel_, &ConnectionPanel::disconnectRequested, this,
                &MainWindow::disconnectBus);
        connect(connection_panel_, &ConnectionPanel::scanRequested, this, &MainWindow::scanBus);
        connect(connection_panel_, &ConnectionPanel::stopAllRequested, this, &MainWindow::stopAll);
    }

    MainWindow::~MainWindow() {
        for (auto& [port, entry] : buses_) {
            entry.session->stop();
        }
    }

    void MainWindow::closeEvent(QCloseEvent* event) {
        for (auto& [port, entry] : buses_) {
            entry.session->stop();
        }
        QMainWindow::closeEvent(event);
    }

    void MainWindow::log(const QString& level, const QString& message) {
        // 日志输出英文原文到 stderr，界面状态栏显示简要信息。
        std::fprintf(stderr, "[%s] %s\n", level.toStdString().c_str(),
                     message.toStdString().c_str());
        if (level == "error" || level == "warn") {
            statusBar()->showMessage(message, 8000);
        }
        else {
            statusBar()->showMessage(message, 4000);
        }
    }

    void MainWindow::connectBus(const QString& port, const int baudRate, const int timeoutMs) {
        if (port.isEmpty()) {
            log("error", "No serial port selected");
            return;
        }
        if (buses_.count(port) != 0) {
            log("warn", "Already connected to " + port);
            return;
        }
        auto driver = servo_core::DriverRegistry::instance().create("feetech_scs");
        if (!driver) {
            log("error", "Driver 'feetech_scs' is not registered");
            return;
        }
        servo_core::BusConfig config;
        config.port = port.toStdString();
        config.baud_rate = baudRate;
        config.response_timeout_ms = timeoutMs;

        auto session = std::make_shared<servo_core::BusSession>(std::move(driver), config);
        auto* bridge = new SessionBridge(this);
        bridge->attach(session);

        connect(bridge, &SessionBridge::logReceived, this, &MainWindow::log);
        connect(bridge, &SessionBridge::statesReceived, this,
                [this](const QString& bus_port, const QVector<servo_core::ServoState>& states) {
                    status_table_->updateStates(bus_port, states);
                });
        connect(bridge, &SessionBridge::scanFinished, this,
                [this](const QString& bus_port, const QVector<servo_core::ServoInfo>& infos) {
                    device_tree_->setServos(bus_port, infos);
                    std::vector<servo_core::ServoId> ids;
                    ids.reserve(infos.size());
                    for (const auto& info : infos) {
                        ids.push_back(info.id);
                    }
                    if (const auto it = buses_.find(bus_port); it != buses_.end()) {
                        it->second.bridge->setPolling(ids, kPollHz);
                    }
                    statusBar()->showMessage(tr("%1：发现 %2 个舵机").arg(bus_port).arg(infos.size()),
                                             6000);
                });
        connect(bridge, &SessionBridge::commandFinished, this,
                [this](const bool success, const QString& error) {
                    if (!success) {
                        log("error", error);
                    }
                });

        if (const auto result = session->start(); !result) {
            log("error", QString::fromStdString(result.error()));
            bridge->deleteLater();
            return;
        }
        buses_[port] = BusEntry{session, bridge};
        device_tree_->addBus(port);
        device_tree_->setBusConnected(port, true);
        connection_panel_->setConnected(port, true);
        log("info", "Connected to " + port);
    }

    void MainWindow::disconnectBus(const QString& port) {
        const auto it = buses_.find(port);
        if (it == buses_.end()) {
            return;
        }
        it->second.session->stop();
        it->second.bridge->deleteLater();
        buses_.erase(it);
        status_table_->removeBus(port);
        device_tree_->removeBus(port);
        connection_panel_->setConnected(port, false);
        log("info", "Disconnected from " + port);
    }

    void MainWindow::scanBus(const QString& port) {
        const auto it = buses_.find(port);
        if (it == buses_.end()) {
            log("error", "Not connected to " + port);
            return;
        }
        statusBar()->showMessage(tr("正在扫描 %1 ...").arg(port));
        it->second.bridge->scan(0, 253);
    }

    void MainWindow::stopAll(const QString& port) {
        const auto it = buses_.find(port);
        if (it == buses_.end()) {
            return;
        }
        const auto ids = device_tree_->servoIds(port);
        it->second.bridge->submit([ids](servo_core::IServoDriver& driver) {
            for (const auto id : ids) {
                if (const auto result = driver.set_torque(id, false); !result) {
                    return result;
                }
            }
            return servo_core::Result::success();
        });
        log("info", "Torque released on " + port);
    }

} // namespace app
