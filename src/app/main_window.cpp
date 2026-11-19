#include "app/main_window.hpp"

#include <QCloseEvent>
#include <QFile>
#include <QFileDialog>
#include <QMessageBox>
#include <QSplitter>
#include <QStatusBar>
#include <QVBoxLayout>
#include <cstdio>

#include "app/panels/chart_panel.hpp"
#include "app/panels/connection_panel.hpp"
#include "app/panels/device_tree.hpp"
#include "app/panels/inspector_panel.hpp"
#include "app/panels/log_panel.hpp"
#include "app/panels/params_dialog.hpp"
#include "app/panels/status_table.hpp"
#include "app/params_backup.hpp"
#include "servo_core/registry.hpp"

namespace app {

    MainWindow::MainWindow(QWidget* parent) :
        QMainWindow(parent) {
        setWindowTitle(tr("servo_studio"));
        resize(1440, 820);

        auto* central = new QWidget(this);
        auto* layout = new QVBoxLayout(central);
        layout->setContentsMargins(10, 10, 10, 10);
        layout->setSpacing(10);

        // 顶栏一行：协议 / 串口 / 刷新 / 波特率 / 打开。
        connection_panel_ = new ConnectionPanel(central);
        layout->addWidget(connection_panel_->topBar());

        // 三列全高（顶栏下直达窗口底部）：设备 | 监控/日志 | 操作。
        auto* main_split = new QSplitter(Qt::Horizontal, central);

        // 左：侧栏工具条（超时/轮询/扫描/全部松扭矩）+ 设备树。
        auto* left = new QWidget(main_split);
        auto* left_layout = new QVBoxLayout(left);
        left_layout->setContentsMargins(0, 0, 0, 0);
        left_layout->setSpacing(8);
        left_layout->addWidget(connection_panel_->sideBar());
        device_tree_ = new DeviceTree(left);
        device_tree_->setMinimumWidth(260);
        left_layout->addWidget(device_tree_, 1);

        // 中：监控（曲线/状态表）在上，日志在监控下方。
        auto* monitor_col = new QSplitter(Qt::Vertical, main_split);
        auto* monitor = new QSplitter(Qt::Vertical, monitor_col);
        chart_panel_ = new ChartPanel(monitor);
        status_table_ = new StatusTable(monitor);
        monitor->addWidget(chart_panel_);
        monitor->addWidget(status_table_);
        monitor->setStretchFactor(0, 2);
        monitor->setStretchFactor(1, 1);
        log_panel_ = new LogPanel(monitor_col);
        log_panel_->setMinimumHeight(140);
        monitor_col->addWidget(monitor);
        monitor_col->addWidget(log_panel_);
        monitor_col->setStretchFactor(0, 1);
        monitor_col->setStretchFactor(1, 0);
        monitor_col->setSizes({760, 220});

        // 右：操作（舵机操作 / 自动调试 / 编程）全高，最大化时全部可见。
        inspector_panel_ = new InspectorPanel(main_split);

        main_split->addWidget(left);
        main_split->addWidget(monitor_col);
        main_split->addWidget(inspector_panel_);
        main_split->setStretchFactor(0, 0);
        main_split->setStretchFactor(1, 1);
        main_split->setStretchFactor(2, 0);
        main_split->setSizes({300, 1100, 420});
        layout->addWidget(main_split, 1);

        setCentralWidget(central);
        statusBar()->showMessage(tr("就绪"));

        connect(connection_panel_, &ConnectionPanel::connectRequested, this, &MainWindow::connectBus);
        connect(connection_panel_, &ConnectionPanel::disconnectRequested, this,
                &MainWindow::disconnectBus);
        connect(connection_panel_, &ConnectionPanel::scanRequested, this, &MainWindow::scanBus);
        connect(connection_panel_, &ConnectionPanel::stopAllRequested, this, &MainWindow::stopAll);
        connect(connection_panel_, &ConnectionPanel::pollRateChanged, this, &MainWindow::setPollRate);
        connect(device_tree_, &QTreeWidget::itemSelectionChanged, this,
                &MainWindow::onServoSelected);
        connect(status_table_, &QTableWidget::itemSelectionChanged, this,
                &MainWindow::onServoSelected);

        connect(inspector_panel_, &InspectorPanel::goalRequested, this, &MainWindow::handleGoal);
        connect(inspector_panel_, &InspectorPanel::torqueRequested, this, &MainWindow::handleTorque);
        connect(inspector_panel_, &InspectorPanel::servoConfigRequested, this,
                &MainWindow::handleServoConfig);

        // 寄存器工作台（非模态弹窗）：全量寄存器读写。
        params_dialog_ = new ParamsDialog(this);
        connect(inspector_panel_, &InspectorPanel::registerRequested, this, [this] {
            if (!has_subject_) {
                log("warn", "No target servo selected");
                return;
            }
            params_dialog_->setSubject(subject_port_, subject_id_);
            params_dialog_->show();
            params_dialog_->raise();
            if (const auto it = buses_.find(subject_port_); it != buses_.end()) {
                it->second.bridge->readParameters(subject_port_, subject_id_);
            }
        });
        connect(params_dialog_, &ParamsDialog::paramWriteRequested, this,
                &MainWindow::handleParamWrite);
        connect(params_dialog_, &ParamsDialog::paramWriteAllRequested, this,
                &MainWindow::handleParamWriteAll);
        connect(params_dialog_, &ParamsDialog::backupRequested, this, &MainWindow::handleBackup);
        connect(params_dialog_, &ParamsDialog::restoreRequested, this, &MainWindow::handleRestore);
        connect(params_dialog_, &ParamsDialog::reloadParametersRequested, this,
                [this](const QString& port, const servo_core::ServoId id) {
                    if (const auto it = buses_.find(port); it != buses_.end()) {
                        it->second.bridge->readParameters(port, id);
                    }
                });
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
        // 日志输出英文原文到 stderr 与日志面板，状态栏显示简要信息。
        std::fprintf(stderr, "[%s] %s\n", level.toStdString().c_str(),
                     message.toStdString().c_str());
        log_panel_->appendLog(level, message);
        statusBar()->showMessage(message, (level == "error" || level == "warn") ? 8000 : 4000);
    }

    servo_core::ServoConfig MainWindow::configFor(const QString& port,
                                                  const servo_core::ServoId id) const {
        const auto port_it = configs_.find(port);
        if (port_it != configs_.end()) {
            const auto id_it = port_it->second.find(id);
            if (id_it != port_it->second.end()) {
                return id_it->second;
            }
        }
        servo_core::ServoConfig config;
        config.id = id;
        return config;
    }

    void MainWindow::connectBus(const QString& backend, const QString& port, const int baudRate,
                                const int timeoutMs) {
        if (port.isEmpty()) {
            log("error", "No serial port selected");
            return;
        }
        if (buses_.count(port) != 0) {
            log("warn", "Already connected to " + port);
            return;
        }
        auto driver = servo_core::DriverRegistry::instance().create(backend.toStdString());
        if (!driver) {
            log("error", "Driver '" + backend + "' is not registered");
            return;
        }
        auto* bridge = new SessionBridge(this);
        // 帧级跟踪在启动前安装到驱动，避免与工作线程竞态。
        driver->set_trace_hook([bridge](const char* direction, const std::vector<uint8_t>& frame) {
            bridge->traceFrame(direction, frame);
        });

        servo_core::BusConfig config;
        config.port = port.toStdString();
        config.baud_rate = baudRate;
        config.response_timeout_ms = timeoutMs;

        auto session = std::make_shared<servo_core::BusSession>(std::move(driver), config);
        bridge->attach(session);

        connect(bridge, &SessionBridge::logReceived, this, &MainWindow::log);
        connect(bridge, &SessionBridge::frameTraced, this,
                [this](const QString& bus_port, const QString& direction, const QByteArray& frame) {
                    log_panel_->appendFrame(bus_port, direction, frame);
                });
        connect(bridge, &SessionBridge::statesReceived, this,
                [this](const QString& bus_port, const QVector<servo_core::ServoState>& states) {
                    status_table_->updateStates(bus_port, states);
                    chart_panel_->appendStates(bus_port, states);
                });
        connect(bridge, &SessionBridge::scanFinished, this,
                [this](const QString& bus_port, const QVector<servo_core::ServoInfo>& infos) {
                    device_tree_->setServos(bus_port, infos);
                    bus_infos_[bus_port] = infos;
                    for (const auto& info : infos) {
                        configs_[bus_port].try_emplace(info.id, configFor(bus_port, info.id));
                    }
                    applyPolling(bus_port);
                    connection_panel_->setScanning(false);
                    log("info", QStringLiteral("Scan on %1 finished: %2 servo(s) found")
                                    .arg(bus_port)
                                    .arg(infos.size()));
                    statusBar()->showMessage(
                        tr("%1：扫描结束，发现 %2 个舵机").arg(bus_port).arg(infos.size()), 10000);
                });
        connect(bridge, &SessionBridge::parametersReceived, this,
                [this](const QString& bus_port, const int id,
                       const QVector<ParamValue>& values) {
                    params_dialog_->setParameters(bus_port, id, values);
                    log("info", "Parameters loaded for servo " + QString::number(id));
                });
        connect(bridge, &SessionBridge::commandFinished, this,
                [this](const bool success, const QString& error) {
                    if (!success) {
                        log("error", error);
                    }
                });
        connect(bridge, &SessionBridge::restoreFinished, this,
                [this](const QString& bus_port, const int id, const int written,
                       const bool success, const QString& error) {
                    if (success) {
                        log("info", QStringLiteral("Restored %1 parameter(s) on servo %2 (%3)")
                                        .arg(written)
                                        .arg(id)
                                        .arg(bus_port));
                        if (const auto it = buses_.find(bus_port); it != buses_.end()) {
                            it->second.bridge->readParameters(bus_port,
                                                              static_cast<servo_core::ServoId>(id));
                        }
                    }
                    else {
                        log("error", error);
                    }
                });

        if (const auto result = session->start(); !result) {
            log("error", QString::fromStdString(result.error()));
            bridge->deleteLater();
            return;
        }
        buses_[port] = BusEntry{session, bridge, backend};
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
        connection_panel_->setScanning(false);
        log("info", "Disconnected from " + port);
    }

    void MainWindow::scanBus(const QString& port) {
        const auto it = buses_.find(port);
        if (it == buses_.end()) {
            log("error", "Not connected to " + port);
            return;
        }
        statusBar()->showMessage(tr("正在扫描 %1 ...").arg(port));
        connection_panel_->setScanning(true);
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

    void MainWindow::setPollRate(const int hz) {
        poll_hz_ = hz;
        for (const auto& [port, entry] : buses_) {
            applyPolling(port);
        }
        statusBar()->showMessage(tr("轮询频率：%1 Hz").arg(hz), 4000);
    }

    void MainWindow::applyPolling(const QString& port) {
        const auto it = buses_.find(port);
        if (it == buses_.end()) {
            return;
        }
        const auto ids = device_tree_->servoIds(port);
        std::vector<servo_core::ServoId> std_ids(ids.begin(), ids.end());
        it->second.bridge->setPolling(std_ids, poll_hz_);
    }

    void MainWindow::onServoSelected() {
        QString port;
        servo_core::ServoId id = 0;
        if (!device_tree_->selectedServo(&port, &id) && !status_table_->selectedServo(&port, &id)) {
            return;
        }
        has_subject_ = true;
        subject_port_ = port;
        subject_id_ = id;

        QString model = "unknown";
        double max_range_deg = 360.0;
        for (const auto& info : bus_infos_[port]) {
            if (info.id == id) {
                model = QString::fromStdString(info.model);
                max_range_deg = info.max_range_deg;
                break;
            }
        }
        inspector_panel_->setSubject(port, id, configFor(port, id), max_range_deg);
        chart_panel_->setSubject(port, id, model);
        // 寄存器弹窗跟随当前选中对象。
        params_dialog_->setSubject(port, id);
        if (const auto it = buses_.find(port); it != buses_.end()) {
            it->second.bridge->readParameters(port, id);
        }
    }

    void MainWindow::handleGoal(const QString& port, const int ticks, const int timeMs,
                                const int speed, const int acc, const GoalTarget target) {
        // 目标集合按总线分组：单机 / 状态表选中（可跨总线）/ 当前协议全部。
        std::map<QString, std::vector<servo_core::ServoId>> targets;
        if (target == GoalTarget::kCurrent) {
            if (has_subject_ && subject_port_ == port) {
                targets[port].push_back(subject_id_);
            }
        }
        else if (target == GoalTarget::kSelection) {
            for (const auto& [row_port, id] : status_table_->selectedServos()) {
                targets[row_port].push_back(id);
            }
            if (targets.empty() && has_subject_) {
                targets[subject_port_].push_back(subject_id_);
            }
        }
        else {
            // 当前协议 = 与当前舵机同驱动名的全部总线（一条串口一个协议）。
            const auto subject = buses_.find(port);
            const QString driver_name =
                subject != buses_.end() ? subject->second.driver_name : QString("feetech_scs");
            for (const auto& [bus_port, entry] : buses_) {
                if (entry.driver_name != driver_name) {
                    continue;
                }
                for (const auto id : device_tree_->servoIds(bus_port)) {
                    targets[bus_port].push_back(id);
                }
            }
        }
        if (targets.empty()) {
            log("warn", "No target servo selected");
            return;
        }

        int total = 0;
        for (const auto& [bus_port, ids] : targets) {
            const auto it = buses_.find(bus_port);
            if (it == buses_.end() || ids.empty()) {
                continue;
            }
            total += static_cast<int>(ids.size());
            if (ids.size() == 1) {
                const auto id = ids.front();
                it->second.bridge->submit(
                    [id, ticks, timeMs, speed, acc](servo_core::IServoDriver& driver) {
                        return driver.write_goal(id, ticks, timeMs, speed, acc);
                    });
            }
            else {
                const std::vector<int> positions(ids.size(), ticks);
                const std::vector<int> speeds(ids.size(), speed);
                it->second.bridge->submit(
                    [ids, positions, timeMs, speeds, acc](servo_core::IServoDriver& driver) {
                        return driver.sync_write_goals(ids, positions, timeMs, speeds, acc);
                    });
            }
        }
        statusBar()->showMessage(tr("已下发 %1 个目标 → %2 tick").arg(total).arg(ticks), 4000);
    }

    void MainWindow::handleTorque(const QString& port, const servo_core::ServoId id,
                                  const int mode) {
        const auto it = buses_.find(port);
        if (it == buses_.end()) {
            return;
        }
        // 扭矩设置：0 自由 / 1 锁紧 / 2 阻尼（寄存器 40）。
        it->second.bridge->submit([id, mode](servo_core::IServoDriver& driver) {
            return driver.write_param(id, "torque_enable", mode);
        });
        static const char* kModes[] = {"free", "locked", "damping"};
        log("info", QStringLiteral("Torque set to %1 on servo %2")
                        .arg(mode >= 0 && mode <= 2 ? kModes[mode] : "?")
                        .arg(id));
    }

    void MainWindow::handleParamWrite(const QString& port, const servo_core::ServoId id,
                                      const QString& key, const int value,
                                      const QString& valueText) {
        const auto it = buses_.find(port);
        if (it == buses_.end()) {
            return;
        }

        // ID / 波特率专属流程：确认 → 写入 → 恢复访问（重扫/重连）。
        if (key == "id") {
            if (QMessageBox::warning(
                    this, tr("修改 ID"),
                    tr("修改 ID 后舵机将不再响应旧 ID，写错需要重新扫描恢复。\n确定写入 ID=%1 ？")
                        .arg(value),
                    QMessageBox::Ok | QMessageBox::Cancel) != QMessageBox::Ok) {
                return;
            }
            it->second.bridge->submit(
                [id, value](servo_core::IServoDriver& driver) {
                    return driver.write_param(id, "id", value);
                },
                [this, port](const bool ok, const QString& error) {
                    QMetaObject::invokeMethod(
                        this,
                        [this, port, ok, error] {
                            if (!ok) {
                                log("error", error);
                                return;
                            }
                            log("info", "ID changed, rescanning " + port);
                            scanBus(port);
                        },
                        Qt::QueuedConnection);
                });
            return;
        }
        if (key == "baud_rate") {
            const int new_baud = valueText.toInt();
            if (QMessageBox::warning(
                    this, tr("修改波特率"),
                    tr("修改波特率后将以 %1 重新连接总线。\n确定写入？").arg(new_baud),
                    QMessageBox::Ok | QMessageBox::Cancel) != QMessageBox::Ok) {
                return;
            }
            it->second.bridge->submit(
                [id, value](servo_core::IServoDriver& driver) {
                    return driver.write_param(id, "baud_rate", value);
                },
                [this, port, new_baud](const bool ok, const QString& error) {
                    QMetaObject::invokeMethod(
                        this,
                        [this, port, new_baud, ok, error] {
                            if (!ok) {
                                log("error", error);
                                return;
                            }
                            log("info", "Baud rate changed, reconnecting " + port);
                            const int timeout =
                                buses_.count(port) != 0
                                    ? buses_[port].session->config().response_timeout_ms
                                    : 10;
                            const QString backend =
                                buses_.count(port) != 0 ? buses_[port].driver_name
                                                        : QString("feetech_scs");
                            disconnectBus(port);
                            connectBus(backend, port, new_baud, timeout);
                            scanBus(port);
                        },
                        Qt::QueuedConnection);
                });
            return;
        }

        // 通用参数写入同样需要确认。
        if (QMessageBox::question(this, tr("写入参数"),
                                  tr("确定将 %1=%2 写入 %3 的舵机 %4？")
                                      .arg(key)
                                      .arg(value)
                                      .arg(port)
                                      .arg(id),
                                  QMessageBox::Ok | QMessageBox::Cancel) != QMessageBox::Ok) {
            return;
        }
        it->second.bridge->submit([id, key, value](servo_core::IServoDriver& driver) {
            return driver.write_param(id, key.toStdString(), value);
        });
    }

    void MainWindow::handleParamWriteAll(const QString& port, const QString& key, const int value) {
        const auto it = buses_.find(port);
        if (it == buses_.end()) {
            return;
        }
        if (QMessageBox::question(
                this, tr("批量写入参数"),
                tr("将 %1=%2 写入 %3 的全部舵机？").arg(key).arg(value).arg(port),
                QMessageBox::Ok | QMessageBox::Cancel) != QMessageBox::Ok) {
            return;
        }
        const auto ids = device_tree_->servoIds(port);
        it->second.bridge->submit([ids, key, value](servo_core::IServoDriver& driver) {
            for (const auto id : ids) {
                if (const auto result = driver.write_param(id, key.toStdString(), value); !result) {
                    return result;
                }
            }
            return servo_core::Result::success();
        });
    }

    void MainWindow::handleServoConfig(const QString& port, const servo_core::ServoId id,
                                       const double rangeDeg, const double minTicks,
                                       const double maxTicks) {
        servo_core::ServoConfig config = configFor(port, id);
        config.range_deg = rangeDeg;
        config.min_ticks = minTicks;
        config.max_ticks = maxTicks;
        configs_[port][id] = config;

        const auto it = buses_.find(port);
        if (it != buses_.end()) {
            it->second.bridge->submit([config](servo_core::IServoDriver& driver) {
                return driver.set_servo_config(config);
            });
        }
        inspector_panel_->updateConfig(config);
        log("info", QStringLiteral("Servo %1 config updated: range=%2 deg, clamp=[%3, %4]")
                        .arg(id)
                        .arg(rangeDeg)
                        .arg(minTicks)
                        .arg(maxTicks));
    }

    void MainWindow::handleBackup(const QString& port, const servo_core::ServoId id) {
        const auto it = buses_.find(port);
        if (it == buses_.end()) {
            return;
        }
        // 默认文件名带舵机型号与 id，便于换机备份归档。
        QString model = "unknown";
        for (const auto& info : bus_infos_[port]) {
            if (info.id == id) {
                model = QString::fromStdString(info.model);
                break;
            }
        }
        const QString path = QFileDialog::getSaveFileName(
            this, tr("备份参数"),
            QStringLiteral("servo_%1_%2_params.json").arg(model).arg(id),
            tr("参数备份 (*.json)"));
        if (path.isEmpty()) {
            return;
        }
        it->second.bridge->backupParameters(port, id, path);
    }

    void MainWindow::handleRestore(const QString& port, const servo_core::ServoId id) {
        const auto it = buses_.find(port);
        if (it == buses_.end()) {
            return;
        }
        const QString path = QFileDialog::getOpenFileName(this, tr("恢复参数"), QString(),
                                                          tr("参数备份 (*.json)"));
        if (path.isEmpty()) {
            return;
        }
        QFile file(path);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            log("error", "Failed to open '" + path + "'");
            return;
        }
        ParamsBackup backup;
        QString error;
        if (!parseBackup(file.readAll(), &backup, &error)) {
            log("error", error);
            return;
        }
        const auto params = paramsForRestore(backup);
        if (params.isEmpty()) {
            log("warn", "Backup contains no restorable parameters");
            return;
        }
        if (QMessageBox::question(
                this, tr("恢复参数"),
                tr("将 %1 个参数写入 %2 的舵机 %3？\n（ID 与波特率不会被恢复）")
                    .arg(params.size())
                    .arg(port)
                    .arg(id),
                QMessageBox::Ok | QMessageBox::Cancel) != QMessageBox::Ok) {
            return;
        }
        it->second.bridge->restoreParameters(port, id, params);
    }

} // namespace app
