#include "app/panels/connection_panel.hpp"

#include "servo_core/registry.hpp"

#include <QComboBox>
#include <QDir>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>

namespace app {

    ConnectionPanel::ConnectionPanel(QWidget* parent) :
        QWidget(parent) {
        // 根布局承载两个子区；嵌入主窗口时子区被重新挂载。
        auto* root = new QVBoxLayout(this);
        root->setContentsMargins(0, 0, 0, 0);
        root->setSpacing(6);

        // 顶栏一行：协议 / 串口 / 刷新 / 波特率 / 打开。
        top_bar_ = new QWidget(this);
        auto* row = new QHBoxLayout(top_bar_);
        row->setContentsMargins(0, 0, 0, 0);
        row->setSpacing(8);

        // 协议后端：一条串口仅支持一种协议；帧格式（含校验位）由协议
        // 决定（FT-SCS 固定 8N1），不在 UI 暴露。
        row->addWidget(new QLabel(tr("协议"), top_bar_));
        backend_combo_ = new QComboBox(top_bar_);
        backend_combo_->setToolTip(tr("舵机协议后端（一条串口仅支持一种协议）"));
        for (const auto& name : servo_core::DriverRegistry::instance().names()) {
            backend_combo_->addItem(QString::fromStdString(name));
        }
        row->addWidget(backend_combo_);

        row->addWidget(new QLabel(tr("串口"), top_bar_));
        port_combo_ = new QComboBox(top_bar_);
        port_combo_->setToolTip(tr("串口设备（优先显示稳定的 by-id 名称）"));
        // 收敛关闭态宽度提示：长 by-id 名称在弹层中完整显示，
        // 不再把窗口最小尺寸撑大（否则 WM 拒绝拖顶最大化）。
        port_combo_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
        port_combo_->setMinimumContentsLength(12);
        port_combo_->setMinimumWidth(180);
        port_combo_->setEditable(true);
        row->addWidget(port_combo_, 1);

        auto* refresh = new QPushButton(tr("刷新"), top_bar_);
        refresh->setToolTip(tr("刷新串口列表"));
        connect(refresh, &QPushButton::clicked, this, &ConnectionPanel::refreshPorts);
        row->addWidget(refresh);

        row->addWidget(new QLabel(tr("波特率"), top_bar_));
        baud_combo_ = new QComboBox(top_bar_);
        baud_combo_->setToolTip(tr("总线波特率（对应舵机寄存器 6 的档位）"));
        baud_combo_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
        baud_combo_->setMinimumContentsLength(8);
        baud_combo_->setEditable(true);
        for (const int baud : {1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400}) {
            baud_combo_->addItem(QString::number(baud));
        }
        row->addWidget(baud_combo_);

        connect_button_ = new QPushButton(tr("打开"), top_bar_);
        connect_button_->setObjectName("primary");
        connect_button_->setToolTip(tr("打开 / 关闭当前串口总线"));
        connect(connect_button_, &QPushButton::clicked, this, [this] {
            if (connect_button_->property("connected").toBool()) {
                emit disconnectRequested(port());
            }
            else {
                emit connectRequested(backend(), port(), baudRate(), timeoutMs());
            }
        });
        row->addWidget(connect_button_);
        row->addStretch(1);
        root->addWidget(top_bar_);

        // 侧栏（设备列表上方）：超时 / 轮询 / 扫描 / 全部松扭矩。
        // 两列网格：超时+输入框 与 扫描 同宽对齐，轮询+输入框 与 全部松扭矩 对齐。
        side_bar_ = new QWidget(this);
        auto* side = new QVBoxLayout(side_bar_);
        side->setContentsMargins(0, 0, 0, 0);
        side->setSpacing(6);

        auto* grid = new QGridLayout;
        grid->setHorizontalSpacing(8);
        grid->setVerticalSpacing(6);

        auto* timeout_box = new QWidget(side_bar_);
        auto* timeout_row = new QHBoxLayout(timeout_box);
        timeout_row->setContentsMargins(0, 0, 0, 0);
        timeout_row->setSpacing(8);
        timeout_row->addWidget(new QLabel(tr("超时"), timeout_box));
        timeout_spin_ = new QSpinBox(timeout_box);
        timeout_spin_->setToolTip(tr("单次应答超时（毫秒）"));
        timeout_spin_->setRange(1, 1000);
        timeout_spin_->setValue(10);
        timeout_row->addWidget(timeout_spin_);
        timeout_row->addStretch(1);

        auto* poll_box = new QWidget(side_bar_);
        auto* poll_row = new QHBoxLayout(poll_box);
        poll_row->setContentsMargins(0, 0, 0, 0);
        poll_row->setSpacing(8);
        poll_row->addWidget(new QLabel(tr("轮询"), poll_box));
        poll_spin_ = new QSpinBox(poll_box);
        poll_spin_->setToolTip(tr("遥测轮询频率（Hz），10–100 可调"));
        poll_spin_->setRange(10, 100);
        poll_spin_->setValue(50);
        connect(poll_spin_, &QSpinBox::valueChanged, this,
                [this](const int hz) { emit pollRateChanged(hz); });
        poll_row->addWidget(poll_spin_);
        poll_row->addStretch(1);

        scan_button_ = new QPushButton(tr("扫描"), side_bar_);
        scan_button_->setToolTip(tr("扫描总线上的全部舵机（id 0..253），扫描期间不可重复触发"));
        scan_button_->setEnabled(false);
        connect(scan_button_, &QPushButton::clicked, this,
                [this] { emit scanRequested(port()); });
        stop_button_ = new QPushButton(tr("全部松扭矩"), side_bar_);
        stop_button_->setObjectName("danger");
        stop_button_->setToolTip(tr("急停：关闭总线上全部舵机的力矩输出，输出轴自由"));
        stop_button_->setEnabled(false);
        connect(stop_button_, &QPushButton::clicked, this,
                [this] { emit stopAllRequested(port()); });

        grid->addWidget(timeout_box, 0, 0);
        grid->addWidget(poll_box, 0, 1);
        grid->addWidget(scan_button_, 1, 0);
        grid->addWidget(stop_button_, 1, 1);
        grid->setColumnStretch(0, 1);
        grid->setColumnStretch(1, 1);
        side->addLayout(grid);
        root->addWidget(side_bar_);

        refreshPorts();
    }

    QWidget* ConnectionPanel::topBar() const {
        return top_bar_;
    }

    QWidget* ConnectionPanel::sideBar() const {
        return side_bar_;
    }

    QString ConnectionPanel::backend() const {
        return backend_combo_->currentText();
    }

    QString ConnectionPanel::port() const {
        return port_combo_->currentText().trimmed();
    }

    int ConnectionPanel::baudRate() const {
        return baud_combo_->currentText().toInt();
    }

    int ConnectionPanel::timeoutMs() const {
        return timeout_spin_->value();
    }

    int ConnectionPanel::pollRate() const {
        return poll_spin_->value();
    }

    void ConnectionPanel::setScanning(const bool scanning) {
        scanning_ = scanning;
        scan_button_->setEnabled(!scanning && connect_button_->property("connected").toBool());
    }

    void ConnectionPanel::setConnected(const QString& port, const bool connected) {
        if (port != this->port()) {
            return;
        }
        connect_button_->setProperty("connected", connected);
        connect_button_->setText(connected ? tr("关闭") : tr("打开"));
        scan_button_->setEnabled(connected && !scanning_);
        stop_button_->setEnabled(connected);
        backend_combo_->setEnabled(!connected);
        port_combo_->setEnabled(!connected);
        baud_combo_->setEnabled(!connected);
    }

    void ConnectionPanel::refreshPorts() {
        const QString current = port_combo_->currentText();
        port_combo_->clear();

        // 优先展示稳定的 by-id 名称，再列出设备节点。
        QDir by_id("/dev/serial/by-id");
        QStringList ports;
        if (by_id.exists()) {
            ports = by_id.entryList(QDir::System | QDir::Files, QDir::Name);
            for (QString& entry : ports) {
                entry = by_id.filePath(entry);
            }
        }
        QDir dev("/dev");
        ports += dev.entryList({"ttyUSB*", "ttyACM*"}, QDir::System | QDir::Readable, QDir::Name);
        port_combo_->addItems(ports);
        if (!current.isEmpty()) {
            port_combo_->setCurrentText(current);
        }
    }

} // namespace app
