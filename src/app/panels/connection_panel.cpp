#include "app/panels/connection_panel.hpp"

#include <QComboBox>
#include <QDir>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>

namespace app {

    ConnectionPanel::ConnectionPanel(QWidget* parent) :
        QWidget(parent) {
        auto* layout = new QHBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(8);

        layout->addWidget(new QLabel(tr("串口"), this));
        port_combo_ = new QComboBox(this);
        port_combo_->setToolTip(tr("串口设备（优先显示稳定的 by-id 名称）"));
        port_combo_->setMinimumWidth(180);
        port_combo_->setEditable(true);
        layout->addWidget(port_combo_);

        auto* refresh = new QPushButton(tr("刷新"), this);
        refresh->setToolTip(tr("刷新串口列表"));
        connect(refresh, &QPushButton::clicked, this, &ConnectionPanel::refreshPorts);
        layout->addWidget(refresh);

        layout->addWidget(new QLabel(tr("波特率"), this));
        baud_combo_ = new QComboBox(this);
        baud_combo_->setToolTip(tr("总线波特率（对应舵机寄存器 6 的档位）"));
        baud_combo_->setEditable(true);
        for (const int baud : {1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400}) {
            baud_combo_->addItem(QString::number(baud));
        }
        layout->addWidget(baud_combo_);

        layout->addWidget(new QLabel(tr("超时(ms)"), this));
        timeout_spin_ = new QSpinBox(this);
        timeout_spin_->setToolTip(tr("单次应答超时（毫秒）"));
        timeout_spin_->setRange(1, 1000);
        timeout_spin_->setValue(10);
        layout->addWidget(timeout_spin_);

        layout->addWidget(new QLabel(tr("轮询(Hz)"), this));
        poll_spin_ = new QSpinBox(this);
        poll_spin_->setToolTip(tr("遥测轮询频率（Hz），10–100 可调"));
        poll_spin_->setRange(10, 100);
        poll_spin_->setValue(50);
        connect(poll_spin_, &QSpinBox::valueChanged, this,
                [this](const int hz) { emit pollRateChanged(hz); });
        layout->addWidget(poll_spin_);

        connect_button_ = new QPushButton(tr("连接"), this);
        connect_button_->setToolTip(tr("连接 / 断开当前串口总线"));
        connect_button_->setObjectName("primary");
        connect(connect_button_, &QPushButton::clicked, this, [this] {
            if (connect_button_->property("connected").toBool()) {
                emit disconnectRequested(port());
            }
            else {
                emit connectRequested(port(), baudRate(), timeoutMs());
            }
        });
        layout->addWidget(connect_button_);

        scan_button_ = new QPushButton(tr("扫描"), this);
        scan_button_->setToolTip(tr("扫描总线上的全部舵机（id 0..253），扫描期间不可重复触发"));
        scan_button_->setEnabled(false);
        connect(scan_button_, &QPushButton::clicked, this,
                [this] { emit scanRequested(port()); });
        layout->addWidget(scan_button_);

        stop_button_ = new QPushButton(tr("全部松扭矩"), this);
        stop_button_->setToolTip(tr("急停：关闭总线上全部舵机的力矩输出，输出轴自由"));
        stop_button_->setObjectName("danger");
        stop_button_->setEnabled(false);
        connect(stop_button_, &QPushButton::clicked, this,
                [this] { emit stopAllRequested(port()); });
        layout->addWidget(stop_button_);

        layout->addStretch(1);
        refreshPorts();
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
        connect_button_->setText(connected ? tr("断开") : tr("连接"));
        scan_button_->setEnabled(connected && !scanning_);
        stop_button_->setEnabled(connected);
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
