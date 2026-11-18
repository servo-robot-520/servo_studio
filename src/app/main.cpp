// servo_studio 桌面上位机入口。
#include <QApplication>
#include <QFile>
#include <QTimer>
#include <QTreeWidgetItem>

#include "app/main_window.hpp"
#include "app/panels/device_tree.hpp"
#include "app/session_bridge.hpp"
#include "servo_backends/feetech_scs_driver.hpp"
#include "servo_core/registry.hpp"

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    app.setApplicationName("servo_studio");

    // 跨线程 queued 投递所需元类型。
    qRegisterMetaType<QVector<servo_core::ServoState>>("QVector<servo_core::ServoState>");
    qRegisterMetaType<QVector<servo_core::ServoInfo>>("QVector<servo_core::ServoInfo>");

    QFile style(":/style.qss");
    if (style.open(QIODevice::ReadOnly)) {
        app.setStyleSheet(QString::fromUtf8(style.readAll()));
    }

    // 驱动注册：中间层通过注册表创建后端。
    servo_backends::FeetechScsDriver::register_driver();

    app::MainWindow window;
    window.show();

    // 临时脚手架（M6 移除）：SERVO_STUDIO_SHOT 指定截图路径；SERVO_STUDIO_AUTOPORT
    // 指定串口时自动连接 + 扫描，用于无显示环境下核对界面与端到端链路。
    const QByteArray shot = qgetenv("SERVO_STUDIO_SHOT");
    const QByteArray auto_port = qgetenv("SERVO_STUDIO_AUTOPORT");
    if (!auto_port.isEmpty()) {
        const QString port = QString::fromUtf8(auto_port);
        QTimer::singleShot(200, &window, [&window, port] {
            QMetaObject::invokeMethod(&window, "connectBus", Qt::QueuedConnection,
                                      Q_ARG(QString, port), Q_ARG(int, 1000000), Q_ARG(int, 10));
        });
        QTimer::singleShot(1200, &window, [&window, port] {
            QMetaObject::invokeMethod(&window, "scanBus", Qt::QueuedConnection,
                                      Q_ARG(QString, port));
        });
        // 选中第一个舵机节点，驱动曲线面板（脚手架专用）。
        QTimer::singleShot(4000, &window, [&window] {
            if (auto* tree = window.findChild<app::DeviceTree*>()) {
                QTreeWidgetItem* bus = tree->topLevelItem(0);
                if (bus != nullptr && bus->childCount() > 0) {
                    tree->setCurrentItem(bus->child(0));
                }
            }
        });
    }
    if (!shot.isEmpty()) {
        QTimer::singleShot(8000, &window, [&window, shot] {
            window.grab().save(QString::fromUtf8(shot));
            QCoreApplication::quit();
        });
    }

    return app.exec();
}
