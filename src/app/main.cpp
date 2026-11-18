// servo_studio 桌面上位机入口。
#include <QApplication>
#include <QFile>

#include "app/main_window.hpp"
#include "app/session_bridge.hpp"
#include "servo_backends/feetech_scs_driver.hpp"
#include "servo_core/registry.hpp"

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    app.setApplicationName("servo_studio");

    // 跨线程 queued 投递所需元类型。
    qRegisterMetaType<QVector<servo_core::ServoState>>("QVector<servo_core::ServoState>");
    qRegisterMetaType<QVector<servo_core::ServoInfo>>("QVector<servo_core::ServoInfo>");
    qRegisterMetaType<QVector<app::ParamValue>>("QVector<app::ParamValue>");

    QFile style(":/style.qss");
    if (style.open(QIODevice::ReadOnly)) {
        app.setStyleSheet(QString::fromUtf8(style.readAll()));
    }

    // 驱动注册：中间层通过注册表创建后端。
    servo_backends::FeetechScsDriver::register_driver();

    app::MainWindow window;
    window.show();

    return app.exec();
}
