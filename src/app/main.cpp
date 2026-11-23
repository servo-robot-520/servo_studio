// servo_studio 桌面上位机入口。
#include <QFile>

#include "app/button_focus_filter.hpp"
#include "app/main_window.hpp"
#include "app/session_bridge.hpp"
#include "app/wheel_focus_filter.hpp"
#include "servo_backends/feetech_scs_driver.hpp"
#include "servo_backends/ubtech_driver.hpp"
#include "servo_backends/ubtech_liqiulong123_driver.hpp"

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    QApplication::setApplicationName("servo_studio");
    QApplication::setWindowIcon(QIcon(":/icon.png"));

    // 鼠标点击按钮后清除滞留的焦点描边（键盘焦点不受影响）。
    auto* focus_filter = new app::ButtonFocusFilter(&app);
    app.installEventFilter(focus_filter);

    // 输入控件滚轮需先点击聚焦，悬停滚动不再误改数值。
    auto* wheel_filter = new app::WheelFocusFilter(&app);
    app.installEventFilter(wheel_filter);

    // 跨线程 queued 投递所需元类型。
    qRegisterMetaType<QVector<servo_core::ServoState>>("QVector<servo_core::ServoState>");
    qRegisterMetaType<QVector<servo_core::ServoInfo>>("QVector<servo_core::ServoInfo>");
    qRegisterMetaType<QVector<app::ParamValue>>("QVector<app::ParamValue>");

    if (QFile style(":/style.qss"); style.open(QIODevice::ReadOnly)) {
        app.setStyleSheet(QString::fromUtf8(style.readAll()));
    }

    // 驱动注册：中间层通过注册表创建后端。
    servo_backends::FeetechScsDriver::register_driver();
    servo_backends::UbtechDriver::register_driver();
    servo_backends::UbtechLiqiulong123Driver::register_driver();

    app::MainWindow window;
    window.showMaximized(); // 程序启动默认最大化

    return QApplication::exec();
}
