#pragma once

#include <QAbstractSpinBox>
#include <QComboBox>
#include <QEvent>
#include <QObject>
#include <QSlider>
#include <QWidget>

namespace app {

    /// @brief 输入控件滚轮守卫：数字框/下拉/滑杆仅在获得焦点后响应滚轮。
    ///
    /// 默认行为是悬停即滚轮改值，极易误触；本过滤器要求先点击聚焦。
    class WheelFocusFilter : public QObject {
    public:
        using QObject::QObject;

    protected:
        bool eventFilter(QObject* watched, QEvent* event) override {
            if (event->type() == QEvent::Wheel && watched->isWidgetType()) {
                auto* widget = static_cast<QWidget*>(watched);
                const bool is_input = qobject_cast<QAbstractSpinBox*>(widget) != nullptr ||
                    qobject_cast<QComboBox*>(widget) != nullptr ||
                    qobject_cast<QSlider*>(widget) != nullptr;
                if (is_input && !widget->hasFocus()) {
                    return true; // 未获焦点的滚轮直接忽略
                }
            }
            return QObject::eventFilter(watched, event);
        }
    };

} // namespace app
