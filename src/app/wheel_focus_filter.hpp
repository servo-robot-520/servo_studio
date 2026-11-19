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
                // 真实滚轮事件可能落在数字框内部的行编辑等子控件上，
                // 需向上查找所属输入控件再判断焦点。
                for (auto* w = static_cast<QWidget*>(watched); w != nullptr;
                     w = w->parentWidget()) {
                    if (qobject_cast<QAbstractSpinBox*>(w) != nullptr ||
                        qobject_cast<QComboBox*>(w) != nullptr ||
                        qobject_cast<QSlider*>(w) != nullptr) {
                        return !w->hasFocus(); // 未聚焦：忽略滚轮
                    }
                }
            }
            return QObject::eventFilter(watched, event);
        }
    };

} // namespace app
