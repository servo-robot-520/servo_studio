#pragma once

#include <QApplication>
#include <QComboBox>
#include <QEvent>
#include <QObject>
#include <QSlider>
#include <QWheelEvent>
#include <QWidget>

namespace app {

    /// @brief 输入控件滚轮守卫：数字框/下拉/滑杆仅在获得焦点后响应滚轮。
    ///
    /// 默认行为是悬停即滚轮改值，极易误触。注意根因：真实滚轮派发到控件前，
    /// QApplication::notify 会按 WheelFocus 策略把焦点"滚"给指针下的控件
    /// （giveFocusAccordingToFocusPolicy，MouseFocusReason），等事件到达任何
    /// 事件过滤器时 hasFocus() 恒为真——基于控件级焦点的过滤必然放行。
    /// 因此必须在顶层 QWindow（派发前）这一级拦截，焦点守卫才成立。
    class WheelFocusFilter : public QObject {
    public:
        using QObject::QObject;

        bool eventFilter(QObject* watched, QEvent* event) override {
            if (event->type() != QEvent::Wheel) {
                return QObject::eventFilter(watched, event);
            }
            if (!watched->isWidgetType()) {
                // 派发前的顶层阶段：指针下控件尚未被滚轮偷走焦点。
                if (const auto* wheel = dynamic_cast<const QWheelEvent*>(event);
                    isUnfocusedInput(QApplication::widgetAt(wheel->globalPosition().toPoint()))) {
                    return true;
                }
            }
            else if (isUnfocusedInput(dynamic_cast<QWidget*>(watched))) {
                // 直送控件的滚轮（合成事件等）：按所属输入控件的焦点取舍。
                return true;
            }
            return QObject::eventFilter(watched, event);
        }

    private:
        /// widget 所属（或即是）数字框/下拉/滑杆且未聚焦时返回 true。
        /// 不跨窗口上溯，避免把下拉弹层归因到组合框。
        static bool isUnfocusedInput(QWidget* widget) {
            for (QWidget* w = widget; w != nullptr; w = w->parentWidget()) {
                if (qobject_cast<QAbstractSpinBox*>(w) != nullptr ||
                    qobject_cast<QComboBox*>(w) != nullptr ||
                    qobject_cast<QSlider*>(w) != nullptr) {
                    return !w->hasFocus();
                }
                if (w->isWindow()) {
                    break;
                }
            }
            return false;
        }
    };

} // namespace app
