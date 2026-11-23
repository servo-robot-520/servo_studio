#pragma once

#include <QAbstractButton>
#include <QEvent>
#include <QObject>
#include <QTimer>
#include <QWidget>

namespace app {

    /// @brief 鼠标点击按钮后清除焦点环的全局过滤器。
    ///
    /// QSS 只有 :focus 而没有 :focus-visible：鼠标点击同样触发焦点，
    /// 琥珀描边会滞留在按钮上。本过滤器在鼠标松开后清除按钮焦点，
    /// 但键盘 Tab 导航的焦点可见性不受影响。
    class ButtonFocusFilter : public QObject {
    public:
        using QObject::QObject;

        bool eventFilter(QObject* watched, QEvent* event) override {
            if (event->type() == QEvent::MouseButtonRelease &&
                qobject_cast<QAbstractButton*>(watched) != nullptr) {
                // 等点击信号分发完成后再清焦点，避免吞掉 clicked。
                QTimer::singleShot(0, [watched] {
                    if (auto* widget = qobject_cast<QWidget*>(watched)) {
                        widget->clearFocus();
                    }
                });
            }
            return QObject::eventFilter(watched, event);
        }
    };

} // namespace app
